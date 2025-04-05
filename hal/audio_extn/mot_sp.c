/**
 * Cirrus Logic Speaker Protection (CSPL) Implementation
 *
 * This file provides functions to handle the Cirrus CS35L41 DSP+AMP.
 * It includes functionality for applying calibration data from persist storage.
 */

#define LOG_TAG "audio_mot_sp"
/*#define LOG_NDEBUG 0*/

#include <errno.h>
#include <math.h>
#include <log/log.h>
#include <fcntl.h>
#include "../audio_hw.h"
#include "platform.h"
#include "platform_api.h"
#include <sys/stat.h>
#include <linux/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <cutils/properties.h>
#include "audio_extn.h"
#include "mot_sp.h"

// Global handle pointer for the cirrus playback session
struct cirrus_playback_session *handle = NULL;

/**
 * Read a numeric value from a file
 *
 * This function reads a single integer value from a file.
 * The file should contain just one number in text format.
 *
 * @param file_path     Path to the file to read
 * @param value_out     Pointer to store the read value
 * @return              0 on success, -EINVAL on failure
 */
int read_from_file(const char *file_path, uint32_t *value_out)
{
    FILE *fp;
    long file_size;
    char *buffer;
    char *endptr = NULL;
    long value;
    int old_errno;

    // Open the file in binary read mode
    fp = fopen(file_path, "rb");
    if (fp == NULL)
    {
        ALOGV("%s: failed to open: %s", __func__, file_path);
        return -EINVAL;
    }

    // Get file size
    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);

    // Check if file size is reasonable (less than 32 bytes) and no error occurred
    if (file_size > 0 && file_size < 32 && !ferror(fp))
    {
        // Go back to the start of the file
        rewind(fp);

        // Allocate buffer for file contents (including null terminator)
        buffer = (char *)calloc(file_size + 1, 1);
        if (buffer == NULL)
        {
            ALOGE("%s: memory allocation failure", __func__);
            fclose(fp);
            return -EINVAL;
        }

        // Read the file contents
        fgets(buffer, file_size + 1, fp);

        // Reset errno before conversion
        errno = 0;

        // Convert string to long integer
        value = strtol(buffer, &endptr, 0);
        old_errno = errno;

        if (old_errno == 0)
        {
            // Check for valid conversion (no trailing characters)
            if (*endptr == '\0')
            {
                // Successful conversion
                *value_out = (uint32_t)value;
                free(buffer);
                fclose(fp);
                return 0;
            }
            else
            {
                ALOGE("%s: %s: strtol() data corruption detected", __func__, "parse_strtol");
            }
        }
        else
        {
            ALOGE("%s: %s: strtol() error during conversion: %d", __func__, "parse_strtol", old_errno);
        }

        ALOGE("%s: strtol() parse failure", __func__);
        free(buffer);
    }
    else
    {
        ALOGE("%s: '%s' file read error:%d, length: %ld", __func__, file_path, ferror(fp), file_size);
    }

    fclose(fp);
    return -EINVAL;
}

/**
 * Apply speaker protection calibration
 *
 * @param type 0 for speaker, 1 for receiver
 */
void cspl_apply_calibration(device_identifier type)
{
    uint32_t cal_value = 0;
    uint32_t status_value = 0x1000000; // Status value (1 in big-endian format)
    unsigned int checksum_value = 0;
    char ctl_value[CIRRUS_CTL_NAME_BUF] = {0};
    struct audio_device *adev = handle->adev_handle;
    struct mixer_ctl *mixer_ctl = NULL;
    int retry_count = 0;
    int ret = 0;
    const char *device_str = (type == 0) ? "speaker" : "receiver";

    // Determine firmware control name based on device choice
    const char *firmware_ctl = (type == 0) ? SPK_FIRMWARE_CTL : RCV_FIRMWARE_CTL;

    // Check if firmware control exists
    mixer_ctl = mixer_get_ctl_by_name(adev->mixer, firmware_ctl);
    if (mixer_ctl == NULL)
    {
        ALOGW("%s: the firmware ctl %s is not found", __func__, firmware_ctl);
        return;
    }

    // Read calibration data from persist storage
    if (type == 0)
    {
        // Speaker path
        ret = read_from_file(SPK_CAL_FILE, &handle->spk->cal_data);
        if (ret != -EINVAL)
        {
            ret = read_from_file(SPK_AMBIENT_FILE, &handle->spk->cal_ambient);
            if (ret == -EINVAL)
            {
                goto use_default_cal;
            }
        }
        else
        {
            goto use_default_cal;
        }
    }
    else
    {
        // Receiver path
        ret = read_from_file(RCV_CAL_FILE, &handle->rcv->cal_data);
        if (ret != -EINVAL)
        {
            ret = read_from_file(RCV_AMBIENT_FILE, &handle->rcv->cal_ambient);
            if (ret == -EINVAL)
            {
                goto use_default_cal;
            }
        }
        else
        {
            goto use_default_cal;
        }
    }

    // Successfully read calibration from file, proceed to apply it
    goto apply_calibration;

use_default_cal:
    // Use default calibration value from properties
    {
        const char *prop_name = (type == 0) ? SPK_DEFAULT_RDC_PROP : RCV_DEFAULT_RDC_PROP;
        if (property_get(prop_name, ctl_value, "0") < 1)
        {
            ALOGWE"%s: Speaker Protection(CSPL) not calibrated on %s, use .bin default ReDC",
                  __func__, device_str);
            return;
        }

        if (type == 0)
        {
            handle->spk->cal_data = atoi(ctl_value);
            handle->spk->cal_ambient = CSPL_DEFAULT_CAL_AMBIENT;
        }
        else
        {
            handle->rcv->cal_data = atoi(ctl_value);
            handle->rcv->cal_ambient = CSPL_DEFAULT_CAL_AMBIENT;
        }

        ALOGW("%s: Speaker Protection(CSPL) not calibrated on %s, use default ReDC (%d)",
              __func__, device_str, (type == 0) ? handle->spk->cal_data : handle->rcv->cal_data);
    }

apply_calibration:
    // Start calibration
    {
        const char *cal_r_ctl = (type == 0) ? SPK_CAL_R_CTL : RCV_CAL_R_CTL;

        // Try to get the control with retries in case it's not immediately available
        for (retry_count = 0; retry_count < MAX_MIXER_CTL_RETRY; retry_count++)
        {
            mixer_ctl = mixer_get_ctl_by_name(adev->mixer, cal_r_ctl);
            if (mixer_ctl != NULL)
            {
                break;
            }

            ALOGW("%s: Speaker Protection(CSPL) ctl %s not found, update ctl and retry",
                  __func__, cal_r_ctl);
            usleep(100000); // 100ms delay

            ret = mixer_add_new_ctls(adev->mixer);
            if (ret != 0)
            {
                ALOGV("%s: mixer_add_new_ctls return failure, ret %d", __func__, ret);
            }
        }

        if (mixer_ctl == NULL)
        {
            ALOGE("%s: ctl %s not found, failed to load Speaker Protection(CSPL) speaker calibration",
                  __func__, cal_r_ctl);
            return;
        }

        // Convert calibration value to big-endian format
        if (type == 0)
        {
            cal_value = ((handle->spk->cal_data & 0xFF) << 24) |
                        (((handle->spk->cal_data >> 8) & 0xFF) << 16) |
                        (((handle->spk->cal_data >> 16) & 0xFF) << 8) |
                        ((handle->spk->cal_data >> 24) & 0xFF);
        }
        else
        {
            cal_value = ((handle->rcv->cal_data & 0xFF) << 24) |
                        (((handle->rcv->cal_data >> 8) & 0xFF) << 16) |
                        (((handle->rcv->cal_data >> 16) & 0xFF) << 8) |
                        ((handle->rcv->cal_data >> 24) & 0xFF);
        }

        ret = mixer_ctl_set_array(mixer_ctl, &cal_value, 1);
        if (ret != 0)
        {
            ALOGE("%s: Failed to set speaker calibration %s", __func__, cal_r_ctl);
            return;
        }

        if (type == 0)
            handle->spk->cal_ok = true;
        else
            handle->rcv->cal_ok = true;

        ALOGI("%s: write %s Protection(CSPL) speaker calibration %x %x %x %x (big-endian)",
              __func__, device_str,
              cal_value & 0xFF,
              (cal_value >> 8) & 0xFF,
              (cal_value >> 16) & 0xFF,
              (cal_value >> 24) & 0xFF);

        const char *cal_status_ctl = (type == 0) ? SPK_CAL_STATUS_CTL : RCV_CAL_STATUS_CTL;
        mixer_ctl = mixer_get_ctl_by_name(adev->mixer, cal_status_ctl);
        ret = mixer_ctl_set_array(mixer_ctl, &status_value, 1);
        if (ret != 0)
        {
            ALOGE("%s: Failed to set speaker calibration status %s", __func__, cal_status_ctl);
            return;
        }

        ALOGI("%s: write %s Protection(CSPL) speaker calibration status as 1", __func__, device_str);

        // Set checksum (original value + 1)
        const char *cal_checksum_ctl = (type == 0) ? SPK_CAL_CHECKSUM_CTL : RCV_CAL_CHECKSUM_CTL;
        mixer_ctl = mixer_get_ctl_by_name(adev->mixer, cal_checksum_ctl);

        // Checksum is original value + 1, converted to big endian
        uint32_t checksum = ((type == 0) ? handle->spk->cal_data : handle->rcv->cal_data) + 1;
        checksum_value = ((checksum & 0xFF) << 24) |
                         (((checksum >> 8) & 0xFF) << 16) |
                         (((checksum >> 16) & 0xFF) << 8) |
                         ((checksum >> 24) & 0xFF);

        ret = mixer_ctl_set_array(mixer_ctl, &checksum_value, 1);
        if (ret != 0)
        {
            ALOGE("%s: Failed to set speaker calibration checksum %s", __func__, cal_checksum_ctl);
            return;
        }

        ALOGI("%s: write %s Protection(CSPL) speaker calibration checksum %x %x %x %x (big-endian)",
              __func__, device_str,
              checksum_value & 0xFF,
              (checksum_value >> 8) & 0xFF,
              (checksum_value >> 16) & 0xFF,
              (checksum_value >> 24) & 0xFF);

        // Set boot switch to 1
        const char *boot_switch_ctl = (type == 0) ? SPK_BOOT_SWITCH_CTL : RCV_BOOT_SWITCH_CTL;
        mixer_ctl = mixer_get_ctl_by_name(adev->mixer, boot_switch_ctl);
        if (mixer_ctl == NULL)
        {
            ALOGW("%s: has not boot switch contol", boot_switch_ctl);
            return;
        }

        ret = mixer_ctl_set_value(mixer_ctl, 0, 1);
        if (ret != 0)
        {
            ALOGE("%s: Failed to set Boot Switch, ctl: %s", __func__, boot_switch_ctl);
        }
    }
}

/**
 * Initialize the CSPL speaker protection calibration system
 *
 * @return 0 on success, non-zero on failure
 */
void audio_extn_spkr_prot_calib_init(void)
{
    cspl_apply_calibration(SPEAKER);  // Try CSPL speaker
    cspl_apply_calibration(RECEIVER); // Try CSPL receiver

    return;
}

void *audio_extn_mot_spkr_prot_init(void *adev)
{
    struct cirrus_playback_session *handle = (struct cirrus_playback_session *)calloc(1, sizeof(struct cirrus_playback_session));
    if (handle == NULL)
    {
        ALOGE("%s: memory allocation failure: mot_prot_data", __func__);
        return NULL;
    }

    handle->adev_handle = adev;

    handle->spk = (struct cirrus_cal_t *)calloc(1, sizeof(struct cirrus_cal_t));
    if (handle->spk == NULL)
    {
        ALOGE("%s: memory allocation failure: spk", __func__);
        free(handle);
        return NULL;
    }

    handle->rcv = (struct cirrus_cal_t *)calloc(1, sizeof(struct cirrus_cal_t));
    if (handle->rcv == NULL)
    {
        ALOGE("%s: memory allocation failure: rcv", __func__);
        free(handle->spk);
        free(handle);
        return NULL;
    }

    audio_extn_spkr_prot_calib_init();

    return handle;
}

void audio_extn_mot_spkr_prot_deinit(void *mot_handle)
{
    struct cirrus_playback_session *handle = (struct cirrus_playback_session *)mot_handle;

    if (handle == NULL)
    {
        return;
    }

    if (handle->spk != NULL)
    {
        free(handle->spk);
    }

    if (handle->rcv != NULL)
    {
        free(handle->rcv);
    }

    free(handle);
}
