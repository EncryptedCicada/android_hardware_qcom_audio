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

// - external function dependency -
static fp_platform_get_snd_device_name_t fp_platform_get_snd_device_name;
static fp_platform_get_pcm_device_id_t fp_platform_get_pcm_device_id;
static fp_get_usecase_from_list_t fp_get_usecase_from_list;
static fp_enable_disable_snd_device_t fp_disable_snd_device;
static fp_enable_disable_snd_device_t fp_enable_snd_device;
static fp_enable_disable_audio_route_t fp_disable_audio_route;
static fp_enable_disable_audio_route_t fp_enable_audio_route;
static fp_platform_check_and_set_codec_backend_cfg_t fp_platform_check_and_set_codec_backend_cfg;

typedef enum
{
    SPEAKER = 0,
    RECEIVER = 1,
} device_identifier;

// CSPL calibration file paths
#define SPK_CAL_FILE "/mnt/vendor/persist/factory/audio/spk_cal_r"
#define SPK_AMBIENT_FILE "/mnt/vendor/persist/factory/audio/spk_ambient"
#define RCV_CAL_FILE "/mnt/vendor/persist/factory/audio/rcv_cal_r"
#define RCV_AMBIENT_FILE "/mnt/vendor/persist/factory/audio/rcv_ambient"

// Property names
#define SPK_DEFAULT_RDC_PROP "persist.vendor.audio.default.spkrdc"
#define RCV_DEFAULT_RDC_PROP "persist.vendor.audio.default.rcvrdc"

// Control names
#define SPK_FIRMWARE_CTL "SPK DSP1 Firmware"
#define RCV_FIRMWARE_CTL "RCV DSP1 Firmware"
#define SPK_CAL_R_CTL "SPK DSP1X protection cd CAL_R"
#define RCV_CAL_R_CTL "RCV DSP1X protection cd CAL_R"
#define SPK_CAL_STATUS_CTL "SPK DSP1X protection cd CAL_STATUS"
#define RCV_CAL_STATUS_CTL "RCV DSP1X protection cd CAL_STATUS"
#define SPK_CAL_CHECKSUM_CTL "SPK DSP1X protection cd CAL_CHECKSUM"
#define RCV_CAL_CHECKSUM_CTL "RCV DSP1X protection cd CAL_CHECKSUM"
#define SPK_BOOT_SWITCH_CTL "SPK DSP1 Boot Switch"
#define RCV_BOOT_SWITCH_CTL "RCV DSP1 Boot Switch"

#define CSPL_DEFAULT_CAL_AMBIENT 28

// Max retry attempts for control lookup
#define MAX_MIXER_CTL_RETRY 30

#define CIRRUS_CTL_NAME_BUF 96
#define CRUS_CONFIG_FILE_SIZE 128

struct pcm_config pcm_config_cirrus_rx = {
    .channels = 2,
    .rate = 48000,
    .period_size = 240,
    .period_count = 4,
    .format = PCM_FORMAT_S16_LE,
    .start_threshold = 0,
    .stop_threshold = 0,
    .silence_threshold = 0,
    .avail_min = 0,
};

// Payload struct for getting calibration result from DSP module
struct __attribute__((__packed__)) cirrus_cal_t
{
    uint32_t cal_data;
    uint32_t cal_ambient;
};

// Cirrus playback session structure
struct cirrus_playback_session
{
    void *adev_handle;
    pthread_mutex_t fb_prot_mutex;
    pthread_t calibration_thread;
    struct cirrus_cal_t dev;
    struct pcm *pcm_rx;
    bool cal_ok;
};

// Global handle for the cirrus playback session
static struct cirrus_playback_session handle;

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
 * Load the Cirrus Logic Speaker Enhancement configuration
 *
 * This function configures the Cirrus SE DSP by:
 * 1. Setting the appropriate audio mode and index
 * 2. Loading the configuration files
 * 3. Checking for external configurations before using internal ones
 *
 * @return 0 on success, -EINVAL on failure
 */
int cspl_se_load_usecase_configs()
{
    struct audio_device *adev = handle.adev_handle;
    struct mixer_ctl *ctl_audio_mode = NULL;
    struct mixer_ctl *ctl_audio_index = NULL;
    struct mixer_ctl *ctl_load_config = NULL;
    int num_enums = 0;
    int current_index = 0;
    int max_index = 0;
    int config_start_index = 0;
    int ret = 0;
    char config_path[CRUS_CONFIG_FILE_SIZE] = {0};

    // Get the mixer controls for Cirrus SE
    ctl_audio_mode = mixer_get_ctl_by_name(adev->mixer, "Cirrus SE Audio Mode");
    ctl_audio_index = mixer_get_ctl_by_name(adev->mixer, "Cirrus SE Audio Index");
    ctl_load_config = mixer_get_ctl_by_name(adev->mixer, "Cirrus SE Load Config");

    // Verify all required controls exist
    if (!ctl_audio_mode || !ctl_audio_index || !ctl_load_config)
    {
        ALOGE("%s: Could not get ctl for mixer commands", __func__);
        return -EINVAL;
    }

    // Get the number of available audio modes
    num_enums = mixer_ctl_get_num_enums(ctl_audio_mode);

    // Get the current audio index
    current_index = mixer_ctl_get_value(ctl_audio_index, 0);
    if (current_index < 0)
    {
        ALOGE("%s: Could not get ctl for usecase index", __func__);
        return -EINVAL;
    }

    // Calculate the range of configuration indices
    config_start_index = current_index;
    max_index = current_index + num_enums;

    // First, check if external configuration files exist
    for (int i = config_start_index; i < max_index; i++)
    {
        // Construct path to check for external config file
        sprintf(config_path, "/vendor/firmware/crus_sp_rx%d.bin", i);

        // Check if the file exists
        if (access(config_path, R_OK) != 0)
        {
            // File doesn't exist, no external config available
            ALOGI("%s: CSPL SE doesn't have external config available, use internal", __func__);
            return 0;
        }
    }

    // External configurations exist, load them
    for (int i = 0; i < num_enums; i++)
    {
        // Set the audio mode
        mixer_ctl_set_value(ctl_audio_mode, 0, i);

        // Trigger configuration loading
        mixer_ctl_set_value(ctl_load_config, 0, 1);

        ALOGI("%s: CSPL SE crus_sp_rx%d.bin loaded for Usecase %d",
              __func__, config_start_index + i, i);
    }

    // Reset to the original audio mode
    mixer_ctl_set_value(ctl_audio_mode, 0, config_start_index);

    ALOGI("%s: CSPL Speaker Enhancement loaded external configuration", __func__);

    return 0;
}

/**
 * Thread function for loading Cirrus Logic Speaker Enhancement parameters
 *
 * This function runs as a background thread to set up the Cirrus DSP with
 * speaker enhancement parameters. It:
 * 1. Waits for the audio subsystem to be fully initialized
 * 2. Creates an audio usecase for the Cirrus DSP
 * 3. Enables the sound device and audio route
 * 4. Opens a PCM device for communication with the DSP
 * 5. Loads the speaker enhancement configuration
 * 6. Cleans up and exits
 *
 * @return NULL (pthread exit)
 */
void *cspl_se_parameter_loading_thread()
{
    struct audio_device *adev = handle.adev_handle;
    struct audio_usecase *usecase = NULL;
    int pcm_device_id;
    int retry_count = 10;
    int ret;

    /* Wait for audio subsystem to be ready */
    while (!adev->platform)
    {
        ALOGI("%s: Waiting for audio device...", __func__);
        sleep(1);
    }

    /* Check if Cirrus Speaker Enhancement is supported */
    if (!mixer_get_ctl_by_name(adev->mixer, "Cirrus SE"))
    {
        pthread_exit(NULL);
    }

    /* Allocate and initialize usecase structure */
    usecase = (struct audio_usecase *)calloc(1, sizeof(struct audio_usecase));
    if (!usecase)
    {
        ALOGE("%s: Failed to allocate memory for usecase", __func__);
        pthread_exit(NULL);
    }

    usecase->id = USECASE_AUDIO_PLAYBACK_LOW_LATENCY; /* Assuming this is the correct usecase ID (value 1) */
    usecase->type = PCM_PLAYBACK;
    usecase->in_snd_device = SND_DEVICE_NONE;
    usecase->stream.out = adev->primary_output;
    list_init(&usecase->device_list);
    usecase->out_snd_device = SND_DEVICE_OUT_SPEAKER_EXTERNAL_1;

    /* Lock audio device mutex before modifying usecase list */
    pthread_mutex_lock(&adev->lock);

    /* Retry loop for PCM device setup */
    for (; retry_count > 0; retry_count--)
    {
        /* Add usecase to the list */
        list_add_tail(&adev->usecase_list, &usecase->list);

        /* Enable sound device and route */
        fp_enable_snd_device(adev, usecase->out_snd_device);
        fp_enable_audio_route(adev, usecase);

        /* Get PCM device ID for this usecase */
        pcm_device_id = fp_platform_get_pcm_device_id(usecase->id, PCM_PLAYBACK);
        if (pcm_device_id < 0)
        {
            ALOGE("%s: Invalid pcm device for usecase (%d)", __func__, usecase->id);
            goto cleanup;
        }

        /* Open PCM device */
        handle.pcm_rx = pcm_open(adev->snd_card, pcm_device_id, PCM_OUT, &pcm_config_cirrus_rx);
        if (!handle.pcm_rx || !pcm_is_ready(handle.pcm_rx))
        {
            ALOGE("%s: PCM device not ready: %s : wait 10ms and retry (%d)",
                  __func__, handle.pcm_rx ? pcm_get_error(handle.pcm_rx) : "unknown", retry_count);

            if (handle.pcm_rx)
            {
                pcm_close(handle.pcm_rx);
                handle.pcm_rx = NULL;
            }

            /* Remove usecase from list and retry */
            fp_disable_snd_device(adev, usecase->out_snd_device);
            fp_disable_audio_route(adev, usecase);
            list_remove(&usecase->list);

            usleep(10000); /* 10ms */
            continue;
        }

        /* Start PCM device */
        ret = pcm_start(handle.pcm_rx);
        if (ret < 0)
        {
            ALOGE("%s: pcm start for RX failed: %s : wait 1s and retry (%d)",
                  __func__, pcm_get_error(handle.pcm_rx), retry_count);

            if (handle.pcm_rx)
            {
                pcm_close(handle.pcm_rx);
                handle.pcm_rx = NULL;
            }

            /* Remove usecase from list and retry */
            fp_disable_snd_device(adev, usecase->out_snd_device);
            fp_disable_audio_route(adev, usecase);
            list_remove(&usecase->list);

            sleep(1); /* 1 second */
            continue;
        }

        /* Successfully opened and started PCM device */
        break;
    }

    /* If retries exhausted, clean up and exit */
    if (retry_count <= 0)
    {
        goto cleanup;
    }

    /* Load speaker enhancement configurations */
    ret = cspl_se_load_usecase_configs();
    if (ret < 0)
    {
        ALOGE("%s: Set tuning configs failed (%d)", __func__, ret);
    }

cleanup:
    /* Clean up resources */
    if (handle.pcm_rx)
    {
        pcm_close(handle.pcm_rx);
        handle.pcm_rx = NULL;
    }

    /* Disable sound device and route */
    fp_disable_snd_device(adev, usecase->out_snd_device);
    fp_disable_audio_route(adev, usecase);

    /* Remove usecase from list */
    list_remove(&usecase->list);

    /* Unlock audio device mutex */
    pthread_mutex_unlock(&adev->lock);

    /* Free allocated memory */
    if (usecase)
    {
        free(usecase);
    }

    pthread_exit(NULL);
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
    struct audio_device *adev = handle.adev_handle;
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
        ALOGE("%s: the firmware ctl %s is not found", __func__, firmware_ctl);
        return;
    }

    // Read calibration data from persist storage
    if (type == 0)
    {
        // Speaker path
        ret = read_from_file(SPK_CAL_FILE, &handle.dev.cal_data);
        if (ret != -EINVAL)
        {
            ret = read_from_file(SPK_AMBIENT_FILE, &handle.dev.cal_ambient);
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
        ret = read_from_file(RCV_CAL_FILE, &handle.dev.cal_data);
        if (ret != -EINVAL)
        {
            ret = read_from_file(RCV_AMBIENT_FILE, &handle.dev.cal_ambient);
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
            ALOGE("%s: Speaker Protection(CSPL) not calibrated on %s, use .bin default ReDC",
                  __func__, device_str);
            ALOGE("%s: error in reading calibration value", __func__);
            return;
        }

        handle.dev.cal_data = atoi(ctl_value);
        handle.dev.cal_ambient = CSPL_DEFAULT_CAL_AMBIENT;

        ALOGE("%s: Speaker Protection(CSPL) not calibrated on %s, use default ReDC (%d)",
              __func__, device_str, handle.dev.cal_data);
    }

apply_calibration:
    const char *cal_r_ctl = (type == 0) ? SPK_CAL_R_CTL : RCV_CAL_R_CTL;

    // Try to get the control with retries in case it's not immediately available
    for (retry_count = 0; retry_count < MAX_MIXER_CTL_RETRY; retry_count++)
    {
        mixer_ctl = mixer_get_ctl_by_name(adev->mixer, cal_r_ctl);
        if (mixer_ctl != NULL)
        {
            break;
        }

        ALOGV("%s: Speaker Protection(CSPL) ctl %s not found, update ctl and retry",
              __func__, cal_r_ctl);
        usleep(100000); // 100ms delay

        ret = mixer_update_ctls(adev->mixer);
        if (ret != 0)
        {
            ALOGV("mixer_update_ctls return failure, ret %d", ret);
        }
    }

    if (mixer_ctl == NULL)
    {
        ALOGE("%s: ctl %s not found, failed to load Speaker Protection(CSPL) speaker calibration",
              __func__, cal_r_ctl);
        return;
    }

    // Convert calibration value to big-endian format
    cal_value = ((handle.dev.cal_data & 0xFF) << 24) |
                (((handle.dev.cal_data >> 8) & 0xFF) << 16) |
                (((handle.dev.cal_data >> 16) & 0xFF) << 8) |
                ((handle.dev.cal_data >> 24) & 0xFF);

    ret = mixer_ctl_set_array(mixer_ctl, &cal_value, 1);
    if (ret != 0)
    {
        ALOGE("%s: Failed to set speaker calibration %s", __func__, cal_r_ctl);
        return;
    }

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
    uint32_t checksum = handle.dev.cal_data + 1;
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
        ALOGV("%s: has not boot switch contol", boot_switch_ctl);
        return;
    }

    ret = mixer_ctl_set_value(mixer_ctl, 0, 1);
    if (ret != 0)
    {
        ALOGE("%s: Failed to set Boot Switch, ctl: %s", __func__, boot_switch_ctl);
    }
}

/**
 * Initialize the CSPL speaker protection calibration system
 *
 * @return 0 on success, non-zero on failure
 */
int spkr_prot_calib_init()
{
    int ret = 0;

    // Check for CSPL (Cirrus Logic Speaker Protection)
    ret = cspl_apply_calibration(SPEAKER); // Try speaker
    if (ret == 0)
    {
        ret = cspl_apply_calibration(RECEIVER); // Try receiver
        if (ret == 0)
        {
            handle.cal_ok = true; // CSPL protection status
            return 0;
        }
    }

    // CSPL supported speaker protection not detected
    return -EINVAL;
}

void spkr_prot_init(void *adev)
{

    if (!adev)
    {
        ALOGE("%s: CIRRUS: Invalid params", __func__);
        return;
    }

    memset(&handle, 0, sizeof(handle));
    handle.adev_handle = adev;

    // init function pointers
    fp_platform_get_snd_device_name = spkr_prot_init_config_val.fp_platform_get_snd_device_name;
    fp_platform_get_pcm_device_id = spkr_prot_init_config_val.fp_platform_get_pcm_device_id;
    fp_get_usecase_from_list = spkr_prot_init_config_val.fp_get_usecase_from_list;
    fp_disable_snd_device = spkr_prot_init_config_val.fp_disable_snd_device;
    fp_enable_snd_device = spkr_prot_init_config_val.fp_enable_snd_device;
    fp_disable_audio_route = spkr_prot_init_config_val.fp_disable_audio_route;
    fp_enable_audio_route = spkr_prot_init_config_val.fp_enable_audio_route;
    fp_platform_check_and_set_codec_backend_cfg = spkr_prot_init_config_val.fp_platform_check_and_set_codec_backend_cfg;

    spkr_prot_calib_init();

    pthread_mutex_init(&handle.fb_prot_mutex, NULL);

    (void)pthread_create(&handle.calibration_thread, (const pthread_attr_t *)NULL, cspl_se_parameter_loading_thread, &handle);
}

/**
 * Deinitialize the speaker protection system
 *
 * This function cleans up resources used by the Cirrus speaker protection system.
 * It closes the PCM device and frees any allocated memory.
 *
 * @return 0 on success, non-zero on failure
 */
int spkr_prot_deinit()
{
    int ret = 0;

    // Clean up the PCM device if still open
    if (handle.pcm_rx)
    {
        pcm_close(handle.pcm_rx);
        handle.pcm_rx = NULL;
    }

    // Clean up thread resources
    if (handle.calibration_thread)
    {
        pthread_join(handle.calibration_thread, NULL);
        handle.calibration_thread = 0;
    }

    // Destroy mutex
    pthread_mutex_destroy(&handle.fb_prot_mutex);

    // Reset handle
    memset(&handle, 0, sizeof(handle));

    ALOGI("%s: Speaker protection deinitialized", __func__);

    return ret;
}

/**
 * Implementation for the speaker protection start processing
 *
 * This function handles mapping regular speaker devices to their
 * protected variants and enabling the necessary processing.
 *
 * @param snd_device The sound device ID
 * @return 0 on success, non-zero on failure
 */
int spkr_prot_start_processing(snd_device_t snd_device)
{
    int ret = 0;
    snd_device_t protected_snd_device;

    // Check if speaker protection is enabled
    if (!handle.cal_ok)
    {
        return 0;
    }

    // Map the regular sound device to its protected variant
    protected_snd_device = get_spkr_prot_snd_device(snd_device);

    // If the device isn't changed, no protection needed
    if (protected_snd_device == snd_device)
    {
        return 0;
    }

    ALOGI("%s: Processing started for snd_device=%d (protected=%d)",
          __func__, snd_device, protected_snd_device);

    // Additional processing logic would go here

    return ret;
}

/* Taken directly from cirrus_sony since decompilation is not good */
int get_spkr_prot_snd_device(snd_device_t snd_device)
{
    switch (snd_device)
    {
    case SND_DEVICE_OUT_SPEAKER:
    case SND_DEVICE_OUT_SPEAKER_REVERSE:
        return SND_DEVICE_OUT_SPEAKER_PROTECTED;
    case SND_DEVICE_OUT_SPEAKER_SAFE:
        return SND_DEVICE_OUT_SPEAKER_SAFE;
    case SND_DEVICE_OUT_VOICE_SPEAKER:
        return SND_DEVICE_OUT_VOICE_SPEAKER_PROTECTED;
    default:
        return snd_device;
    }
}

/* Dummy, since decompilation is not good */
int spkr_prot_stop_processing(__unused snd_device_t snd_device)
{
    return 0;
}

void spkr_prot_calib_cancel(__unused void *adev)
{
    return;
}

/* Taken directly from cirrus_sony since decompilation is not good */
bool spkr_prot_is_enabled()
{
    return true;
}