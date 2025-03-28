/*
 * Refined Speaker Protection (CSPL) Implementation
 *
 * This file is derived from disassembled stock HAL code.
 *
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
#include <linux/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <cutils/properties.h>
#include "audio_extn.h"

// - external function dependency -
static fp_platform_get_snd_device_name_t fp_platform_get_snd_device_name;
static fp_platform_get_pcm_device_id_t fp_platform_get_pcm_device_id;
static fp_get_usecase_from_list_t fp_get_usecase_from_list;
static fp_enable_disable_snd_device_t fp_disable_snd_device;
static fp_enable_disable_snd_device_t  fp_enable_snd_device;
static fp_enable_disable_audio_route_t fp_disable_audio_route;
static fp_enable_disable_audio_route_t fp_enable_audio_route;
static fp_audio_extn_get_snd_card_split_t fp_audio_extn_get_snd_card_split;

/* Payload struct for getting calibration result from DSP module */
struct __attribute__((__packed__)) cirrus_cal_result_t {
    uint8_t status[4];
    uint8_t checksum[4];
    uint8_t cal_r[4];
    bool cal_ok;
};

struct cirrus_playback_session {
    void *adev_handle;
    pthread_mutex_t fb_prot_mutex;
    pthread_t calibration_thread;
    struct pcm *pcm_rx;
    struct pcm *pcm_tx;
    struct cirrus_cal_result_t spk;
    volatile int32_t state;
};

enum cirrus_playback_state {
    INIT = 0,
    CALIBRATING = 1,
    CALIBRATION_ERROR = 2,
    IDLE = 3,
    PLAYBACK = 4
};

struct pcm_config pcm_config_cirrus_rx = {
    .channels = 8,
    .rate = 48000,
    .period_size = 320,
    .period_count = 4,
    .format = PCM_FORMAT_S32_LE,
    .start_threshold = 0,
    .stop_threshold = INT_MAX,
    .avail_min = 0,
};

#define PERSIST_CIRRUS_CAL_SPK_CAL_AMBIENT "/mnt/vendor/persist/factory/audio/spk_ambient"
#define PERSIST_CIRRUS_CAL_SPK_CAL_R "/mnt/vendor/persist/factory/audio/spk_cal_r"
#define PERSIST_CIRRUS_CAL_SPK_CAL_F0 "/mnt/vendor/persist/factory/audio/spk_f0"

#define CIRRUS_DEFAULT_CSPL_REDC "0"

/* Mixer Controls */
#define CIRRUS_CTL_SPK_FORCE_WAKE "SPK Hibernate Force Wake"
#define CIRRUS_CTL_SPK_CCM_RESET "SPK CCM Reset"
#define CIRRUS_CTL_SPK_PROT_CAL_R "SPK DSP1X protection cd CAL_R"
#define CIRRUS_CTL_SPK_PROT_CAL_STATUS "SPK DSP1X protection cd CAL_STATUS"
#define CIRRUS_CTL_SPK_PROT_CAL_CHECKSUM "SPK DSP1X protection cd CAL_CHECKSUM"
#define CIRRUS_CTL_SPK_PROT_CSPL_ERRORNO "SPK DSP1X protection cd CSPL_ERRORNO"

#define CRUS_RX_CONF_FILE "vendor/firmware/crus_sp_config_%s_rx.bin"
#define CONFIG_FILE_SIZE 128

#define CIRRUS_CTL_NAME_BUF 96

#define CIRRUS_ERROR_DETECT_SLEEP_US 250000
#define CIRRUS_FIRMWARE_LOAD_SLEEP_US 5000
#define CIRRUS_FIRMWARE_MAX_RETRY 30

/* Global variables */
static struct cirrus_playback_session handle;

/*
 * Function: audio_extn_read_from_file
 *
 * Reads a numerical value from the file at 'path' and stores it in *value.
 * Returns 0 for success, but <0 values like -EINVAL or -ENOMEM etc, whichever is suitable, otherwise.
 */
int audio_extn_read_from_file(char *path, long *value)
{
    int ret = 0;
    FILE *stream = fopen(path, "rb");
    if (!stream) {
        ALOGE("%s: failed to open: %s", __func__, path);
        ret = -ENOENT; // No such file or directory
        goto early_exit:
    }
    
    fseek(stream, 0, SEEK_END);
    long len = ftell(stream);
    
    if (ferror(stream) != 0) {
        ALOGE("%s: '%s' file seek error", __func__, path);
        ret = -EIO; // Input/output error
        goto check_error;
    }
    
    if ((len - 1U) >= 0x20) {
        ALOGE("%s: '%s' file too large", __func__, path);
        ret = -EFBIG; // File too big
        goto check_error;
    }
    
    rewind(stream);
    char *buffer = calloc(len + 1, 1);
    if (!buffer) {
        ALOGE("%s: memory allocation failure", __func__, path);
        ret = -ENOMEM; // Not enough memory
        goto check_error;
    }
    
    if (fgets_unlocked(buffer, len + 1, stream) == NULL) {
        ALOGE("%s: '%s' file read error", __func__, path);
        ret = -EIO; // Input/output error
        goto exit;
    }
    
    errno = 0;
    char *endptr = NULL;
    long val = strtol(buffer, &endptr, 0);
    
    if (errno != 0 || *endptr != '\0') {
        ALOGE("%s: strtol() parse failure", __func__, path);
        ret = -EINVAL; // Invalid argument (parsing error)
        goto exit;
    }
    
    *value = val;
exit:
    free(buffer);
check_error:
    fclose(stream);
early_exit:
    return ret;
}

/* TODO: This function assumes that we are always using CARD 0 */
static int cirrus_set_mixer_value_by_name(char* ctl_name, int value) {
    struct mixer *card_mixer = NULL;
    struct mixer_ctl *ctl_config = NULL;
    int sndcard_id = 0, ret = 0;

    card_mixer = mixer_open(sndcard_id);
    if (!card_mixer) {
        ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
        return -1;
    }

    ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
    if (!ctl_config) {
        ALOGD("%s: Cannot get mixer control %s", __func__, ctl_name);
        ret = -1;
        goto exit;
    }

    ret = mixer_ctl_set_value(ctl_config, 0, value);
    if (ret < 0)
        ALOGE("%s: Cannot set mixer '%s' to '%d'",
              __func__, ctl_name, value);
exit:
    mixer_close(card_mixer);
    return ret;
}

static int cirrus_get_mixer_value_by_name(char* ctl_name) {
    struct mixer *card_mixer = NULL;
    struct mixer_ctl *ctl_config = NULL;
    int sndcard_id = 0, ret = -EINVAL;

    card_mixer = mixer_open(sndcard_id);
    if (!card_mixer) {
        ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
        return -1;
    }

    ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
    if (!ctl_config) {
        ALOGE("%s: Cannot get mixer control %s", __func__, ctl_name);
        ret = -1;
        goto exit;
    }

    ret = mixer_ctl_get_value(ctl_config, 0);
    if (ret < 0)
        ALOGE("%s: Cannot get mixer %s value: error %d",
              __func__, ctl_name, ret);
exit:
    mixer_close(card_mixer);
    return ret;
}

static int cirrus_set_mixer_array_by_name(char* ctl_name, void* array, size_t count) {
    struct mixer *card_mixer = NULL;
    struct mixer_ctl *ctl_config = NULL;
    int sndcard_id = 0, ret = 0;

    card_mixer = mixer_open(sndcard_id);
    if (!card_mixer) {
        ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
        return -1;
    }

    ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
    if (!ctl_config) {
        ALOGD("%s: Cannot get mixer control %s", __func__, ctl_name);
        ret = -1;
        goto exit;
    }

    ret = mixer_ctl_set_array(ctl_config, array, count);
    if (ret < 0)
        ALOGE("%s: Cannot set mixer %s",
              __func__, ctl_name);
exit:
    mixer_close(card_mixer);
    return ret;
}

static int cirrus_set_mixer_enum_by_name(char* ctl_name, const char* value) {
    struct mixer *card_mixer = NULL;
    struct mixer_ctl *ctl_config = NULL;
    int sndcard_id = 0, ret = 0;

    card_mixer = mixer_open(sndcard_id);
    if (!card_mixer) {
        ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
        return -1;
    }

    ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
    if (!ctl_config) {
        ALOGE("%s: Cannot get mixer control '%s'", __func__, ctl_name);
        ret = -1;
        goto exit;
    }

    ret = mixer_ctl_set_enum_by_string(ctl_config, value);
    if (ret < 0)
        ALOGE("%s: Cannot set mixer '%s' to '%s'",
              __func__, ctl_name, value);
exit:
    mixer_close(card_mixer);
    return ret;
}

static int cirrus_do_reset() {
    int ret = 0;

    ret = cirrus_get_mixer_value_by_name(CIRRUS_CTL_SPK_CCM_RESET);
    if (ret < 0) {
        ALOGE("%s: CCM Reset is missing!!!", __func__);
    } else {
        ret = cirrus_set_mixer_value_by_name(CIRRUS_CTL_SPK_CCM_RESET, 1);
        ALOGI("%s: CCM Reset done.", __func__);
    }

    return ret;
}

static int cirrus_mixer_wait_for_setting(char *ctl, int val, int retry)
{
    int i, ret;

    for (i = 0; i < retry; i++) {
        /* Start firmware download sequence: shut down DSP and reset states */
        ret = cirrus_get_mixer_value_by_name(ctl);
        if (ret < 0 || ret == val)
            break;

        usleep(10000);
    }
    if (ret < 0 && i == retry)
        return -ETIMEDOUT;

    return ret;
}

void prepare_control_name(char *buffer, size_t buffer_size, const char *name)
{
    if (buffer == NULL || name == NULL) {
        return;
    }
    
    // Clear the buffer
    memset(buffer, 0, buffer_size);
    
    // Copy the desired name into the buffer
    snprintf(buffer, buffer_size, "%s", name);
}

static int cirrus_exec_prot_fw_download(int do_reset) {
    char ctl_name[CIRRUS_CTL_NAME_BUF];
    uint8_t cspl_ena[4] = { 0 };
    int retry = 0, ret;

    ALOGD("%s: Asking for speaker firmware %s", __func__, (do_reset ? "with reset" : "without reset"));
    if (do_reset)
        ret = cirrus_do_reset();

    /* If this one is missing, we're not using our Cirrus codec... */
    prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP Booted");
    ret = cirrus_get_mixer_value_by_name(ctl_name);
    if (ret < 0) {
        ALOGE("%s: %s control is missing. Bailing out.", __func__, ctl_name);
        ret = -ENODEV;
        goto exit;
    }

    /* Start firmware download sequence: shut down DSP and reset states */
    ret = cirrus_set_mixer_value_by_name(ctl_name, 0);
    if (ret < 0) {
        ALOGE("%s: Cannot reset %s status", __func__, ctl_name);
        goto exit;
    }

    ret = cirrus_mixer_wait_for_setting(ctl_name, 0, 10);
    if (ret < 0) {
        ALOGE("%s: %s wait setting error %d", __func__, ctl_name, ret);
        goto exit;
    }

    usleep(10000);

    prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1 Preload Switch");
    ret = cirrus_set_mixer_value_by_name(ctl_name, 0);
    if (ret < 0) {
        ALOGE("%s: Cannot reset %s", __func__, ctl_name);
        goto exit;
    }

    ret = cirrus_mixer_wait_for_setting(ctl_name, 0, 10);
    if (ret < 0) {
        ALOGE("%s: %s wait setting error %d", __func__, ctl_name, ret);
        goto exit;
    }

    usleep(10000);

    /* Determine what firmware to load and configure DSP */
    prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1 Firmware");
    ret = cirrus_set_mixer_enum_by_name(ctl_name, "protection");
    if (ret < 0) {
        ALOGE("%s: Cannot set %s to protection", __func__, ctl_name);
        goto exit;
    }

    prepare_control_name(ctl_name, sizeof(ctl_name), "SPK PCM Source");
    ret = cirrus_set_mixer_enum_by_name(ctl_name, "DSP");
    if (ret < 0) {
        ALOGE("%s: Cannot set %s to DSP", __func__, ctl_name);
        goto exit;
    }

    /* Send the firmware! */
    prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1 Preload Switch");
    ret = cirrus_set_mixer_value_by_name(ctl_name, 1);
    if (ret < 0) {
        ALOGE("%s: Cannot set %s to %s", __func__, ctl_name, fw_type);
        goto exit;
    }

    prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1X protection cd CSPL_ENABLE");

retry_fw:
    /*
     * Sleep for some time: checking right after sending the load command
     * is useless, the firmware at least won't be booted for sure.
     */
    usleep(CIRRUS_FIRMWARE_LOAD_SLEEP_US);

    ret = cirrus_get_mixer_array_by_name(ctl_name, &cspl_ena, 4);
    if (ret < 0) {
        if (retry < CIRRUS_FIRMWARE_MAX_RETRY) {
            retry++;
            ALOGI("%s: Retrying...\n", __func__);
            goto retry_fw;
        } else {
            ALOGE("%s: Cannot get %s stats", __func__, ctl_name);
            goto exit;
        }
    }

    if ((cspl_ena[0] + cspl_ena[1] + cspl_ena[2]) == 0 && cspl_ena[3] == 1) {
        ALOGI("%s: Cirrus protection Firmware Download SUCCESS.", __func__);
        /* Wait for the hardware to stabilize */
        usleep(100000);
        ret = 0;
    } else {
        /*
         * Since we are using a poor hack to load the firmware, we cannot know
         * if the firmware was found nor if it finished loading remotely.
         * We also don't know how much time does the chip require to actually
         * boot it, so we will sleep and retry for X times, until it loads and
         * boots, or we assume that something went wrong: in that case the
         * only thing left to do is to return an error, hoping that developers
         * will catch it before going crazy...
         *
         * Perhaps, one day we will rewrite this messy part.
         */
        if (retry < CIRRUS_FIRMWARE_MAX_RETRY) {
            retry++;
            ALOGI("%s: Retrying...\n", __func__);
            goto retry_fw;
        }

        ALOGE("%s: Firmware download failure. CSPL Status: %u %u %u %u",
              __func__, cspl_ena[0], cspl_ena[1], cspl_ena[2], cspl_ena[3]);
        ret = -EINVAL;
    }

exit:
    return ret;
}

static int cirrus_do_fw_download(int do_reset) {
    bool cal_valid = false, status_ok = false, checksum_ok = false;
    int i, max_retries = 32, ret = 0;

    for (i = 0; i < max_retries; i++) {
        ret = cirrus_exec_prot_fw_download(0, do_reset);
        if (ret == 0)
            break;
        usleep(500000);
    }
    if (ret != 0) {
        ALOGE("%s: Cannot send Protection firmware: bailing out.",
              __func__);
        return -EINVAL;
    }

    /* If the calibration is not valid, keep the fw loaded but get out. */
    if (!handle.spk.cal_ok)
        return -EINVAL;

    ret = cirrus_set_force_wake(true);
    if (ret < 0)
        goto exit;

    ret = cirrus_set_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CAL_R,
                                         &handle.spk.cal_r, 4);
    if (ret < 0) {
        ALOGE("%s: Cannot set Z calibration", __func__);
        goto exit;
    }

    ret = cirrus_set_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CAL_STATUS, handle.spk.status, 4);
    if (ret < 0) {
        ALOGE("%s: Cannot set calibration status", __func__);
        goto exit;
    }

    ret = cirrus_set_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CAL_CHECKSUM, handle.spk.checksum, 4);
    if (ret < 0) {
        ALOGE("%s: Cannot set calibration checksum", __func__);
        goto exit;
    }

    /* Time to get some rest: work is done! */
    ret = cirrus_set_force_wake(false);
    if (ret < 0)
        goto exit;

exit:
    return ret;
}

static void *cirrus_do_calibration() {
    struct audio_device *adev = handle.adev_handle;
    int ret = 0;

    pthread_mutex_lock(&adev->lock);
    handle.state = CALIBRATING;
    pthread_mutex_unlock(&adev->lock);

    ret = cirrus_do_fw_download(0);
    if (ret < 0)
        ALOGE("%s: Cannot send speaker protection FW", __func__);

end:
    pthread_mutex_lock(&adev->lock);
    if (ret < 0)
        handle.state = CALIBRATION_ERROR;
    else
        handle.state = IDLE;
    pthread_mutex_unlock(&adev->lock);

    pthread_exit(0);
    return NULL;
}

/*
 * Function: spkr_prot_init
 *
 * Main speaker protection initialization.
 * Allocates calibration data, initializes calibration,
 * sets the global device pointer, and creates a thread to load additional parameters.
 */
void spkr_prot_init(void *adev, spkr_prot_init_config_t spkr_prot_init_config_val)
{
    if (!adev) {
        ALOGE("%s: Invalid params", __func__);
        return;
    }

    memset(&handle, 0, sizeof(handle));
    if (handle) {
        handle.adev_handle = adev;
        handle.state = INIT;
        
        int tmp = 0;
        char prop_val[CONFIG_FILE_SIZE] = {0};
        ret = audio_extn_read_from_file(PERSIST_CIRRUS_CAL_SPK_CAL_R, &handle.spk.cal_r);
        if (ret < 0) {
            ret = audio_extn_read_from_file(PERSIST_CIRRUS_CAL_SPK_CAL_AMBIENT, &handle.spk.cal_r);
            if (ret == 0) {
                break;
            }
            property_get("persist.vendor.audio.default.spkrdc", prop_val, CIRRUS_DEFAULT_CSPL_REDC);
            tmp = atoi(prop_val);
            memcpy(handle.spk.cal_r, &tmp sizeof(tmp));
            ALOGE("%s: Speaker Protection(CSPL) not calibrated on speaker, using default ReDC (%ld)", __func__, cal_val);
        }

        // init function pointers
        fp_platform_get_snd_device_name = spkr_prot_init_config_val.fp_platform_get_snd_device_name;
        fp_platform_get_pcm_device_id = spkr_prot_init_config_val.fp_platform_get_pcm_device_id;
        fp_get_usecase_from_list =  spkr_prot_init_config_val.fp_get_usecase_from_list;
        fp_disable_snd_device = spkr_prot_init_config_val.fp_disable_snd_device;
        fp_enable_snd_device = spkr_prot_init_config_val.fp_enable_snd_device;
        fp_disable_audio_route = spkr_prot_init_config_val.fp_disable_audio_route;
        fp_enable_audio_route = spkr_prot_init_config_val.fp_enable_audio_route;
        fp_audio_extn_get_snd_card_split = spkr_prot_init_config_val.fp_audio_extn_get_snd_card_split;

        pthread_mutex_init(&handle.fb_prot_mutex, NULL);

        spkr_prot_calib_init();

        /* We assume calibration part is okay as there are no mixers for calibration */
        handle.spk.cal_ok = true;

        (void)pthread_create(&handle.calibration_thread,
            (const pthread_attr_t *) NULL,
            cirrus_do_calibration, &handle);
        
        return;
    }
    ALOGE("%s: memory allocation failure: handle", __func__);
}

int spkr_prot_deinit()
{
    ALOGV("%s: Entry", __func__);
    if (!handle) {
        return 0;
    }

    pthread_join(handle.calibration_thread, NULL);
    pthread_mutex_destroy(&handle.fb_prot_mutex);
    free(&handle);

    ALOGV("%s: Exit", __func__);
    return 0;
}

static int cirrus_get_mixer_array_by_name(char* ctl_name, void* array, size_t count) {
    struct mixer *card_mixer = NULL;
    struct mixer_ctl *ctl_config = NULL;
    int sndcard_id = 0, ret = -EINVAL;

    card_mixer = mixer_open(sndcard_id);
    if (!card_mixer) {
        ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
        return -1;
    }

    ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
    if (!ctl_config) {
        ALOGE("%s: Cannot get mixer control %s", __func__, ctl_name);
        ret = -1;
        goto exit;
    }

    memset(array, 0, count);

    ret = mixer_ctl_get_array(ctl_config, array, count);
    if (ret < 0)
        ALOGE("%s: Cannot get mixer %s value: error %d",
              __func__, ctl_name, ret);
exit:
    mixer_close(card_mixer);
    return ret;
}

static inline int cirrus_set_force_wake(bool enable) {
    int ret = 0;

    ret = cirrus_set_mixer_value_by_name(CIRRUS_CTL_SPK_FORCE_WAKE, (int)enable);

    if (ret < 0)
        ALOGE("%s: Cannot %s force wakeup", __func__,
              enable ? "enable" : "disable");
    else
        ALOGD("%s: Set %s %s", __func__, CIRRUS_CTL_SPK_FORCE_WAKE,
              enable ? "enable" : "disable");
    return ret;
}

static int cirrus_check_error_state(void) {
    uint8_t cspl_error[4] = { 0 };
    int ret = 0;

    ret = cirrus_set_force_wake(true);
    if (ret < 0)
        return ret;

    ret = cirrus_get_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CSPL_ERRORNO,
                                         &cspl_error, 4);
    if (ret < 0) {
        ALOGE("%s: Cannot get CSPL status", __func__);
        goto exit;
    }

    if (cspl_error[3] != 0) {
        ALOGE("%s: Error state detected!", __func__);
        ret = -EREMOTEIO;
    }

exit:
    ret = cirrus_set_force_wake(false);
    return ret;
}

static int cirrus_check_error_fatal(void) {
    int ret = 0;

    pthread_mutex_lock(&handle.fb_prot_mutex);

    ret = cirrus_check_error_state();
    if (ret == 0)
        goto success;

    /* Ouch! Error! Let's wait just a lil and retry to be extra sure... */
    usleep(CIRRUS_ERROR_DETECT_SLEEP_US);
    ret = cirrus_check_error_state();
    if (ret == 0)
        goto success;

    /* Error recovery: this should actually never happen, anyway... */
    ALOGE("%s: Cirrus DSP is in error state: resetting device", __func__);
    cirrus_do_reset();

    ALOGE("%s: Reset done, crashing HAL to reload firmware...", __func__);
    ALOGE("%s: Goodbye, infamous world...", __func__);
    abort();

success:
    pthread_mutex_unlock(&handle.fb_prot_mutex);
    return ret;
}

static void *cirrus_failure_detect_thread() {
    ALOGD("%s: Entry", __func__);

    (void)cirrus_check_error_fatal();

    ALOGD("%s: Exit ", __func__);

    pthread_exit(0);
    return NULL;
}

int spkr_prot_start_processing(__unused snd_device_t snd_device) {
    struct audio_device *adev = handle.adev_handle;
    int ret = 0;

    ALOGV("%s: Entry", __func__);

    if (!adev) {
        ALOGE("%s: Invalid params", __func__);
        return -EINVAL;
    }

    if (pthread_self() == handle.calibration_thread) {
        // Succeed without doing anything; the calibration already
        // selects the right paths, and we do not want the failure
        // detect thread to run just yet.
        ALOGV("%s: We are the calibration thread", __func__);
        goto end;
    }

    pthread_mutex_lock(&handle.fb_prot_mutex);

    ALOGV("%s: current state %d", __func__, handle.state);

    /*
     * If we are still in calibration phase, we cannot play audio...
     * and it's the same if we got an error during the process.
     *
     * Reason is that if we try playing audio during calibration, then
     * the result will be bad and we will end up with a poorly calibrated
     * speaker. Also, the DSP may get left in a bad state and not accept
     * the protection firmware when we're ready for it.
     */
    if (handle.state == CALIBRATING || handle.state == CALIBRATION_ERROR) {
        ALOGI("%s: Forbidden. Calibration %s", __func__,
              handle.state == CALIBRATING ? "is in progress..." : "failed.");
        ret = -1;
        goto end;
    }

    ret = cirrus_set_force_wake(true);

    audio_route_apply_and_update_path(adev->audio_route,
                                      fp_platform_get_snd_device_name(snd_device));

    if (handle.state == IDLE)
        (void)pthread_create(&handle.failure_detect_thread,
                    (const pthread_attr_t *) NULL,
                    cirrus_failure_detect_thread,
                    &handle);

    handle.state = PLAYBACK;
end:
    pthread_mutex_unlock(&handle.fb_prot_mutex);

    ALOGV("%s: Exit", __func__);
    return ret;
}

void spkr_prot_stop_processing(__unused snd_device_t snd_device) {
    struct audio_usecase *uc_info_tx;
    struct audio_device *adev = handle.adev_handle;

    ALOGV("%s: Entry", __func__);

    pthread_mutex_lock(&handle.fb_prot_mutex);

    if (pthread_self() == handle.calibration_thread) {
        // This happens when stopping the device from calibration. We bailed
        // and never set PLAYBACK, so we should also never update the audio
        // route nor unconditionally set the state back to IDLE
        ALOGV("%s: We are the calibration thread", __func__);
        goto end;
    }

    if (handle.state != PLAYBACK) {
        ALOGE("%s: Cannot stop processing, state is not PLAYBACK (but %d)",
              __func__, handle.state);
        goto end;
    }

    handle.state = IDLE;

    audio_route_reset_and_update_path(adev->audio_route,
                                      fp_platform_get_snd_device_name(snd_device));

end:
    pthread_mutex_unlock(&handle.fb_prot_mutex);

    (void)cirrus_set_force_wake(false);

    ALOGV("%s: Exit", __func__);
}

bool spkr_prot_is_enabled() {
    return true;
}

int get_spkr_prot_snd_device(snd_device_t snd_device) {
    switch(snd_device) {
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

void spkr_prot_calib_cancel(__unused void *adev) {
    return;
}