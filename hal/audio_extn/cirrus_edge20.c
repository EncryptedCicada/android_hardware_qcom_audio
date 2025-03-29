/*
 * Refined Speaker Protection (CSPL) Implementation
 *
 * This file is derived from disassembled stock HAL code.
 *
 */

 #define LOG_TAG "audio_mot_sp"
 #define DEBUG 1
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
 
 #ifdef DEBUG
 #define CSPL_DBGPRT(fmt, ...) ALOGD("%s: " fmt, __func__, ##__VA_ARGS__)
 #else
 #define CSPL_DBGPRT(fmt, ...)
 #endif
 
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
     pthread_t failure_detect_thread;
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
  * Function: persist_read_from_file
  *
  * Reads a numerical value from the file at 'path' and stores it in *arr.
  * Returns 0 for success, but <0 values like -EINVAL or -ENOMEM etc, whichever is suitable, otherwise.
  */
 int persist_read_from_file(char *path, uint8_t *arr, size_t arr_size)
 {
     int ret = 0;
     
     #ifdef DEBUG
     CSPL_DBGPRT("Reading from file %s with array size %zu", path, arr_size);
     #endif
     
     // Check if array is at least 4 bytes
     if (arr_size != 4) {
         ALOGE("%s: Provided array size is not suitable (size: %zu, required: 4)", __func__, arr_size);
         return -EINVAL;
     }
 
     FILE *stream = fopen(path, "rb");
     if (!stream) {
         ALOGE("%s: failed to open: %s", __func__, path);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to open file: %s (errno: %d: %s)", path, errno, strerror(errno));
         #endif
         return -ENOENT; // No such file or directory
     }
     
     #ifdef DEBUG
     CSPL_DBGPRT("File opened successfully: %s", path);
     #endif
     
     fseek(stream, 0, SEEK_END);
     long len = ftell(stream);
     
     #ifdef DEBUG
     CSPL_DBGPRT("File length: %ld bytes", len);
     #endif
     
     if (ferror(stream) != 0) {
         ALOGE("%s: '%s' file seek error", __func__, path);
         #ifdef DEBUG
         CSPL_DBGPRT("File seek error on %s", path);
         #endif
         ret = -EIO; // Input/output error
         goto check_error;
     }
     
     if ((len - 1U) >= 0x20) {
         ALOGE("%s: '%s' file too large", __func__, path);
         #ifdef DEBUG
         CSPL_DBGPRT("File too large: %ld bytes (max 31 bytes)", len);
         #endif
         ret = -EFBIG; // File too big
         goto check_error;
     }
     
     rewind(stream);
     char *buffer = calloc(len + 1, 1);
     if (!buffer) {
         ALOGE("%s: memory allocation failure", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Memory allocation failure for buffer of size %ld", len + 1);
         #endif
         ret = -ENOMEM; // Not enough memory
         goto check_error;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Memory allocated for buffer: %ld bytes", len + 1);
     #endif
 
     if (fgets(buffer, len + 1, stream) == NULL) {
         if (ferror(stream)) {
             // A read error occurred
             ALOGE("%s: Error reading from file: %s", __func__, strerror(errno));
             #ifdef DEBUG
             CSPL_DBGPRT("Error reading from file: %s", strerror(errno));
             #endif
             ret = -EIO;
         } else {
             // EOF reached without reading anything
             ALOGE("%s: No data read from file", __func__);
             #ifdef DEBUG
             CSPL_DBGPRT("No data read from file (EOF)");
             #endif
             ret = -EINVAL;
         }
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Successfully read data from file: '%s'", buffer);
     #endif
 
     errno = 0;
     char *endptr = NULL;
     long val = strtol(buffer, &endptr, 0);
     
     #ifdef DEBUG
     CSPL_DBGPRT("Parsed value: %ld (0x%lx)", val, val);
     #endif
     
     if (errno != 0 || *endptr != '\0') {
         ALOGE("%s: strtol() parse failure", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to parse value: errno=%d, remaining string='%s'", errno, endptr);
         #endif
         ret = -EINVAL; // Invalid argument (parsing error)
         goto exit;
     }
     
     arr[0] = val & 0xFF;
     arr[1] = (val >> 8) & 0xFF;
     arr[2] = (val >> 16) & 0xFF;
     arr[3] = (val >> 24) & 0xFF;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Stored in array: [0x%02x, 0x%02x, 0x%02x, 0x%02x]", 
                 arr[0], arr[1], arr[2], arr[3]);
     #endif
 
 exit:
     free(buffer);
 check_error:
     fclose(stream);
     #ifdef DEBUG
     CSPL_DBGPRT("Returning %d", ret);
     #endif
     return ret;
 }
 
 /* TODO: This function assumes that we are always using CARD 0 */
 static int cirrus_set_mixer_value_by_name(char* ctl_name, int value) {
     struct mixer *card_mixer = NULL;
     struct mixer_ctl *ctl_config = NULL;
     int sndcard_id = 0, ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Setting mixer '%s' to value %d", ctl_name, value);
     #endif
 
     card_mixer = mixer_open(sndcard_id);
     if (!card_mixer) {
         ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to open mixer for card %d", sndcard_id);
         #endif
         return -1;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer opened successfully for card %d", sndcard_id);
     #endif
 
     ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
     if (!ctl_config) {
         ALOGD("%s: Cannot get mixer control %s", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot get mixer control '%s'", ctl_name);
         #endif
         ret = -1;
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Found mixer control '%s'", ctl_name);
     #endif
 
     ret = mixer_ctl_set_value(ctl_config, 0, value);
     if (ret < 0) {
         ALOGE("%s: Cannot set mixer '%s' to '%d'",
               __func__, ctl_name, value);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set mixer value: error %d", ret);
         #endif
     } else {
         #ifdef DEBUG
         CSPL_DBGPRT("Successfully set mixer '%s' to value %d", ctl_name, value);
         #endif
     }
     
 exit:
     mixer_close(card_mixer);
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer closed, returning %d", ret);
     #endif
     return ret;
 }
 
 static int cirrus_get_mixer_value_by_name(char* ctl_name) {
     struct mixer *card_mixer = NULL;
     struct mixer_ctl *ctl_config = NULL;
     int sndcard_id = 0, ret = -EINVAL;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Getting mixer value for '%s'", ctl_name);
     #endif
 
     card_mixer = mixer_open(sndcard_id);
     if (!card_mixer) {
         ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to open mixer for card %d", sndcard_id);
         #endif
         return -1;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer opened successfully for card %d", sndcard_id);
     #endif
 
     ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
     if (!ctl_config) {
         ALOGE("%s: Cannot get mixer control %s", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot get mixer control '%s'", ctl_name);
         #endif
         ret = -1;
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Found mixer control '%s'", ctl_name);
     #endif
 
     ret = mixer_ctl_get_value(ctl_config, 0);
     if (ret < 0) {
         ALOGE("%s: Cannot get mixer %s value: error %d",
               __func__, ctl_name, ret);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to get mixer value: error %d", ret);
         #endif
     } else {
         #ifdef DEBUG
         CSPL_DBGPRT("Mixer '%s' value = %d", ctl_name, ret);
         #endif
     }
     
 exit:
     mixer_close(card_mixer);
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer closed, returning %d", ret);
     #endif
     return ret;
 }
 
 static int cirrus_set_mixer_array_by_name(char* ctl_name, void* array, size_t count) {
     struct mixer *card_mixer = NULL;
     struct mixer_ctl *ctl_config = NULL;
     int sndcard_id = 0, ret = 0;
 
     #ifdef DEBUG
     uint8_t *arr = (uint8_t*)array;
     CSPL_DBGPRT("Setting mixer array '%s' with %zu bytes: [%02x,%02x,%02x,%02x]", 
                 ctl_name, count, arr[0], arr[1], arr[2], arr[3]);
     #endif
 
     card_mixer = mixer_open(sndcard_id);
     if (!card_mixer) {
         ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to open mixer for card %d", sndcard_id);
         #endif
         return -1;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer opened successfully for card %d", sndcard_id);
     #endif
 
     ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
     if (!ctl_config) {
         ALOGD("%s: Cannot get mixer control %s", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot get mixer control '%s'", ctl_name);
         #endif
         ret = -1;
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Found mixer control '%s'", ctl_name);
     #endif
 
     ret = mixer_ctl_set_array(ctl_config, array, count);
     if (ret < 0) {
         ALOGE("%s: Cannot set mixer %s",
               __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set mixer array: error %d", ret);
         #endif
     } else {
         #ifdef DEBUG
         CSPL_DBGPRT("Successfully set mixer array '%s'", ctl_name);
         #endif
     }
     
 exit:
     mixer_close(card_mixer);
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer closed, returning %d", ret);
     #endif
     return ret;
 }
 
 static int cirrus_get_mixer_array_by_name(char* ctl_name, void* array, size_t count) {
     struct mixer *card_mixer = NULL;
     struct mixer_ctl *ctl_config = NULL;
     int sndcard_id = 0, ret = -EINVAL;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Getting mixer array '%s' with size %zu", ctl_name, count);
     #endif
 
     card_mixer = mixer_open(sndcard_id);
     if (!card_mixer) {
         ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to open mixer for card %d", sndcard_id);
         #endif
         return -1;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer opened successfully for card %d", sndcard_id);
     #endif
 
     ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
     if (!ctl_config) {
         ALOGE("%s: Cannot get mixer control %s", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot get mixer control '%s'", ctl_name);
         #endif
         ret = -1;
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Found mixer control '%s'", ctl_name);
     #endif
 
     memset(array, 0, count);
 
     ret = mixer_ctl_get_array(ctl_config, array, count);
     if (ret < 0) {
         ALOGE("%s: Cannot get mixer %s value: error %d",
               __func__, ctl_name, ret);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to get mixer array: error %d", ret);
         #endif
     } else {
         #ifdef DEBUG
         uint8_t *arr = (uint8_t*)array;
         CSPL_DBGPRT("Mixer array '%s' values: [%02x,%02x,%02x,%02x]", 
                     ctl_name, arr[0], arr[1], arr[2], arr[3]);
         #endif
     }
     
 exit:
     mixer_close(card_mixer);
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer closed, returning %d", ret);
     #endif
     return ret;
 }
 
 static int cirrus_set_mixer_enum_by_name(char* ctl_name, const char* value) {
     struct mixer *card_mixer = NULL;
     struct mixer_ctl *ctl_config = NULL;
     int sndcard_id = 0, ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Setting mixer enum '%s' to '%s'", ctl_name, value);
     #endif
 
     card_mixer = mixer_open(sndcard_id);
     if (!card_mixer) {
         ALOGE("%s: Cannot open mixer for card %d.", __func__, sndcard_id);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to open mixer for card %d", sndcard_id);
         #endif
         return -1;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer opened successfully for card %d", sndcard_id);
     #endif
 
     ctl_config = mixer_get_ctl_by_name(card_mixer, ctl_name);
     if (!ctl_config) {
         ALOGE("%s: Cannot get mixer control '%s'", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot get mixer control '%s'", ctl_name);
         #endif
         ret = -1;
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Found mixer control '%s'", ctl_name);
     #endif
 
     ret = mixer_ctl_set_enum_by_string(ctl_config, value);
     if (ret < 0) {
         ALOGE("%s: Cannot set mixer '%s' to '%s'",
               __func__, ctl_name, value);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set mixer enum: error %d", ret);
         #endif
     } else {
         #ifdef DEBUG
         CSPL_DBGPRT("Successfully set mixer enum '%s' to '%s'", ctl_name, value);
         #endif
     }
     
 exit:
     mixer_close(card_mixer);
     #ifdef DEBUG
     CSPL_DBGPRT("Mixer closed, returning %d", ret);
     #endif
     return ret;
 }
 
 static int cirrus_do_reset() {
     int ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Starting CCM reset");
     #endif
 
     ret = cirrus_get_mixer_value_by_name(CIRRUS_CTL_SPK_CCM_RESET);
     if (ret < 0) {
         ALOGE("%s: CCM Reset is missing!!!", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("CCM Reset control is missing");
         #endif
     } else {
         ret = cirrus_set_mixer_value_by_name(CIRRUS_CTL_SPK_CCM_RESET, 1);
         ALOGI("%s: CCM Reset done.", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("CCM Reset completed");
         #endif
     }
 
     return ret;
 }
 
 static int cirrus_mixer_wait_for_setting(char *ctl, int val, int retry)
 {
     int i, ret;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Waiting for mixer '%s' to reach value %d (max %d retries)", ctl, val, retry);
     #endif
 
     for (i = 0; i < retry; i++) {
         /* Start firmware download sequence: shut down DSP and reset states */
         ret = cirrus_get_mixer_value_by_name(ctl);
         #ifdef DEBUG
         CSPL_DBGPRT("Retry %d: current value = %d", i+1, ret);
         #endif
         
         if (ret < 0 || ret == val)
             break;
 
         usleep(10000);
     }
     
     if (ret < 0 && i == retry) {
         #ifdef DEBUG
         CSPL_DBGPRT("Timed out waiting for setting");
         #endif
         return -ETIMEDOUT;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Wait completed, value reached or error occurred: %d", ret);
     #endif
     return ret;
 }
 
 void prepare_control_name(char *buffer, size_t buffer_size, const char *name)
 {
     if (buffer == NULL || name == NULL) {
         #ifdef DEBUG
         if (buffer == NULL) CSPL_DBGPRT("Buffer pointer is NULL");
         if (name == NULL) CSPL_DBGPRT("Name pointer is NULL");
         #endif
         return;
     }
     
     #ifdef DEBUG
     CSPL_DBGPRT("Preparing control name: '%s'", name);
     #endif
     
     // Clear the buffer
     memset(buffer, 0, buffer_size);
     
     // Copy the desired name into the buffer
     snprintf(buffer, buffer_size, "%s", name);
     
     #ifdef DEBUG
     CSPL_DBGPRT("Control name prepared: '%s'", buffer);
     #endif
 }
 
 static int cirrus_exec_prot_fw_download(int do_reset) {
     char ctl_name[CIRRUS_CTL_NAME_BUF];
     uint8_t cspl_ena[4] = { 0 };
     int retry = 0, ret;
 
     ALOGD("%s: Asking for speaker firmware %s", __func__, (do_reset ? "with reset" : "without reset"));
     #ifdef DEBUG
     CSPL_DBGPRT("Starting firmware download with%s reset", do_reset ? "" : "out");
     #endif
 
     if (do_reset) {
         ret = cirrus_do_reset();
         #ifdef DEBUG
         CSPL_DBGPRT("Reset completed with result: %d", ret);
         #endif
     }
 
     /* If this one is missing, we're not using our Cirrus codec... */
     prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP Booted");
     ret = cirrus_get_mixer_value_by_name(ctl_name);
     if (ret < 0) {
         ALOGE("%s: %s control is missing. Bailing out.", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("DSP Booted control is missing, bailing out");
         #endif
         ret = -ENODEV;
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP Booted control found with value: %d", ret);
     #endif
 
     /* Start firmware download sequence: shut down DSP and reset states */
     ret = cirrus_set_mixer_value_by_name(ctl_name, 0);
     if (ret < 0) {
         ALOGE("%s: Cannot reset %s status", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to reset DSP Booted status");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP Booted status reset");
     #endif
 
     ret = cirrus_mixer_wait_for_setting(ctl_name, 0, 10);
     if (ret < 0) {
         ALOGE("%s: %s wait setting error %d", __func__, ctl_name, ret);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to wait for DSP Booted setting");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP Booted wait completed");
     #endif
 
     usleep(10000);
 
     prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1 Preload Switch");
     ret = cirrus_set_mixer_value_by_name(ctl_name, 0);
     if (ret < 0) {
         ALOGE("%s: Cannot reset %s", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to reset DSP1 Preload Switch");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP1 Preload Switch reset");
     #endif
 
     ret = cirrus_mixer_wait_for_setting(ctl_name, 0, 10);
     if (ret < 0) {
         ALOGE("%s: %s wait setting error %d", __func__, ctl_name, ret);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to wait for DSP1 Preload Switch setting");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP1 Preload Switch wait completed");
     #endif
 
     usleep(10000);
 
     /* Determine what firmware to load and configure DSP */
     prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1 Firmware");
     ret = cirrus_set_mixer_enum_by_name(ctl_name, "protection");
     if (ret < 0) {
         ALOGE("%s: Cannot set %s to protection", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set DSP1 Firmware to protection");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP1 Firmware set to protection");
     #endif
 
     prepare_control_name(ctl_name, sizeof(ctl_name), "SPK PCM Source");
     ret = cirrus_set_mixer_enum_by_name(ctl_name, "DSP");
     if (ret < 0) {
         ALOGE("%s: Cannot set %s to DSP", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set PCM Source to DSP");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("PCM Source set to DSP");
     #endif
 
     /* Send the firmware! */
     prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1 Preload Switch");
     ret = cirrus_set_mixer_value_by_name(ctl_name, 1);
     if (ret < 0) {
         ALOGE("%s: Cannot set %s to protection", __func__, ctl_name);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set DSP1 Preload Switch to 1");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("DSP1 Preload Switch set to 1 - firmware loading started");
     #endif
 
     prepare_control_name(ctl_name, sizeof(ctl_name), "SPK DSP1X protection cd CSPL_ENABLE");
 
 retry_fw:
     /*
      * Sleep for some time: checking right after sending the load command
      * is useless, the firmware at least won't be booted for sure.
      */
     #ifdef DEBUG
     CSPL_DBGPRT("Sleeping for %d us before checking firmware status", CIRRUS_FIRMWARE_LOAD_SLEEP_US);
     #endif
     
     usleep(CIRRUS_FIRMWARE_LOAD_SLEEP_US);
 
     ret = cirrus_get_mixer_array_by_name(ctl_name, &cspl_ena, 4);
     if (ret < 0) {
         if (retry < CIRRUS_FIRMWARE_MAX_RETRY) {
             retry++;
             ALOGI("%s: Retrying...\n", __func__);
             #ifdef DEBUG
             CSPL_DBGPRT("Retry %d/%d after failed get_mixer_array", 
                        retry, CIRRUS_FIRMWARE_MAX_RETRY);
             #endif
             goto retry_fw;
         } else {
             ALOGE("%s: Cannot get %s stats", __func__, ctl_name);
             #ifdef DEBUG
             CSPL_DBGPRT("Failed to get CSPL_ENABLE stats after %d retries", 
                        CIRRUS_FIRMWARE_MAX_RETRY);
             #endif
             goto exit;
         }
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("CSPL_ENABLE status: [%u,%u,%u,%u]", 
                cspl_ena[0], cspl_ena[1], cspl_ena[2], cspl_ena[3]);
     #endif
 
     if ((cspl_ena[0] + cspl_ena[1] + cspl_ena[2]) == 0 && cspl_ena[3] == 1) {
         ALOGI("%s: Cirrus protection Firmware Download SUCCESS.", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Firmware download successful!");
         #endif
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
             #ifdef DEBUG
             CSPL_DBGPRT("Retry %d/%d after CSPL_ENABLE check failed", 
                        retry, CIRRUS_FIRMWARE_MAX_RETRY);
             #endif
             goto retry_fw;
         }
 
         ALOGE("%s: Firmware download failure. CSPL Status: %u %u %u %u",
               __func__, cspl_ena[0], cspl_ena[1], cspl_ena[2], cspl_ena[3]);
         #ifdef DEBUG
         CSPL_DBGPRT("Firmware download failed after %d retries", 
                    CIRRUS_FIRMWARE_MAX_RETRY);
         #endif
         ret = -EINVAL;
     }
 
 exit:
     #ifdef DEBUG
     CSPL_DBGPRT("Firmware download process completed with result: %d", ret);
     #endif
     return ret;
 }
 
 static inline int cirrus_set_force_wake(bool enable) {
     int ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("%s force wake", enable ? "Enabling" : "Disabling");
     #endif
 
     ret = cirrus_set_mixer_value_by_name(CIRRUS_CTL_SPK_FORCE_WAKE, (int)enable);
 
     if (ret < 0)
         ALOGE("%s: Cannot %s force wakeup", __func__,
               enable ? "enable" : "disable");
     else
         ALOGD("%s: Set %s %s", __func__, CIRRUS_CTL_SPK_FORCE_WAKE,
               enable ? "enable" : "disable");
               
     #ifdef DEBUG
     CSPL_DBGPRT("Force wake %s with result: %d", 
                enable ? "enabled" : "disabled", ret);
     #endif
     
     return ret;
 }
 
 static int cirrus_do_fw_download(int do_reset) {
     bool cal_valid = false, status_ok = false, checksum_ok = false;
     int i, max_retries = 32, ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Starting firmware download with%s reset (max retries: %d)", 
                do_reset ? "" : "out", max_retries);
     #endif
 
     for (i = 0; i < max_retries; i++) {
         #ifdef DEBUG
         CSPL_DBGPRT("Attempt %d/%d to send protection firmware", i+1, max_retries);
         #endif
         
         ret = cirrus_exec_prot_fw_download(do_reset);
         if (ret == 0)
             break;
             
         #ifdef DEBUG
         CSPL_DBGPRT("Attempt %d failed, sleeping before retry", i+1);
         #endif
         
         usleep(500000);
     }
     
     if (ret != 0) {
         ALOGE("%s: Cannot send Protection firmware: bailing out.",
               __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to send protection firmware after %d attempts", i);
         #endif
         return -EINVAL;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Protection firmware sent successfully on attempt %d", i+1);
     CSPL_DBGPRT("Checking if calibration is valid: %s", 
                handle.spk.cal_ok ? "VALID" : "INVALID");
     #endif
 
     /* If the calibration is not valid, keep the fw loaded but get out. */
     if (!handle.spk.cal_ok) {
         #ifdef DEBUG
         CSPL_DBGPRT("Calibration not valid, exiting without setting cal values");
         #endif
         return -EINVAL;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Calibration valid, proceeding with setting cal values");
     #endif
 
     ret = cirrus_set_force_wake(true);
     if (ret < 0) {
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set force wake");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Force wake set, setting calibration data");
     CSPL_DBGPRT("Setting CAL_R: [%02x,%02x,%02x,%02x]", 
                handle.spk.cal_r[0], handle.spk.cal_r[1], 
                handle.spk.cal_r[2], handle.spk.cal_r[3]);
     #endif
 
     ret = cirrus_set_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CAL_R, &handle.spk.cal_r, 4);
     if (ret < 0) {
         ALOGE("%s: Cannot set Z calibration", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set Z calibration");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Setting CAL_STATUS: [%02x,%02x,%02x,%02x]", 
                handle.spk.status[0], handle.spk.status[1], 
                handle.spk.status[2], handle.spk.status[3]);
     #endif
 
     ret = cirrus_set_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CAL_STATUS, &handle.spk.status, 4);
     if (ret < 0) {
         ALOGE("%s: Cannot set calibration status", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set calibration status");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Setting CAL_CHECKSUM: [%02x,%02x,%02x,%02x]", 
                handle.spk.checksum[0], handle.spk.checksum[1], 
                handle.spk.checksum[2], handle.spk.checksum[3]);
     #endif
 
     ret = cirrus_set_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CAL_CHECKSUM, &handle.spk.checksum, 4);
     if (ret < 0) {
         ALOGE("%s: Cannot set calibration checksum", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set calibration checksum");
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("All calibration data set successfully, disabling force wake");
     #endif
 
     /* Time to get some rest: work is done! */
     ret = cirrus_set_force_wake(false);
     if (ret < 0) {
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to disable force wake");
         #endif
         goto exit;
     }
 
 exit:
     #ifdef DEBUG
     CSPL_DBGPRT("Firmware download and calibration completed with result: %d", ret);
     #endif
     return ret;
 }
 
 static void *cirrus_do_calibration() {
     struct audio_device *adev = handle.adev_handle;
     int ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Starting calibration thread");
     #endif
 
     pthread_mutex_lock(&adev->lock);
     handle.state = CALIBRATING;
     pthread_mutex_unlock(&adev->lock);
 
     #ifdef DEBUG
     CSPL_DBGPRT("State set to CALIBRATING, downloading firmware");
     #endif
 
     ret = cirrus_do_fw_download(0);
     if (ret < 0) {
         ALOGE("%s: Cannot send speaker protection FW", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to send speaker protection firmware: %d", ret);
         #endif
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Firmware download %s", ret < 0 ? "failed" : "successful");
     CSPL_DBGPRT("Updating state based on result");
     #endif
 
     pthread_mutex_lock(&adev->lock);
     if (ret < 0) {
         handle.state = CALIBRATION_ERROR;
         #ifdef DEBUG
         CSPL_DBGPRT("State set to CALIBRATION_ERROR");
         #endif
     } else {
         handle.state = IDLE;
         #ifdef DEBUG
         CSPL_DBGPRT("State set to IDLE");
         #endif
     }
     pthread_mutex_unlock(&adev->lock);
 
     #ifdef DEBUG
     CSPL_DBGPRT("Calibration thread exiting");
     #endif
 
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
         #ifdef DEBUG
         CSPL_DBGPRT("Invalid adev parameter");
         #endif
         return;
     }
     
     int ret = 0;
     int32_t tmp = 0;
     char prop_val[CONFIG_FILE_SIZE] = {0};
 
     #ifdef DEBUG
     CSPL_DBGPRT("Initializing speaker protection");
     #endif
 
     memset(&handle, 0, sizeof(handle));
     #ifdef DEBUG
     CSPL_DBGPRT("Handle structure initialized to zeros");
     #endif
 
     handle.adev_handle = adev;
     handle.state = INIT;
     
     #ifdef DEBUG
     CSPL_DBGPRT("State set to INIT, reading calibration values");
     #endif
     
     ret = persist_read_from_file(PERSIST_CIRRUS_CAL_SPK_CAL_R, handle.spk.cal_r, sizeof(handle.spk.cal_r));
     if (ret != 0) {
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to read from %s, trying ambient file", PERSIST_CIRRUS_CAL_SPK_CAL_R);
         #endif
         
         ret = persist_read_from_file(PERSIST_CIRRUS_CAL_SPK_CAL_AMBIENT, handle.spk.cal_r, sizeof(handle.spk.cal_r));
         if (ret != 0) {
             #ifdef DEBUG
             CSPL_DBGPRT("Failed to read from ambient file, using default value");
             #endif
             
             property_get("persist.vendor.audio.default.spkrdc", prop_val, CIRRUS_DEFAULT_CSPL_REDC);
             tmp = atoi(prop_val);
             
             #ifdef DEBUG
             CSPL_DBGPRT("Default ReDC value: %d (0x%x)", tmp, tmp);
             #endif
             
             memcpy(handle.spk.cal_r, &tmp, sizeof(tmp));
             ALOGE("%s: Speaker Protection(CSPL) not calibrated on speaker, using default ReDC (%d)", __func__, tmp);
         } else {
             #ifdef DEBUG
             CSPL_DBGPRT("Successfully read from ambient file");
             CSPL_DBGPRT("CAL_R values: [%02x,%02x,%02x,%02x]", 
                        handle.spk.cal_r[0], handle.spk.cal_r[1], 
                        handle.spk.cal_r[2], handle.spk.cal_r[3]);
             #endif
         }
     } else {
         #ifdef DEBUG
         CSPL_DBGPRT("Successfully read from cal_r file");
         CSPL_DBGPRT("CAL_R values: [%02x,%02x,%02x,%02x]", 
                    handle.spk.cal_r[0], handle.spk.cal_r[1], 
                    handle.spk.cal_r[2], handle.spk.cal_r[3]);
         #endif
     }
 
     // init function pointers
     #ifdef DEBUG
     CSPL_DBGPRT("Initializing function pointers");
     #endif
     
     fp_platform_get_snd_device_name = spkr_prot_init_config_val.fp_platform_get_snd_device_name;
     fp_platform_get_pcm_device_id = spkr_prot_init_config_val.fp_platform_get_pcm_device_id;
     fp_get_usecase_from_list =  spkr_prot_init_config_val.fp_get_usecase_from_list;
     fp_disable_snd_device = spkr_prot_init_config_val.fp_disable_snd_device;
     fp_enable_snd_device = spkr_prot_init_config_val.fp_enable_snd_device;
     fp_disable_audio_route = spkr_prot_init_config_val.fp_disable_audio_route;
     fp_enable_audio_route = spkr_prot_init_config_val.fp_enable_audio_route;
     fp_audio_extn_get_snd_card_split = spkr_prot_init_config_val.fp_audio_extn_get_snd_card_split;
 
     pthread_mutex_init(&handle.fb_prot_mutex, NULL);
     #ifdef DEBUG
     CSPL_DBGPRT("Mutex initialized");
     #endif
 
     /* We assume calibration part is okay as there are no mixers for calibration */
     handle.spk.cal_ok = true;
     #ifdef DEBUG
     CSPL_DBGPRT("Calibration assumed OK");
     #endif
 
     #ifdef DEBUG
     CSPL_DBGPRT("Creating calibration thread");
     #endif
     (void)pthread_create(&handle.calibration_thread,
         (const pthread_attr_t *) NULL,
         cirrus_do_calibration, &handle);
     
     #ifdef DEBUG
     CSPL_DBGPRT("Speaker protection initialization completed");
     #endif
     return;
 }
 
 int spkr_prot_deinit()
 {
     ALOGV("%s: Entry", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("De-initializing speaker protection");
     #endif
 
     #ifdef DEBUG
     CSPL_DBGPRT("Joining failure detection thread");
     #endif
     pthread_join(handle.failure_detect_thread, NULL);
     
     #ifdef DEBUG
     CSPL_DBGPRT("Joining calibration thread");
     #endif
     pthread_join(handle.calibration_thread, NULL);
     
     #ifdef DEBUG
     CSPL_DBGPRT("Destroying mutex");
     #endif
     pthread_mutex_destroy(&handle.fb_prot_mutex);
 
     #ifdef DEBUG
     CSPL_DBGPRT("Speaker protection de-initialization completed");
     #endif
     ALOGV("%s: Exit", __func__);
     return 0;
 }
 
 static int cirrus_check_error_state(void) {
     uint8_t cspl_error[4] = { 0 };
     int ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Checking error state");
     #endif
 
     ret = cirrus_set_force_wake(true);
     if (ret < 0) {
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to set force wake: %d", ret);
         #endif
         return ret;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Reading CSPL_ERRORNO control");
     #endif
     ret = cirrus_get_mixer_array_by_name(CIRRUS_CTL_SPK_PROT_CSPL_ERRORNO,
                                          &cspl_error, 4);
     if (ret < 0) {
         ALOGE("%s: Cannot get CSPL status", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Failed to get CSPL error status: %d", ret);
         #endif
         goto exit;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("CSPL_ERRORNO values: [%02x,%02x,%02x,%02x]", 
                cspl_error[0], cspl_error[1], cspl_error[2], cspl_error[3]);
     #endif
 
     if (cspl_error[3] != 0) {
         ALOGE("%s: Error state detected!", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Error state detected, error code: %d", cspl_error[3]);
         #endif
         ret = -EREMOTEIO;
     } else {
         #ifdef DEBUG
         CSPL_DBGPRT("No error state detected");
         #endif
     }
 
 exit:
     #ifdef DEBUG
     CSPL_DBGPRT("Disabling force wake");
     #endif
     ret = cirrus_set_force_wake(false);
     #ifdef DEBUG
     CSPL_DBGPRT("Error check completed with result: %d", ret);
     #endif
     return ret;
 }
 
 static int cirrus_check_error_fatal(void) {
     int ret = 0;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Checking for fatal errors");
     #endif
 
     pthread_mutex_lock(&handle.fb_prot_mutex);
     #ifdef DEBUG
     CSPL_DBGPRT("Mutex locked, performing first error check");
     #endif
 
     ret = cirrus_check_error_state();
     if (ret == 0) {
         #ifdef DEBUG
         CSPL_DBGPRT("First check passed, no errors");
         #endif
         goto success;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Error detected, sleeping before retry");
     #endif
     /* Ouch! Error! Let's wait just a lil and retry to be extra sure... */
     usleep(CIRRUS_ERROR_DETECT_SLEEP_US);
     
     #ifdef DEBUG
     CSPL_DBGPRT("Performing second error check");
     #endif
     ret = cirrus_check_error_state();
     if (ret == 0) {
         #ifdef DEBUG
         CSPL_DBGPRT("Second check passed, false alarm");
         #endif
         goto success;
     }
 
     /* Error recovery: this should actually never happen, anyway... */
     ALOGE("%s: Cirrus DSP is in error state: resetting device", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("FATAL ERROR: Cirrus DSP is in error state, resetting device");
     #endif
     cirrus_do_reset();
 
     ALOGE("%s: Reset done, crashing HAL to reload firmware...", __func__);
     ALOGE("%s: Goodbye, infamous world...", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("FATAL ERROR: Crashing HAL to reload firmware");
     #endif
     abort();
 
 success:
     #ifdef DEBUG
     CSPL_DBGPRT("Error check completed successfully");
     #endif
     pthread_mutex_unlock(&handle.fb_prot_mutex);
     return ret;
 }
 
 static void *cirrus_failure_detect_thread() {
     ALOGD("%s: Entry", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("Failure detection thread started");
     #endif
 
     (void)cirrus_check_error_fatal();
     #ifdef DEBUG
     CSPL_DBGPRT("Failure detection completed");
     #endif
 
     ALOGD("%s: Exit ", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("Failure detection thread exiting");
     #endif
 
     pthread_exit(0);
     return NULL;
 }
 
 int spkr_prot_start_processing(__unused snd_device_t snd_device) {
     struct audio_device *adev = handle.adev_handle;
     int ret = 0;
 
     ALOGV("%s: Entry", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("Starting speaker protection processing for device: %d", snd_device);
     #endif
 
     if (!adev) {
         ALOGE("%s: Invalid params", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Invalid adev parameter");
         #endif
         return -EINVAL;
     }
 
     if (pthread_self() == handle.calibration_thread) {
         // Succeed without doing anything; the calibration already
         // selects the right paths, and we do not want the failure
         // detect thread to run just yet.
         ALOGV("%s: We are the calibration thread", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Called from calibration thread, skipping processing");
         #endif
         goto end;
     }
 
     pthread_mutex_lock(&handle.fb_prot_mutex);
     #ifdef DEBUG
     CSPL_DBGPRT("Mutex locked, current state: %d", handle.state);
     #endif
 
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
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot start processing during calibration or after error");
         #endif
         ret = -1;
         goto end;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Enabling force wake");
     #endif
     ret = cirrus_set_force_wake(true);
 
     #ifdef DEBUG
     CSPL_DBGPRT("Applying audio route for device: %s", 
                fp_platform_get_snd_device_name(snd_device));
     #endif
     audio_route_apply_and_update_path(adev->audio_route,
                                       fp_platform_get_snd_device_name(snd_device));
 
     if (handle.state == IDLE) {
         #ifdef DEBUG
         CSPL_DBGPRT("State is IDLE, creating failure detection thread");
         #endif
         (void)pthread_create(&handle.failure_detect_thread,
                     (const pthread_attr_t *) NULL,
                     cirrus_failure_detect_thread,
                     &handle);
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Setting state to PLAYBACK");
     #endif
     handle.state = PLAYBACK;
 end:
     #ifdef DEBUG
     if (pthread_self() != handle.calibration_thread) {
         CSPL_DBGPRT("Unlocking mutex");
     }
     #endif
     pthread_mutex_unlock(&handle.fb_prot_mutex);
 
     #ifdef DEBUG
     CSPL_DBGPRT("Start processing completed with result: %d", ret);
     #endif
     ALOGV("%s: Exit", __func__);
     return ret;
 }
 
 void spkr_prot_stop_processing(__unused snd_device_t snd_device) {
     struct audio_usecase *uc_info_tx;
     struct audio_device *adev = handle.adev_handle;
 
     ALOGV("%s: Entry", __func__);
     #ifdef DEBUG
     CSPL_DBGPRT("Stopping speaker protection processing for device: %d", snd_device);
     #endif
 
     pthread_mutex_lock(&handle.fb_prot_mutex);
     #ifdef DEBUG
     CSPL_DBGPRT("Mutex locked");
     #endif
 
     if (pthread_self() == handle.calibration_thread) {
         // This happens when stopping the device from calibration. We bailed
         // and never set PLAYBACK, so we should also never update the audio
         // route nor unconditionally set the state back to IDLE
         ALOGV("%s: We are the calibration thread", __func__);
         #ifdef DEBUG
         CSPL_DBGPRT("Called from calibration thread, skipping processing");
         #endif
         goto end;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Current state: %d", handle.state);
     #endif
     if (handle.state != PLAYBACK) {
         ALOGE("%s: Cannot stop processing, state is not PLAYBACK (but %d)",
               __func__, handle.state);
         #ifdef DEBUG
         CSPL_DBGPRT("Cannot stop processing, not in PLAYBACK state");
         #endif
         goto end;
     }
 
     #ifdef DEBUG
     CSPL_DBGPRT("Setting state to IDLE");
     #endif
     handle.state = IDLE;
 
     #ifdef DEBUG
     CSPL_DBGPRT("Resetting audio route for device: %s",
                fp_platform_get_snd_device_name(snd_device));
     #endif
     audio_route_reset_and_update_path(adev->audio_route,
                                       fp_platform_get_snd_device_name(snd_device));
 
 end:
     #ifdef DEBUG
     CSPL_DBGPRT("Unlocking mutex");
     #endif
     pthread_mutex_unlock(&handle.fb_prot_mutex);
 
     #ifdef DEBUG
     CSPL_DBGPRT("Disabling force wake");
     #endif
     (void)cirrus_set_force_wake(false);
 
     #ifdef DEBUG
     CSPL_DBGPRT("Stop processing completed");
     #endif
     ALOGV("%s: Exit", __func__);
 }
 
 bool spkr_prot_is_enabled() {
     #ifdef DEBUG
     CSPL_DBGPRT("Speaker protection is enabled");
     #endif
     return true;
 }
 
 int get_spkr_prot_snd_device(snd_device_t snd_device) {
     #ifdef DEBUG
     CSPL_DBGPRT("Mapping device %d to protected variant", snd_device);
     #endif
     
     switch(snd_device) {
     case SND_DEVICE_OUT_SPEAKER:
     case SND_DEVICE_OUT_SPEAKER_REVERSE:
         #ifdef DEBUG
         CSPL_DBGPRT("Mapped to SND_DEVICE_OUT_SPEAKER_PROTECTED");
         #endif
         return SND_DEVICE_OUT_SPEAKER_PROTECTED;
     case SND_DEVICE_OUT_SPEAKER_SAFE:
         #ifdef DEBUG
         CSPL_DBGPRT("Mapped to SND_DEVICE_OUT_SPEAKER_SAFE");
         #endif
         return SND_DEVICE_OUT_SPEAKER_SAFE;
     case SND_DEVICE_OUT_VOICE_SPEAKER:
         #ifdef DEBUG
         CSPL_DBGPRT("Mapped to SND_DEVICE_OUT_VOICE_SPEAKER_PROTECTED");
         #endif
         return SND_DEVICE_OUT_VOICE_SPEAKER_PROTECTED;
     default:
         #ifdef DEBUG
         CSPL_DBGPRT("No mapping needed, returning original device: %d", snd_device);
         #endif
         return snd_device;
     }
 }
 
 void spkr_prot_calib_cancel(__unused void *adev) {
     #ifdef DEBUG
     CSPL_DBGPRT("Calibration cancellation requested (no-op)");
     #endif
     return;
 }