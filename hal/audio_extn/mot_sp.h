#ifndef MOT_SP_H_
#define MOT_SP_H_

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

// Payload struct for getting calibration result from DSP module
struct __attribute__((__packed__)) cirrus_cal_t
{
    uint32_t cal_data;
    uint32_t cal_ambient;
    bool cal_ok;
};

// Cirrus playback session structure
struct cirrus_playback_session
{
    void *adev_handle;
    struct cirrus_cal_t *spk;
    struct cirrus_cal_t *rcv;
};

int read_from_file(const char *file_path, uint32_t *value_out);
void cspl_apply_calibration(device_identifier type);
void audio_extn_spkr_prot_calib_init(void);
void *audio_extn_mot_spkr_prot_init(void *adev);
void audio_extn_mot_spkr_prot_deinit(void *handle);

#endif /* MOT_SP_H_ */