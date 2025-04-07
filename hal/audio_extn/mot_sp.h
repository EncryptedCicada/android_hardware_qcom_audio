#ifndef MOT_SP_H_
#define MOT_SP_H_

int read_from_file(const char *file_path, uint32_t *value_out);
void cspl_apply_calibration(device_identifier type);
void spkr_prot_calib_init(void);
void spkr_prot_init(void *adev, spkr_prot_init_config_t spkr_prot_init_config_val);
void spkr_prot_deinit(void);

#endif /* MOT_SP_H_ */