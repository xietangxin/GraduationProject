#ifndef __DRV_MPU9250_H__
#define __DRV_MPU9250_H__

#include <rthw.h>
#include <rtdevice.h>

rt_err_t mpu9250_write_reg(rt_uint8_t reg, rt_uint8_t data);
rt_err_t mpu9250_read_reg(rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf);
rt_err_t mpu9250_gyro_fsr_set(rt_uint8_t fsr);
rt_err_t mpu9250_accel_fsr_set(rt_uint8_t fsr);
rt_err_t mpu9250_lpf_set(rt_uint16_t lpf);
rt_err_t mpu9250_sample_rate_set(rt_uint16_t rate);
rt_err_t mpu9250_temperature_get(rt_int16_t *temperature);
rt_err_t mpu9250_gyroscope_get(rt_int16_t *gx, rt_int16_t *gy, rt_int16_t *gz);
rt_err_t mpu9250_accelerometer_get(rt_int16_t *ax, rt_int16_t *ay, rt_int16_t *az);
int mpu9250_hw_init(void);

#endif
