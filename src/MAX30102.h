#ifndef MAX30102_H
#define MAX30102_H

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

int MAX_30102_start_collecting_data(struct i2c_dt_spec max30102);
int MAX_30102_end_collecting_data(struct i2c_dt_spec max30102);
int MAX_30102_SPO2_Heartrate(struct i2c_dt_spec max30102, uint8_t *spo2, uint32_t *heartbeat);


#endif