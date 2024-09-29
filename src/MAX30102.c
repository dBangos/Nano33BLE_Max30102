#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

int MAX_30102_start_collecting_data(struct i2c_dt_spec max30102){
	uint8_t wbuf[2] = {0,1};
	if (i2c_burst_write_dt(&max30102,0x20,wbuf, sizeof(wbuf))){
		//LOG_ERR("Failed while starting MAX30102 \n");
	}
	return 0;
}
int MAX_30102_end_collecting_data(struct i2c_dt_spec max30102){
	uint8_t wbuf[2] = {0,2};
	if (i2c_burst_write_dt(&max30102,0x20,wbuf, sizeof(wbuf))){
		//LOG_ERR("Failed while starting MAX30102 \n");
	}
	return 0;
} 
int MAX_30102_SPO2_Heartrate(struct i2c_dt_spec max30102, uint8_t *spo2, uint32_t *heartbeat){
	uint8_t buffer[8];
	if(i2c_burst_read_dt(&max30102,0x0c,buffer,sizeof(buffer))){
		//LOG_ERR("Failed while reading MAX30102\n");
	}	
	// Based on the code from https://github.com/DFRobot/DFRobot_BloodOxygen_S/blob/master/DFRobot_BloodOxygen_S.cpp
	*spo2 = buffer[0];
	*heartbeat = ((uint32_t)buffer[2] << 24) | ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | ((uint32_t)buffer[5]);
	return 0;
} 