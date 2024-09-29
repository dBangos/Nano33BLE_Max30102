#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "MAX30102.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/hrs.h>

LOG_MODULE_REGISTER(BLE_Heartrate);

/*========================BLE Setup===========================*/
//UUID of the custom service for sending data
#define BT_UUID_CUSTOM_SERVICE_VAL BT_UUID_128_ENCODE(0x140c0497,0x8cec,0x45ad,0x95f7,0xd12a25fac2f9)
#define BT_UUID_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)
//UUID of the custom gyroscope characteristic
#define BT_UUID_CUSTOM_GYRO_CHAR_VAL BT_UUID_128_ENCODE(0x140c0497,0x8cec,0x45ad,0x95f7,0xd12a25fac2f0)
#define BT_UUID_CUSTOM_GYRO_CHAR BT_UUID_DECLARE_128(BT_UUID_CUSTOM_GYRO_CHAR_VAL)

/*Data to be sent over BLE*/
uint32_t Heartbeat;

/*========================BLE Setup===========================*/
/*BLE code from the Bluetooth HR zephyr sample*/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};

static void connected(struct bt_conn *conn, uint8_t err){
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason){
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void){
	int err;
	printk("Bluetooth initialized\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn){
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

static void hrs_notify(uint32_t max30102_heartrate){
	uint8_t heartrate = max30102_heartrate;
	bt_hrs_notify(heartrate);
}
/*======================MAX30102 Setup=========================*/
static const struct i2c_dt_spec max30102_dev = I2C_DT_SPEC_GET(DT_NODELABEL(max30102));
/*========================I2C Accel Start===========================*/
const struct device *const lsm6ds0 = DEVICE_DT_GET_ONE(st_lsm6ds0);
/*========================ADC Setup===========================*/
#define ADC_RESOLUTION 10
#define ADC_CHANNEL 1
const struct device *dev_adc = DEVICE_DT_GET(DT_NODELABEL(adc));
static const struct adc_channel_cfg channel_cfg = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_1));
int16_t force_value[1];
struct adc_sequence adc_seq ={
	.channels = BIT(ADC_CHANNEL),
	.buffer = force_value,
	.buffer_size = sizeof(force_value),
	.resolution = ADC_RESOLUTION
};

int main(void)
{
	/*======================MAX30102 Start=========================*/
	/*Check if device is ready*/
	while(!device_is_ready(max30102_dev.bus)){
			LOG_INF("MAX30102 is not ready\n");
	}
	/*========================ADC Start===========================*/
	int err;
	if (!device_is_ready(dev_adc)) {
		LOG_INF("%s: device not ready.\n", dev_adc->name);
		return 0;
	}
	err = adc_channel_setup(dev_adc, &channel_cfg);
	if (err != 0){
		LOG_INF("ADC channel setup failed with error %d\n",err);
		return 0;
	}
	/*========================I2C Accel Start===========================*/
	if (!device_is_ready(lsm6ds0)) {
		LOG_INF("%s: device not ready.\n", lsm6ds0->name);
		//return 0;
	}
	/*========================BLE Start===========================*/
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	bt_ready();
	bt_conn_auth_cb_register(&auth_cb_display);

	uint8_t SPO2;
	struct sensor_value accel_xyz[3];
	struct sensor_value gyro_xyz[3];
	bool activate_heartbeat = false;

	/*Values of previous measurements to calculate change*/
	float previous_accel_xyz[3] = {0};
	float previous_gyro_xyz[3] = {0};
	uint32_t previous_heartbeat[8] = {1};
	bool valid_heartbeat;
	uint8_t heartbeat_count = 0;
	/* If the force sensor return a value and the accel sensors
	values change turn on the heartbeat sensor. While the heartbeat
	sensor returns a non-zero value keep it active and sending values.
	Otherwise turn it off and wait for the accel and force sensors
	to turn it back on.
	*/
	while(1){
		err = adc_read(dev_adc, &adc_seq);
		if(err != 0){
			LOG_INF("ADC read failed with error %d\n",err);
			return 0;
		}
		/* Get sensor samples */
		if (sensor_sample_fetch(lsm6ds0) < 0) {
			LOG_INF("LSM6DS0 Sensor sample update error\n");
			return 0;
		}
		/* Get sensor data */
		sensor_channel_get(lsm6ds0, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
		sensor_channel_get(lsm6ds0, SENSOR_CHAN_GYRO_XYZ,gyro_xyz);
		/*If the accel/gyro moves and the force sensor is above a threshold, activate*/
		if((previous_gyro_xyz[0] != sensor_value_to_float(&gyro_xyz[0]) ||
			previous_gyro_xyz[1] != sensor_value_to_float(&gyro_xyz[1]) ||
			previous_gyro_xyz[2] != sensor_value_to_float(&gyro_xyz[2]) ||
			previous_accel_xyz[0] != sensor_value_to_float(&accel_xyz[0]) ||
			previous_accel_xyz[1] != sensor_value_to_float(&accel_xyz[1]) ||
			previous_accel_xyz[2] != sensor_value_to_float(&accel_xyz[2])) &&
			(force_value[0] > 500)){
				activate_heartbeat = true;
			}
		/*Store the previous values for future comparison*/
		previous_gyro_xyz[0] = sensor_value_to_float(&gyro_xyz[0]);
		previous_gyro_xyz[1] = sensor_value_to_float(&gyro_xyz[1]);
		previous_gyro_xyz[2] = sensor_value_to_float(&gyro_xyz[2]);
		previous_accel_xyz[0] = sensor_value_to_float(&accel_xyz[0]);
		previous_accel_xyz[1] = sensor_value_to_float(&accel_xyz[1]);
		previous_accel_xyz[2] = sensor_value_to_float(&accel_xyz[2]);
		while(activate_heartbeat){
			MAX_30102_start_collecting_data(max30102_dev);
			MAX_30102_SPO2_Heartrate(max30102_dev, &SPO2, &Heartbeat);
			LOG_INF("%d O2  %d BPM\n",SPO2,Heartbeat);
			hrs_notify(Heartbeat);
			/*Check if there is a valid heartbeat in the previous values,
			if not deactivate the sensor*/
			previous_heartbeat[heartbeat_count%8] = Heartbeat; 
			heartbeat_count++;
			valid_heartbeat = false;
			for (int i = 0; i < 8; i++){
				if(previous_heartbeat[i] != 0){
					valid_heartbeat = true;
				}
			}
			if(!valid_heartbeat){
				activate_heartbeat = false;
				/*Restoring the values so the next loop will have enough time for a new valid value*/
				for (int i = 0; i < 8; i++){
					previous_heartbeat[i] = 1; 
				}
			}
			k_sleep(K_SECONDS(1));
		}
		MAX_30102_end_collecting_data(max30102_dev);
		k_sleep(K_SECONDS(1));  // 1 second delay between updating sensor values
	}
}