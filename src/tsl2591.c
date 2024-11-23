#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_test);


/*========================USB Setup===========================*/
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static struct usbd_contex *sample_usbd;

static int enable_usb_device_next(void)
{
	int err;

	sample_usbd = sample_usbd_init_device();
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	err = usbd_enable(sample_usbd);
	if (err) {
		return err;
	}

	return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */
/*======================I2C Setup=========================*/
#define I2C0_NODE DT_NODELABEL(tsl2591)
#define TSL2591_I2C_ADDRESS 0x28

#define ENABLE_REGISTER_ADDRESS 0x00
#define CONFIG_REGISTER_ADDRESS 0x01

#define INTERRUPT_AILTL_REGISTER 0x04
#define INTERRUPT_AILTH_REGISTER 0x05
#define INTERRUPT_AIHTL_REGISTER 0x06
#define INTERRUPT_AIHTH_REGISTER 0x07
#define INTERRUPT_NPAILTL_REGISTER 0x08
#define INTERRUPT_NPAILTH_REGISTER 0x09
#define INTERRUPT_NPAIHTL_REGISTER 0x0A
#define INTERRUPT_NPAIHTH_REGISTER 0x0B
#define PERSIST_REGISTER 0x0C

#define ID_REGISTER_ADDRESS 0x12
#define C0DATAL_REGISTER_ADDRESS 0x14
#define C0DATAH_REGISTER_ADDRESS 0x15
#define C1DATAL_REGISTER_ADDRESS 0x16
#define C1DATAH_REGISTER_ADDRESS 0x17

#define COMMAND_CMD BIT(7)
#define COMMAND_TRANSACTION_NORMAL BIT(5)

static const struct i2c_dt_spec tsl2591_dev = I2C_DT_SPEC_GET(I2C0_NODE);
uint8_t als_data0_register_buffer[2]={0};
uint8_t als_data1_register_buffer[2]={0};
uint8_t part_id_buffer = 0;

uint8_t enable_command[2]= {(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | ENABLE_REGISTER_ADDRESS),0b10010011};
uint8_t disable_command[2]= {(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | ENABLE_REGISTER_ADDRESS),0b00000000};


static const int tsl2591_als_interrupt_config(const struct i2c_dt_spec dev,uint8_t low_lower,uint8_t low_upper,uint8_t high_lower,uint8_t high_upper, uint8_t peristence_value){
	int err;
	uint8_t address_byte_buffer[2]={(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_AILTL_REGISTER), low_lower};
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	address_byte_buffer[0] =(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_AILTH_REGISTER);
	address_byte_buffer[1] = low_upper;
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	address_byte_buffer[0] =(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_AIHTL_REGISTER);
	address_byte_buffer[1] = high_lower;
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	address_byte_buffer[0] = (COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_AIHTH_REGISTER);
	address_byte_buffer[1] = high_upper;
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	if(peristence_value>16){
		address_byte_buffer[1] = 16;
	}
	else{
		address_byte_buffer[1] = peristence_value;
	}
	address_byte_buffer[0] = (COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | PERSIST_REGISTER);
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));
	return 0;
}

static const int tsl2591_np_interrupt_config(const struct i2c_dt_spec dev,uint8_t low_lower,uint8_t low_upper,uint8_t high_lower,uint8_t high_upper){
	int err;
	uint8_t address_byte_buffer[2]={(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_NPAILTL_REGISTER), low_lower};
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	address_byte_buffer[1] = low_upper;
	address_byte_buffer[0] = (COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_NPAILTH_REGISTER);
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	address_byte_buffer[1] = high_lower;
	address_byte_buffer[0] = (COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_NPAIHTL_REGISTER);
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));

	address_byte_buffer[1] = high_upper;
	address_byte_buffer[0] = (COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | INTERRUPT_NPAIHTH_REGISTER);
	err=i2c_write_dt(&dev,address_byte_buffer, sizeof(address_byte_buffer));
	return 0;
}

//TODO: implement the different levels of gain and integration time
uint8_t als_config_command[2] = {(COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | CONFIG_REGISTER_ADDRESS),0b00010000};
/*Set the ALS gain and integration time*/
static const int tsl2591_config(){
	int err;
	err = i2c_write_dt(&tsl2591_dev,als_config_command,sizeof(als_config_command));
	if(err != 0){
		printk("Failed while configuring TSL2591 \n");
	}
	return 0;
}

static const int tsl2591_read(const struct i2c_dt_spec dev, uint8_t reg_address, uint8_t *read_buf, uint8_t read_size){
	int err;
	uint8_t reg_cmd = (COMMAND_CMD | COMMAND_TRANSACTION_NORMAL | reg_address);
	err = i2c_write_read_dt(&dev,  &reg_cmd, 1U, read_buf, read_size);
	if(err != 0){
		printk("Failed while reading TSL2591 %x \n",reg_address);
	}
	return 0;
}
static const int tsl2591_sample_fetch(const struct i2c_dt_spec dev, uint8_t *ch0_buffer, uint8_t *ch1_buffer){
	int err;
	printf("Enter sample\n");
	/*Enable the sensor*/
	err=i2c_write_dt(&dev,enable_command,sizeof(enable_command));
	if(err != 0){
		printk("Failed while enabling TSL2591 \n");
	}
	k_sleep(K_MSEC(120));
	/*Read the sensor data*/
	err = tsl2591_read(dev, C0DATAL_REGISTER_ADDRESS, ch0_buffer, 2U);

	if(err != 0){
		printk("Failed while reading TSL2591 channel0\n");
	}
	err = tsl2591_read(dev, C1DATAL_REGISTER_ADDRESS, ch1_buffer, 2U);

	if(err != 0){
		printk("Failed while reading TSL2591 channel1\n");
	}
	/*Disable the sensor*/
	err=i2c_write_dt(&dev,disable_command,sizeof(disable_command));
	if(err != 0){
		printk("Failed while disabling TSL2591 \n");
	}
	return 0;
}

/**************************************************************/
/****************************Main******************************/
/**************************************************************/
int main(void)
{
	/*========================USB Start===========================*/
    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;
	#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		if (enable_usb_device_next()) {
			return 0;
		}
	#else
		if (usb_enable(NULL)) {
			return 0;
		}
	#endif
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}
	/*======================I2C Start=========================*/
	/*Check if device is ready*/
	if(!device_is_ready(tsl2591_dev.bus)){
			printf("TSL2591 is not ready\n");
	}
	int err;
	/*Check if device part ID matches*/
	err = tsl2591_read(tsl2591_dev, ID_REGISTER_ADDRESS, &part_id_buffer, 1U);
	if(err != 0){
		printk("Failed while reading TSL2591 ID \n");
	}
	if(part_id_buffer != 0x50){
		printk("TSL2591 part ID does not match\n");
	}
	/*Enable the sensor*/
	err=i2c_write_dt(&tsl2591_dev,enable_command,sizeof(enable_command));
	if(err != 0){
		printk("Failed while enabling TSL2591 \n");
	}
	tsl2591_config();
	/***************************************************************/
	/****************************while******************************/
	/***************************************************************/
	tsl2591_np_interrupt_config(tsl2591_dev,90,0,13,0);
	while (1) {
		//tsl2591_sample_fetch(tsl2591_dev, als_data0_register_buffer, als_data1_register_buffer);
		printf("%d %d %d %d\n", als_data0_register_buffer[0], als_data0_register_buffer[1], als_data1_register_buffer[0], als_data1_register_buffer[1]);
        printf("Printing fine\n");
		k_sleep(K_SECONDS(2));
	}
}