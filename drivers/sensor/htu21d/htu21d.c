/*
 * Copyright (c) 2016 Firmwave
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <i2c.h>
#include <misc/byteorder.h>
#include <misc/util.h>
#include <kernel.h>
#include <sensor.h>
#include <misc/__assert.h>
#include <logging/log.h>
#include <device.h>
#include <drivers/gpio.h>

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(HTU21DF);


/** Default I2C address for the HTU21D. */
#define HTU21D_I2C_ADDRESS      CONFIG_HTU21D_I2C_ADDR

#define ERROR_I2C_TIMEOUT 	998
#define ERROR_BAD_CRC		999

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

#define USER_REGISTER_RESOLUTION_MASK 0x81
#define USER_REGISTER_RESOLUTION_RH12_TEMP14 0x00
#define USER_REGISTER_RESOLUTION_RH8_TEMP12 0x01
#define USER_REGISTER_RESOLUTION_RH10_TEMP13 0x80
#define USER_REGISTER_RESOLUTION_RH11_TEMP11 0x81

#define USER_REGISTER_END_OF_BATTERY 0x40
#define USER_REGISTER_HEATER_ENABLED 0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD 0x02

#define MAX_WAIT 100
#define DELAY_INTERVAL 100
#define MAX_COUNTER (MAX_WAIT/DELAY_INTERVAL)

u16_t readData;
struct htu21d_data {
	struct device *i2c;
	s32_t temp;
        s32_t humidity;
};

//Give this function the 2 byte message (measurement) and the check_value byte from the HTU21D
//If it returns 0, then the transmission was good
//If it returns something other than 0, then the communication was corrupted
//From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
//POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes


u8_t htu21d_checkCRC(u16_t message_from_sensor, u8_t check_value_from_sensor)
{
  //Test cases from datasheet:
  //message = 0xDC, checkvalue is 0x79
  //message = 0x683A, checkvalue is 0x7C
  //message = 0x4E85, checkvalue is 0x6B

  u32_t remainder = (u32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
  remainder |= check_value_from_sensor; //Add on the check value

  u32_t divsor = (u32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {
    //Serial.print("remainder: ");
    //Serial.println(remainder, BIN);
    //Serial.print("divsor:    ");
    //Serial.println(divsor, BIN);
    //Serial.println();

    if ( remainder & (u32_t)1 << (23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return (u8_t)remainder;
}
//Given a command, reads a given 2-byte value with CRC from the HTU21D
static int htu21d_readValue(struct htu21d_data *drv_data,
			   u8_t reg)
{
  //Request a humidity reading
        u8_t readVal[3] = {0};
        u8_t sendbuf[2] = {0};
        sendbuf[0] = reg;
        u8_t crc;
        int ret;
        int counter;
        int validResult = 1;
         //Hang out while measurement is taken. datasheet says 50ms, practice may call for more
        
          validResult = i2c_write(drv_data->i2c, sendbuf, 1, HTU21D_I2C_ADDRESS);
          if (validResult < 0) {
                printk("Failed to access to Device\n");
		return -EIO;
          }
          for (counter = 0; counter < MAX_COUNTER; counter++){
            k_sleep(DELAY_INTERVAL);
            validResult = i2c_read(drv_data->i2c,
			   readVal, 3, HTU21D_I2C_ADDRESS);
            if (validResult == 0) break;
          }


  	if (validResult < 0) {
                printk("Failed to read from 0x%x: I2C TIMEOUT ERROR\n", reg);
		return -EIO;
	}
        u16_t rawValue = ((uint16_t) readVal[0] << 8) | (uint16_t) readVal[1];
	crc = readVal[2];
        if (htu21d_checkCRC(rawValue, crc) != 0)
         { 
           printk("Failed to checksum from 0x%x: BAD CRC ERROR\n", reg);
           return (ERROR_BAD_CRC); //Error out
         }
        readData = rawValue & 0xFFFC; // Zero out the status bits 
	return 0;
}
//Read the humidity
/*******************************************************************************************/
//Calc humidity and return it to the user
//Returns 998 if I2C timed out
//Returns 999 if CRC is wrong
static int htu21d_readHumidity (struct htu21d_data *drv_data)
{
        
        int ret;
//        u16_t* rawHumidity;
        ret = htu21d_readValue(drv_data, TRIGGER_HUMD_MEASURE_NOHOLD);
        if (ret != 0) 
        {
            printk(" Failed to read Humidity");
            return -EIO;
        }
  
        //if(&rawHumidity == ERROR_I2C_TIMEOUT || &rawHumidity == ERROR_BAD_CRC) return(&rawHumidity);

         //Given the raw humidity data, calculate the actual relative humidity
        float tempRH = readData * (125.0 / 65536.0); //2^16 = 65536
        float rh = tempRH - 6.0; //From page 14
        drv_data->humidity = (s32_t)rh * 1000000;
        return 0;
}

//Read the temperature
/*******************************************************************************************/
//Calc temperature and return it to the user
//Returns 998 if I2C timed out
//Returns 999 if CRC is wrong
static int htu21d_readTemperature(struct htu21d_data *drv_data)
{
      int ret;
      //u16_t* rawTemperature;
      ret = htu21d_readValue(drv_data, TRIGGER_TEMP_MEASURE_NOHOLD);
      if (ret != 0) 
        {
            printk(" Failed to read Temperature\n");
            return -EIO;
        }
      //Given the raw temperature data, calculate the actual temperature
      float tempTemperature = readData * (175.72 / 65536.0); //2^16 = 65536
      float realTemperature = tempTemperature - 46.85; //From page 14
      drv_data->temp = (s32_t)realTemperature * 1000000;
      return 0;
}
//Set sensor resolution
///*******************************************************************************************/
//Sets the sensor resolution to one of four levels
//Page 12:
// 0/0 = 12bit RH, 14bit Temp
// 0/1 = 8bit RH, 12bit Temp
// 1/0 = 10bit RH, 13bit Temp
// 1/1 = 11bit RH, 11bit Temp
//Power on default is 0/0
//
//static int htu21d_setResolution(int resolution)
//{
//  int userRegister = htu21d_readUserRegister(); //Go get the current register state
//  userRegister &= B01111110; //Turn off the resolution bits
//  resolution &= B10000001; //Turn off all other bits but resolution bits
//  userRegister |= resolution; //Mask in the requested resolution bits
//
//  //Request a write to user register
//  writeUserRegister(userRegister);
//}
//
//Read the user register
//static int htu21d_readUserRegister(void)
//{
//  int ret;
//
//  //Request the user register
//  ret = i2c_reg_read_byte(i2c_dev, DT_AMS_ENS210_0_BASE_ADDRESS,
//				 ENS210_REG_SYS_CTRL, *(u8_t *)&sys_ctrl);
//                                 (drv_data->i2c, HTU21D_I2C_ADDRESS,
//			   reg, readVal, 3)
//
//  userRegister = _i2cPort->read();
//
//  return (userRegister);
//}
//
//void HTU21D::writeUserRegister(byte val)
//{
//  _i2cPort->beginTransmission(HTU21D_ADDRESS);
//  _i2cPort->write(WRITE_USER_REG); //Write to the user register
//  _i2cPort->write(val); //Write the new resolution bits
//  _i2cPort->endTransmission();
//}
//


static int htu21d_attr_set(struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
//	struct htu21d_data *drv_data = dev->driver_data;
//	s64_t value;
//	u16_t cr;
//
//	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
//		return -ENOTSUP;
//	}
//
//	switch (attr) {
//	case SENSOR_ATTR_FULL_SCALE:
//		/* the sensor supports two ranges -55 to 128 and -55 to 150 */
//		/* the value contains the upper limit */
//		if (val->val1 == 128) {
//			value = 0x0000;
//		} else if (val->val1 == 150) {
//			value = HTU21D_EM_BIT;
//		} else {
//			return -ENOTSUP;
//		}
//
//		if (htu21d_reg_update(drv_data, HTU21D_REG_CONFIG,
//				      HTU21D_EM_BIT, value) < 0) {
//			LOG_DBG("Failed to set attribute!");
//			return -EIO;
//		}
//
//		return 0;
//
//	case SENSOR_ATTR_SAMPLING_FREQUENCY:
//		/* conversion rate in mHz */
//		cr = val->val1 * 1000 + val->val2 / 1000;
//
//		/* the sensor supports 0.25Hz, 1Hz, 4Hz and 8Hz */
//		/* conversion rate */
//		switch (cr) {
//		case 250:
//			value = 0x0000;
//			break;
//
//		case 1000:
//			value = HTU21D_CR0_BIT;
//			break;
//
//		case 4000:
//			value = HTU21D_CR1_BIT;
//			break;
//
//		case 8000:
//			value = HTU21D_CR0_BIT | HTU21D_CR1_BIT;
//			break;
//
//		default:
//			return -ENOTSUP;
//		}
//
//		if (htu21d_reg_update(drv_data, HTU21D_REG_CONFIG,
//				      HTU21D_CR0_BIT | HTU21D_CR1_BIT,
//				      value) < 0) {
//			LOG_DBG("Failed to set attribute!");
//			return -EIO;
//		}
//
//		return 0;
//
//	default:
//		return -ENOTSUP;
//	}
//
	return 0;
}

static int htu21d_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct htu21d_data *drv_data = dev->driver_data;
	u16_t val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP || chan = SENSOR_CHAN_HUMIDITY);

	if (htu21d_readHumidity(drv_data) < 0) {
		return -EIO;
	}
        if (htu21d_readTemperature(drv_data) < 0) {
		return -EIO;
	}
       return 0;
}

static int htu21d_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
        struct htu21d_data *drv_data = dev->driver_data;
	//s32_t uval;
	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		val->val1 = drv_data->temp / 1000000;
		val->val2 = drv_data->temp % 1000000;
		break;
	case  SENSOR_CHAN_HUMIDITY:
                val->val1 = drv_data->humidity / 1000000;
		val->val2 = drv_data->humidity % 1000000;

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
static const struct sensor_driver_api htu21d_driver_api = {
	.attr_set = htu21d_attr_set,
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

int htu21d_init(struct device *dev)
{
	struct htu21d_data *drv_data = dev->driver_data;

        /* Set enable pin as output and write 1 to enable */

	drv_data->i2c = device_get_binding(CONFIG_HTU21D_I2C_MASTER_DEV_NAME);
	if (drv_data->i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
			    CONFIG_HTU21D_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}
       
	return 0;
}

static struct htu21d_data htu21d_driver;

DEVICE_AND_API_INIT(htu21d, CONFIG_HTU21D_NAME, htu21d_init, &htu21d_driver,
	    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &htu21d_driver_api);
