#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "Wire.h"
#include "bmi2.h"
#include "bmi270.h"

/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_I2C UINT8_C(1)

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_SPI UINT8_C(0)

static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev);
#if BMI270_INTERFACE_I2C == 1
/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t i2c_bus;
#elif BMI270_INTERFACE_SPI == 1
static uint8_t spi_bus;
#endif


// Interrupt Pin - Digital Pin2
// Most Arduino Boards can use digital pin 2 as pin interrupt
const byte interruptPin = 2;
#define BMI270_CS       10

/* Create an instance of sensor data structure */
struct bmi2_sensor_data sensor_data = {.type = BMI2_STEP_COUNTER};

/* Structure to define BMI2 sensor configurations */
struct bmi2_dev bmi2;


void bmi2xy_hal_delay_usec(uint32_t period_us, void *intf_ptr)
{
  delayMicroseconds(period_us);
}

/*! This API is used to perform I2C read operation with sensor */
int8_t bmi2xy_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //uint8_t dev_id = 0x68;
  uint8_t* dev_id = (uint8_t *)intf_ptr;

  rslt = BMI270_read_i2c(*dev_id, reg_addr, reg_data, length);

  return rslt;
}

/*! This API is used to perform I2C write operations with sensor */
int8_t bmi2xy_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //    uint8_t dev_id = 0x68;

  uint8_t* dev_id = (uint8_t *)intf_ptr;
  rslt = BMI270_write_i2c(*dev_id, reg_addr, (uint8_t *)reg_data, length);

  return rslt;
}

int8_t BMI270_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /* dev_addr: I2C device address.
    reg_addr: Starting address for writing the data.
    reg_data: Data to be written.
    count: Number of bytes to write */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();

  if (returned)
  {
    /*
      case 1:Data too long to fit in transmit buffer
          break;
      case 2:received NACK on transmit of address.
          break;
      case 3:received NACK on transmit of data."
          break;
      case 4:Unspecified error.
          break;
      default:Unexpected Wire.endTransmission() return code:
    */
    return returned;
  }

  // Requests the required number of bytes from the sensor
  Wire.requestFrom((int)dev_addr, (int)count);

  uint16_t i;
  // Reads the requested number of bytes into the provided array
  for (i = 0; (i < count) && Wire.available(); i++)
  {
    reg_data[i] = Wire.read(); // This is for the modern Wire library
  }

  // This must return 0 on success, any other value will be interpreted as a communication failure.
  return 0;
}

int8_t BMI270_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /*  dev_addr: I2C device address.
    reg_addr: Starting address for reading the data.
    reg_data: Buffer to take up the read data.
    count: Number of bytes to read. */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);

  uint16_t i;
  // Writes the requested number of bytes from the provided array
  for (i = 0; i < count; i++)
  {
    Wire.write(reg_data[i]); // This is for the modern Wire library
  }
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();
  /*
      case 1:Data too long to fit in transmit buffer
      case 2:received NACK on transmit of address.
      case 3:received NACK on transmit of data.
      case 4:Unspecified error.
      default:Unexpected Wire.endTransmission() return code:
  */
  // This must return 0 on sucess, any other value will be interpretted as a communication failure.
  return returned;
}

int8_t bmi2_step_counter_set_config(struct bmi2_dev *bmi2_dev)
{
  /* Variable to define result */
  int8_t rslt;

  /* Initialize interrupts for gyroscope */
  struct bmi2_sens_int_config sens_int = { .type = BMI2_STEP_COUNTER, .hw_int_pin = BMI2_INT2 };

  /* List the sensors which are required to enable */
  uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_STEP_COUNTER};

  /* Structure to define the type of the sensor and its configurations */
  struct bmi2_sens_config config;

  /* Configure type of feature */
  config.type = BMI2_STEP_COUNTER;



  /* Enable the selected sensors */
  rslt = bmi2_sensor_enable(sens_list, 2, bmi2_dev);

  if (rslt == BMI2_OK)
  {
    /* Get default configurations for the type of feature selected */
    rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {
      config.cfg.step_counter.watermark_level = 1;

      rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
      if (rslt == BMI2_OK) {
        /* Map interrupt to pins */
        rslt = bmi2_map_feat_int(&sens_int, 1, bmi2_dev);
      } else {
        Serial.println("Set Sensor Config failed");
      }


    } else {
      Serial.println("Get sensor config failed");
    }
  } else {
    Serial.println("Sensor Enable Failed");
  }

  return rslt;
}

Adafruit_ST7789 tft = Adafruit_ST7789(9, 8, 11, 10);

void setup() {
  Wire.begin();

  /* Variable to define result */
  int8_t rslt;

  //  Wire.setModule(0);
  Serial.begin(115200);
  
  i2c_bus = BMI2_I2C_PRIM_ADDR;
  /* To initialize the hal function */
  bmi2.intf_ptr = &i2c_bus;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.read = bmi2xy_hal_i2c_bus_read;
  bmi2.write = bmi2xy_hal_i2c_bus_write;
  bmi2.read_write_len = 30;
  bmi2.delay_us = bmi2xy_hal_delay_usec;

  /* Config file pointer should be assigned to NULL, so that default file address is assigned in bmi270_init */
  bmi2.config_file_ptr = NULL;

  /* Initialize bmi270 */
  rslt = bmi270_init(&bmi2);

  if (rslt == BMI2_OK) {
    Serial.println("BMI270 Init Done");
  }


  rslt = bmi2_step_counter_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    Serial.println("Step counter enabled");
    Serial.println(" Move the board to recognize some steps");
    tft.init(240, 240);
    tft.fillScreen(ST77XX_BLACK);

    tft.setCursor(15, 30);

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(5);
    tft.println("Hello");
  }

  delay(250);


}


void loop() {

  int8_t rslt;
  uint16_t int_status = 0;

  rslt = bmi2_get_int_status(&int_status, &bmi2);

  if (rslt == BMI2_OK) {

    if (int_status & BMI270_STEP_CNT_STATUS_MASK) {
      Serial.println("Step Detected");

      rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi2);
      if (rslt == BMI2_OK) {
        Serial.println(sensor_data.sens_data.step_counter_output);
        tft.setTextWrap(false);
        tft.setCursor(15, 30);
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(5);
        tft.println(sensor_data.sens_data.step_counter_output);
      }
    }
  }

}
