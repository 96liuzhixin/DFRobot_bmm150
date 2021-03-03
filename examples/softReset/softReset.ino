 /*!
  * @file  softReset.ino
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version     V1.0
  * @date        2021-02-07
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_bmm150
  */

#include "DFRobot_bmm150.h"

#define I2C_COMMUNICATION
#ifdef  I2C_COMMUNICATION
  #define I2C_ADDRESS    0x13
  DFRobot_BMM150_I2C bmm150(&Wire ,I2C_ADDRESS);
#else
  #define CS_PIN         10
  DFRobot_BMM150_SPI bmm150(&SPI ,CS_PIN);
#endif

void setup() 
{
  Serial.begin(115200);
  while(!bmm150.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  } Serial.println("devices connect success!");
  
  while(!bmm150.sensorInit()){
    Serial.println("bmm150 init failed ,Please try again!");
  } Serial.println("bmm150 init success!");

  /*
   *  set operation mode
   *      BMM150_POWERMODE_NORMAL
   *      BMM150_POWERMODE_FORCED
   *      BMM150_POWERMODE_SLEEP
   *      BMM150_POWERMODE_SUSPEND 
   */
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);

  switch(bmm150.getOperationMode())
  {
    case BMM150_POWERMODE_NORMAL:
      Serial.println("bmm150 is normal mode!");
      break;
    case BMM150_POWERMODE_FORCED:
      Serial.println("bmm150 is forced mode!");
      break;
    case BMM150_POWERMODE_SLEEP:
      Serial.println("bmm150 is sleep mode!");
      break;
    case BMM150_POWERMODE_SUSPEND:
      Serial.println("bmm150 is suspend mode!");
      break;
    default:
      Serial.println("error mode!");
      break;
  }

#if 0
  Serial.print("chip id = 0x");
  Serial.println(bmm150.getChipID(),HEX);

  /*
   *  set rate
   *      BMM150_DATA_RATE_10HZ   (default rate)
   *      BMM150_DATA_RATE_02HZ
   *      BMM150_DATA_RATE_06HZ
   *      BMM150_DATA_RATE_08HZ
   *      BMM150_DATA_RATE_15HZ
   *      BMM150_DATA_RATE_20HZ
   *      BMM150_DATA_RATE_25HZ
   *      BMM150_DATA_RATE_30HZ
   */
  bmm150.setRate(BMM150_DATA_RATE_30HZ);

  /*
   *  get rate
   *      BMM150_DATA_RATE_10HZ   (default rate)
   *      BMM150_DATA_RATE_02HZ
   *      BMM150_DATA_RATE_06HZ
   *      BMM150_DATA_RATE_08HZ
   *      BMM150_DATA_RATE_15HZ
   *      BMM150_DATA_RATE_20HZ
   *      BMM150_DATA_RATE_25HZ
   *      BMM150_DATA_RATE_30HZ
   */
  uint8_t rate = bmm150.getRate();
  switch(rate)
  {
    case BMM150_DATA_RATE_10HZ:
      Serial.println("rate is 10hz");
      break;
    case BMM150_DATA_RATE_02HZ:
      Serial.println("rate is 2hz");
      break;
    case BMM150_DATA_RATE_06HZ:
      Serial.println("rate is 6hz");
      break;
    case BMM150_DATA_RATE_08HZ:
      Serial.println("rate is 8hz");
      break;
    case BMM150_DATA_RATE_15HZ:
      Serial.println("rate is 15hz");
      break;
    case BMM150_DATA_RATE_20HZ:
      Serial.println("rate is 20hz");
      break;
    case BMM150_DATA_RATE_25HZ:
      Serial.println("rate is 25hz");
      break;
    case BMM150_DATA_RATE_30HZ:
      Serial.println("rate is 30hz");
      break;
  }
  /*
   *  Set the measurement of the xyz axis channel 
   *
   *  channel x selection:
   *      enable x  : MEASUREMENT_X_ENABLE  (Default x-axis channel enabled)
   *      disable x : MEASUREMENT_X_DISABLE
   *  channel y selection:
   *      enable y  : MEASUREMENT_Y_ENABLE  (Default y-axis channel enabled)
   *      disable y : MEASUREMENT_Y_DISABLE
   *  channel y selection:
   *      enable z  : MEASUREMENT_Z_ENABLE  (Default z-axis channel enabled)
   *      disable z : MEASUREMENT_Z_DISABLE
   */
  bmm150.setMeasurementXYZ(MEASUREMENT_X_ENABLE ,MEASUREMENT_Y_ENABLE ,MEASUREMENT_Z_ENABLE);
  
  /*
   *  channel ? selection:
   *      channel x : CHANNEL_X
   *      channel y : CHANNEL_Y
   *      channel z : CHANNEL_Z
   *  measurement status
   *      1 is  enable measurement
   *      0 is  disable measurement
   */
  uint8_t channel = bmm150.getMeasurementStateXYZ(CHANNEL_X);
  if(channel == 1){
    Serial.println("channel x is enable!");
  }else{
    Serial.println("channel x is disable!");
  }
#endif
  bmm150.softReset();    // After software reset ,resume sleep mode ,(Suspended mode cannot be reset)
}


void loop() 
{
  switch(bmm150.getOperationMode())
  {
    case BMM150_POWERMODE_NORMAL:
      Serial.println("bmm150 is normal mode!");
      break;
    case BMM150_POWERMODE_FORCED:
      Serial.println("bmm150 is forced mode!");
      break;
    case BMM150_POWERMODE_SLEEP:
      Serial.println("bmm150 is sleep mode!");
      break;
    case BMM150_POWERMODE_SUSPEND:
      Serial.println("bmm150 is suspend mode!");
      break;
    default:
      Serial.println("error mode!");
      break;
  }
  delay(1000);
}