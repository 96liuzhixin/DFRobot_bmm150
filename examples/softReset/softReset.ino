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

#define I2C_ADDRESS     0x13
DFRobot_bmm150_I2C bmm150(&Wire ,I2C_ADDRESS);
//#define CS_PIN          10
//DFRobot_bmm150_SPI bmm150(&SPI ,CS_PIN);

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