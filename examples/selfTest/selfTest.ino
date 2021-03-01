 /*!
  * @file  softReset.ino
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version     V1.0
  * @date        2021-02-24
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

  /*
   *  self mode:
   *      BMM150_SELF_TEST_NORMAL
   *      BMM150_SELF_TEST_ADVANCED
   */
  switch(bmm150.selfTest(BMM150_SELF_TEST_NORMAL))
  {
    case BMM150_OK:
      Serial.println("self test success!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_YZ_FAIL:
      Serial.println("x success yz failed!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_XZ_FAIL:
      Serial.println("y success xz failed!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_Z_FAIL:
      Serial.println("xy success z failed!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_XY_FAIL:
      Serial.println("z success xy failed!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_Y_FAIL:
      Serial.println("xz success y failed!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_X_FAIL:
      Serial.println("yz success x failed!");
      break;
    case BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL:
      Serial.println("xyz all failed!");
      break;
    default :
      Serial.println("self test error!");
      break;
  }
}


void loop() 
{
  delay(1000);
}