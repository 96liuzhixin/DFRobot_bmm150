 /*!
  * @file  magDrdyInterrupt.ino
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version     V1.0
  * @date        2021-02-23
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
   *  preset_mode
   *    BMM150_PRESETMODE_LOWPOWER
   *    BMM150_PRESETMODE_REGULAR
   *    BMM150_PRESETMODE_HIGHACCURACY
   *    BMM150_PRESETMODE_ENHANCED
   */
  bmm150.setPresetMode(BMM150_PRESETMODE_LOWPOWER);

  /*
   *  set data readly interrupt pin
   *    ENABLE_DRDY
   *    DISABLE_DRDY    (default mode)
   *
   *  set pin polarity  (Active level)
   *    POKARITY_HIGH   (default active high level )
   *    POKARITY_LOW
   */
  bmm150.setDataReadlyInterruptPin(ENABLE_DRDY ,POKARITY_HIGH);
  
}


void loop() 
{
  struct bmm150MagData magData;
  if(bmm150.getDataReadlyState()){
    bmm150.getGeomagneticData(&magData);
    Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
    Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
    Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
    Serial.println();
  }
  delay(1000);
}