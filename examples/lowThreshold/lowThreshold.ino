 /*!
  * @file  lowThreshold.ino
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version     V1.0
  * @date        2021-02-23
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

  /*
   *  preset_mode
   *    BMM150_PRESETMODE_LOWPOWER
   *    BMM150_PRESETMODE_REGULAR
   *    BMM150_PRESETMODE_HIGHACCURACY
   *    BMM150_PRESETMODE_ENHANCED
   */
  bmm150.setPresetMode(BMM150_PRESETMODE_LOWPOWER);

  /*
   *  channel x selection:
   *      LOW_INTERRUPT_X_ENABLE
   *      LOW_INTERRUPT_X_DISABLE
   *
   *  channel y selection:
   *      LOW_INTERRUPT_Y_ENABLE
   *      LOW_INTERRUPT_Y_DISABLE
   *
   *  channel y selection:
   *      LOW_INTERRUPT_Z_ENABLE
   *      LOW_INTERRUPT_Z_DISABLE
   *
   *  low threshold range is (-128 to 127)
   *  The value is multiplied by 16 by default.  (-128*16 to 127*16)
   */
  bmm150.setLowThresholdInterrupt(LOW_INTERRUPT_X_ENABLE ,LOW_INTERRUPT_Y_ENABLE ,LOW_INTERRUPT_Z_ENABLE ,1);
}

void loop() 
{
  struct bmm150MagData magData;
  uint8_t interruptState = bmm150.getLowThresholdInterrputState();
  switch(interruptState){
    case 1:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
      Serial.println();
      break;
    case 2:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
      Serial.println();
      break;
    case 3:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
      Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
      Serial.println();
      break;
    case 4:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
      Serial.println();
      break;
    case 5:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
      Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
      Serial.println();
      break;
    case 6:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
      Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
      Serial.println();
      break;
    case 7:
      bmm150.getGeomagneticData(&magData);
      Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
      Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
      Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
      Serial.println();
      break;
    default:
      break;
  }
  delay(1000);
}