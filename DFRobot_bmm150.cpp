/*!
 * @file DFRobot_BMM150.cpp
 * @brief Define the infrastructure of the DFRobot_BMM150 class and the implementation of the underlying methods
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version     V1.0
 * @date        2020-6-8
 * @url         https://github.com/DFRobot/DFRobot_BMM150
 */
#include "DFRobot_bmm150.h"

DFRobot_BMM150::DFRobot_BMM150()
{
}
DFRobot_BMM150::~DFRobot_BMM150()
{
}

/********************** Global function definitions ************************/

/*!
 *  @brief Initialize bmm150 
 *
 *  @return status  init status
 *      true is   init success
 *      false is  init error
 */
bool DFRobot_BMM150::sensorInit()
{
  uint8_t chipID;
  int8_t  rslt    = 0;
  /* Power up the sensor from suspend to sleep mode */
  setPowerControlBit(BMM150_POWER_CNTRL_DISABLE);
  setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
  getReg(BMM150_REG_CHIP_ID, &chipID, 1);

  if(chipID == BMM150_CHIP_ID){
    getTrimData();
    return true;
  }else{
    return false;
  }
}

/*!
 *  @brief get bmm150 chip id
 *
 *  @return chip id
 */
uint8_t DFRobot_BMM150::getChipID(void)
{
  uint8_t chipID = 0;
  getReg(BMM150_REG_CHIP_ID, &chipID, 1);
  return chipID;
}
/*!
 *  @brief Set register data
 *
 *  @param regAddr register address
 *  @param regData register data
 *  @param len register data length
 */
void DFRobot_BMM150::setReg(uint8_t regAddr ,uint8_t *regData, uint8_t len)
{
  delay(3);
  writeData(regAddr ,regData ,len);
}

/*!
 *  @brief Set register data
 *
 *  @param regAddr register address
 *  @param regData register data
 *  @param len register data length
 */
void DFRobot_BMM150::getReg(uint8_t regAddr, uint8_t *regData, uint8_t len)
{
  delay(3);
  readData(regAddr ,regData ,len);
}

/*!
 *  @brief soft reset
 */
void DFRobot_BMM150::softReset(void)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_POWER_CONTROL, &regData, 1);
  regData = regData | BMM150_SET_SOFT_RESET;
  setReg(BMM150_REG_POWER_CONTROL, &regData, 1);
  delay(1000);
}

/*!
 *  @brief set bmm150 operation mode
 *
 *  @param op mode
 *      BMM150_POWERMODE_NORMAL
 *      BMM150_POWERMODE_FORCED  (After a single measurement, return to sleep mode)
 *      BMM150_POWERMODE_SLEEP
 *      BMM150_POWERMODE_SUSPEND
 */
void DFRobot_BMM150::setOperationMode(uint8_t opMode)
{
  switch (opMode)
  {
    case BMM150_POWERMODE_NORMAL:
      suspendToSleepMode();
      writeOpMode(opMode);
      break;
    case BMM150_POWERMODE_FORCED:
      suspendToSleepMode();
      writeOpMode(opMode);
      break;
    case BMM150_POWERMODE_SLEEP:
      suspendToSleepMode();
      writeOpMode(opMode);
      break;
    case BMM150_POWERMODE_SUSPEND:
      setPowerControlBit(BMM150_POWER_CNTRL_DISABLE);
      break;
    default:
      break;
  }
}

/*!
 *  @brief set rate
 *
 *  @param rate
 *      BMM150_DATA_RATE_10HZ   (default rate)
 *      BMM150_DATA_RATE_02HZ
 *      BMM150_DATA_RATE_06HZ
 *      BMM150_DATA_RATE_08HZ
 *      BMM150_DATA_RATE_15HZ
 *      BMM150_DATA_RATE_20HZ
 *      BMM150_DATA_RATE_25HZ
 *      BMM150_DATA_RATE_30HZ
 */
void DFRobot_BMM150::setRate(uint8_t rate)
{
  uint8_t regData;
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  
  switch(rate){
    case BMM150_DATA_RATE_10HZ:
    case BMM150_DATA_RATE_02HZ:
    case BMM150_DATA_RATE_06HZ:
    case BMM150_DATA_RATE_08HZ:
    case BMM150_DATA_RATE_15HZ:
    case BMM150_DATA_RATE_20HZ:
    case BMM150_DATA_RATE_25HZ:
    case BMM150_DATA_RATE_30HZ:
      regData = BMM150_SET_BITS(regData, BMM150_ODR, rate);
    break;
    default:
      break;
  }
  setReg(BMM150_REG_OP_MODE, &regData, 1);
}

/*!
 *  @brief get rate
 *
 *  @return rate
 *      BMM150_DATA_RATE_10HZ
 *      BMM150_DATA_RATE_02HZ
 *      BMM150_DATA_RATE_06HZ
 *      BMM150_DATA_RATE_08HZ
 *      BMM150_DATA_RATE_15HZ
 *      BMM150_DATA_RATE_20HZ
 *      BMM150_DATA_RATE_25HZ
 *      BMM150_DATA_RATE_30HZ
 */
uint8_t DFRobot_BMM150::getRate(void)
{
  uint8_t regData;
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  return ((regData&0x38)>>3);
}

/*!
 *  @brief get bmm150 operation mode
 *
 *  @return op mode
 *      BMM150_POWERMODE_NORMAL
 *      BMM150_POWERMODE_FORCED
 *      BMM150_POWERMODE_SLEEP
 *      BMM150_POWERMODE_SUSPEND
 */
uint8_t DFRobot_BMM150::getOperationMode(void)
{
  uint8_t regData = 0;
  uint8_t powerData = 0;
  getReg(BMM150_REG_POWER_CONTROL ,&powerData ,1);
  if(powerData == 0)
    return BMM150_POWERMODE_SUSPEND;
  getReg(BMM150_REG_OP_MODE ,&regData ,1);
  return ((regData&0x06)>>1);
}

/*!
 *  @brief set bmm150 operation mode
 *
 *  @param preset mode
 *      BMM150_PRESETMODE_LOWPOWER
 *      BMM150_PRESETMODE_REGULAR
 *      BMM150_PRESETMODE_HIGHACCURACY
 *      BMM150_PRESETMODE_ENHANCED
 */
void DFRobot_BMM150::setPresetMode(uint8_t presetMode)
{
  struct bmm150Settings settings;
  switch (presetMode){
    case BMM150_PRESETMODE_LOWPOWER:
      settings.dataRate = BMM150_DATA_RATE_10HZ;
      settings.xyRep = BMM150_REPXY_LOWPOWER;
      settings.zRep = BMM150_REPZ_LOWPOWER;
      setOdrXYZ(&settings);
      break;
    case BMM150_PRESETMODE_REGULAR:
      settings.dataRate = BMM150_DATA_RATE_10HZ;
      settings.xyRep = BMM150_REPXY_REGULAR;
      settings.zRep = BMM150_REPZ_REGULAR;
      setOdrXYZ(&settings);
      break;
    case BMM150_PRESETMODE_HIGHACCURACY:
      settings.dataRate = BMM150_DATA_RATE_20HZ;
      settings.xyRep = BMM150_REPXY_HIGHACCURACY;
      settings.zRep = BMM150_REPZ_HIGHACCURACY;
      setOdrXYZ(&settings);
      break;
    case BMM150_PRESETMODE_ENHANCED:
      settings.dataRate = BMM150_DATA_RATE_10HZ;
      settings.xyRep = BMM150_REPXY_ENHANCED;
      settings.zRep = BMM150_REPZ_ENHANCED;
      setOdrXYZ(&settings);
      break;
    default:
      break;
  }
}

void DFRobot_BMM150::setOdrXYZ(const struct bmm150Settings *settings)
{
  setOdr(settings);
  setXYRep(settings);
  setZRep(settings);
}

void DFRobot_BMM150::setXYRep(const struct bmm150Settings *settings)
{
  uint8_t repXY;
  /* Set the xy repetition */
  repXY = settings->xyRep;
  setReg(BMM150_REG_REP_XY, &repXY, 1);
}

void DFRobot_BMM150::setOdr(const struct bmm150Settings *settings)
{
  uint8_t regData;
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  regData = BMM150_SET_BITS(regData, BMM150_ODR, settings->dataRate);
  setReg(BMM150_REG_OP_MODE, &regData, 1);
}

void DFRobot_BMM150::setZRep(const struct bmm150Settings *settings)
{
  uint8_t repZ;
  repZ = settings->zRep;
  setReg(BMM150_REG_REP_Z, &repZ, 1);
}

/*!
 *  @brief read bmm150 x y z data
 *
 *  @param struct bmm150message data 
 */
void DFRobot_BMM150::getGeomagneticData(struct bmm150MagData *magData)
{
  int16_t msbData;
  uint8_t regData[BMM150_LEN_XYZR_DATA] = { 0 };
  struct bmm150RawMagData rawMagData;

  // Read the mag data registers
  getReg(BMM150_REG_DATA_X_LSB, regData, BMM150_LEN_XYZR_DATA);

  regData[0] = BMM150_GET_BITS(regData[0], BMM150_DATA_X);
  msbData = ((int16_t)((int8_t)regData[1])) * 32;
  rawMagData.rawDataX = (int16_t)(msbData | regData[0]);

  regData[2] = BMM150_GET_BITS(regData[2], BMM150_DATA_Y);
  msbData = ((int16_t)((int8_t)regData[3])) * 32;
  rawMagData.rawDataY = (int16_t)(msbData | regData[2]);

  regData[4] = BMM150_GET_BITS(regData[4], BMM150_DATA_Z);
  msbData = ((int16_t)((int8_t)regData[5])) * 128;
  rawMagData.rawDataZ = (int16_t)(msbData | regData[4]);
  
  regData[6] = BMM150_GET_BITS(regData[6], BMM150_DATA_RHALL);
  rawMagData.rawDataR = (uint16_t)(((uint16_t)regData[7] << 6) | regData[6]);
  
  magData->x = compensateX(rawMagData.rawDataX, rawMagData.rawDataR);
  magData->y = compensateY(rawMagData.rawDataY, rawMagData.rawDataR);
  magData->z = compensateZ(rawMagData.rawDataZ, rawMagData.rawDataR);

}

/*!
 *  @brief set bmm150 self test
 *
 *  @param self mode:
 *      BMM150_SELF_TEST_NORMAL
 *      BMM150_SELF_TEST_ADVANCED
 *  @retval  0       BMM150_OK
 *  @retval  1       BMM150_W_NORMAL_SELF_TEST_YZ_FAIL
 *  @retval  2       BMM150_W_NORMAL_SELF_TEST_XZ_FAIL
 *  @retval  3       BMM150_W_NORMAL_SELF_TEST_Z_FAIL
 *  @retval  4       BMM150_W_NORMAL_SELF_TEST_XY_FAIL
 *  @retval  5       BMM150_W_NORMAL_SELF_TEST_Y_FAIL
 *  @retval  6       BMM150_W_NORMAL_SELF_TEST_X_FAIL
 *  @retval  7       BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL
 *  @retval  8       BMM150_W_ADV_SELF_TEST_FAIL
 */
int8_t DFRobot_BMM150::selfTest(uint8_t testMode)
{
  int8_t rslt;
  struct bmm150Settings settings;

  switch (testMode){
    case BMM150_SELF_TEST_NORMAL:
      setOperationMode(BMM150_POWERMODE_SLEEP);
      rslt = normalSelfTest();
      break;
    case BMM150_SELF_TEST_ADVANCED:
      rslt = advSelfTest();
      softReset();
      break;
    default:
      rslt = BMM150_E_INVALID_CONFIG;
      break;
    }
    return rslt;
}

/*!
 *  @brief Set the power control bit of BMM150
 *
 *  @param power ctrl bit:
 *      enable  : BMM150_POWER_CNTRL_ENABLE
 *      disable : BMM150_POWER_CNTRL_DISABLE  (default disable)
 */
void DFRobot_BMM150::setPowerControlBit(uint8_t powerBit)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_POWER_CONTROL, &regData, 1);
  regData = BMM150_SET_BITS_POS_0(regData, BMM150_PWR_CNTRL, powerBit);
  setReg(BMM150_REG_POWER_CONTROL, &regData, 1);
}

/*!
 *  @brief read trim data
 */
void DFRobot_BMM150::getTrimData(void)
{
  uint8_t trimX1Y1[2] = { 0 };
  uint8_t trimXYXData[4] = { 0 };
  uint8_t trimXY1XY2[10] = { 0 };
  uint16_t tempMsb = 0;

  /* Trim register value is read */
  getReg(BMM150_DIG_X1, trimX1Y1, 2);
  getReg(BMM150_DIG_Z4_LSB, trimXYXData, 4);
  getReg(BMM150_DIG_Z2_LSB, trimXY1XY2, 10);

  // Trim data which is read is updated in the device structure
  _trimData.digX1 = (int8_t)trimX1Y1[0];
  _trimData.digY1 = (int8_t)trimX1Y1[1];
  _trimData.digX2 = (int8_t)trimXYXData[2];
  _trimData.digY2 = (int8_t)trimXYXData[3];
  tempMsb = ((uint16_t)trimXY1XY2[3]) << 8;
  _trimData.digZ1 = (uint16_t)(tempMsb | trimXY1XY2[2]);
  tempMsb = ((uint16_t)trimXY1XY2[1]) << 8;
  _trimData.digZ2 = (int16_t)(tempMsb | trimXY1XY2[0]);
  tempMsb = ((uint16_t)trimXY1XY2[7]) << 8;
  _trimData.digZ3 = (int16_t)(tempMsb | trimXY1XY2[6]);
  tempMsb = ((uint16_t)trimXYXData[1]) << 8;
  _trimData.digZ4 = (int16_t)(tempMsb | trimXYXData[0]);
  _trimData.digXY1 = trimXY1XY2[9];
  _trimData.digXY2 = (int8_t)trimXY1XY2[8];
  tempMsb = ((uint16_t)(trimXY1XY2[5] & 0x7F)) << 8;
  _trimData.digXYZ1 = (uint16_t)(tempMsb | trimXY1XY2[4]);
}


void DFRobot_BMM150::writeOpMode(uint8_t opMode)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  regData = BMM150_SET_BITS(regData, BMM150_OP_MODE, opMode);
  setReg(BMM150_REG_OP_MODE, &regData, 1);
}

void DFRobot_BMM150::suspendToSleepMode(void)
{
  setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
}



void DFRobot_BMM150::setControlMeasurementXYZ(const struct bmm150Settings *settings)
{
  uint8_t regData;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  regData = BMM150_SET_BITS(regData, BMM150_CONTROL_MEASURE, settings->xyzAxesControl);
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}


/*!
 *  @brief compensateX 
 * 
 *  @param magDataX   x axis data
 *  @param dataRhall   compensate data
 *  @return Compensated X-axis data
 */
int16_t DFRobot_BMM150::compensateX(int16_t magDataX, uint16_t dataRhall)
{
  int16_t retval;
  uint16_t processCompX0 = 0;
  int32_t processCompX1;
  uint16_t processCompX2;
  int32_t processCompX3;
  int32_t processCompX4;
  int32_t processCompX5;
  int32_t processCompX6;
  int32_t processCompX7;
  int32_t processCompX8;
  int32_t processCompX9;
  int32_t processCompX10;

  /* Overflow condition check */
  if (magDataX != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP){
    if (dataRhall != 0){
      /* Availability of valid data */
      processCompX0 = dataRhall;
    }else if (_trimData.digXYZ1 != 0){
      processCompX0 = _trimData.digXYZ1;
    }else{
      processCompX0 = 0;
    }
    if (processCompX0 != 0){
      /* Processing compensation equations */
      processCompX1 = ((int32_t)_trimData.digXYZ1) * 16384;
      processCompX2 = ((uint16_t)(processCompX1 / processCompX0)) - ((uint16_t)0x4000);
      retval = ((int16_t)processCompX2);
      processCompX3 = (((int32_t)retval) * ((int32_t)retval));
      processCompX4 = (((int32_t)_trimData.digXY2) * (processCompX3 / 128));
      processCompX5 = (int32_t)(((int16_t)_trimData.digXY1) * 128);
      processCompX6 = ((int32_t)retval) * processCompX5;
      processCompX7 = (((processCompX4 + processCompX6) / 512) + ((int32_t)0x100000));
      processCompX8 = ((int32_t)(((int16_t)_trimData.digX2) + ((int16_t)0xA0)));
      processCompX9 = ((processCompX7 * processCompX8) / 4096);
      processCompX10 = ((int32_t)magDataX) * processCompX9;
      retval = ((int16_t)(processCompX10 / 8192));
      retval = (retval + (((int16_t)_trimData.digX1) * 8)) / 16;
    }else{
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  }else{
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }
  return retval;
}

/*!
 *  @brief compensateY
 * 
 *  @param magDataY   y axis data
 *  @param dataRhall   compensate data
 *  @return Compensated y-axis data
 */
int16_t DFRobot_BMM150::compensateY(int16_t magDataY, uint16_t dataRhall)
{
  int16_t retval;
  uint16_t processCompY0 = 0;
  int32_t processCompY1;
  uint16_t processCompY2;
  int32_t processCompY3;
  int32_t processCompY4;
  int32_t processCompY5;
  int32_t processCompY6;
  int32_t processCompY7;
  int32_t processCompY8;
  int32_t processCompY9;

  /* Overflow condition check */
  if (magDataY != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP){
    if (dataRhall != 0){
      /* Availability of valid data */
      processCompY0 = dataRhall;
    }else if (_trimData.digXYZ1 != 0){
      processCompY0 = _trimData.digXYZ1;
    }else{
      processCompY0 = 0;
    }

    if (processCompY0 != 0){
      /* Processing compensation equations */
      processCompY1 = (((int32_t)_trimData.digXYZ1) * 16384) / processCompY0;
      processCompY2 = ((uint16_t)processCompY1) - ((uint16_t)0x4000);
      retval = ((int16_t)processCompY2);
      processCompY3 = ((int32_t) retval) * ((int32_t)retval);
      processCompY4 = ((int32_t)_trimData.digXY2) * (processCompY3 / 128);
      processCompY5 = ((int32_t)(((int16_t)_trimData.digXY1) * 128));
      processCompY6 = ((processCompY4 + (((int32_t)retval) * processCompY5)) / 512);
      processCompY7 = ((int32_t)(((int16_t)_trimData.digY2) + ((int16_t)0xA0)));
      processCompY8 = (((processCompY6 + ((int32_t)0x100000)) * processCompY7) / 4096);
      processCompY9 = (((int32_t)magDataY) * processCompY8);
      retval = (int16_t)(processCompY9 / 8192);
      retval = (retval + (((int16_t)_trimData.digY1) * 8)) / 16;
    }else{
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  }else{
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }
    return retval;
}

/*!
 *  @brief compensateZ
 * 
 *  @param magDataZ   z axis data
 *  @param dataRhall   compensate data
 *  @return Compensated z-axis data
 */
int16_t DFRobot_BMM150::compensateZ(int16_t magDataZ, uint16_t dataRhall)
{
  int32_t retval;
  int16_t processCompZ0;
  int32_t processCompZ1;
  int32_t processCompZ2;
  int32_t processCompZ3;
  int16_t processCompZ4;

  if (magDataZ != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL){
    if ((_trimData.digZ2 != 0) && (_trimData.digZ1 != 0) && (dataRhall != 0) &&(_trimData.digXYZ1 != 0)){
      /*Processing compensation equations */
      processCompZ0 = ((int16_t)dataRhall) - ((int16_t) _trimData.digXYZ1);
      processCompZ1 = (((int32_t)_trimData.digZ3) * ((int32_t)(processCompZ0))) / 4;
      processCompZ2 = (((int32_t)(magDataZ - _trimData.digZ4)) * 32768);
      processCompZ3 = ((int32_t)_trimData.digZ1) * (((int16_t)dataRhall) * 2);
      processCompZ4 = (int16_t)((processCompZ3 + (32768)) / 65536);
      retval = ((processCompZ2 - processCompZ1) / (_trimData.digZ2 + processCompZ4));

      /* Saturate result to +/- 2 micro-tesla */
      if (retval > BMM150_POSITIVE_SATURATION_Z){
        retval = BMM150_POSITIVE_SATURATION_Z;
      }else if (retval < BMM150_NEGATIVE_SATURATION_Z){
         retval = BMM150_NEGATIVE_SATURATION_Z;
      }
      /* Conversion of LSB to micro-tesla */
      retval = retval / 16;
    }else{
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  }else{
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }
  return (int16_t)retval;
}

/*!
 * @brief This internal API is used to perform the normal self test
 * of the sensor and return the self test result as return value
 */
int8_t DFRobot_BMM150::normalSelfTest(void)
{
  int8_t rslt;
  uint8_t selfTestBit;

  /* Triggers the start of normal self test */
  selfTestBit = enableNormalSelfTest();

  /* Check for self test completion status */
  if(selfTestBit == 0){
    /* Validates the self test results for all 3 axes */
    return validatNormalSelfTest();
  }
  return -1;
}

/*!
 * @brief This internal API is used to enable the normal self test by setting
 * the Self Test bit (bit0) of the 0x4C register,
 * which triggers the start of self test
 */
uint8_t DFRobot_BMM150::enableNormalSelfTest(void)
{
  uint8_t regData;
  uint8_t selfTestValue;

  /* Read the data from register 0x4C */
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  /* Set the Self Test bit(bit0) of the 0x4C register */
  selfTestValue = 1;
  regData = BMM150_SET_BITS_POS_0(regData, BMM150_SELF_TEST, selfTestValue);
  /* Write the data to 0x4C register to trigger self test */
  setReg(BMM150_REG_OP_MODE, &regData, 1);
  delay(300);
  /* Read the data from register 0x4C */
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  // Self Test bit(bit0) is stored in self_test_enable, It will be reset to zero after the self test is over
  return BMM150_GET_BITS_POS_0(regData, BMM150_SELF_TEST);
}

/*!
 * @brief This internal API is used to validate the results of normal self test
 * by using the self test status available in the bit0 of registers 0x42,0x44
 * and 0x46.
 */
int8_t DFRobot_BMM150::validatNormalSelfTest(void)
{
  int8_t rslt;
  uint8_t status;
  uint8_t self_test_rslt[5];

  /* Read the data from register 0x42 to 0x46 */
  getReg(BMM150_REG_DATA_X_LSB, self_test_rslt, BMM150_LEN_SELF_TEST);

  /* Parse and get the self test status bits */
  /* X-Self-Test (bit0) of 0x42 register is stored */
  self_test_rslt[0] = BMM150_GET_BITS_POS_0(self_test_rslt[0], BMM150_SELF_TEST);

  /* Y-Self-Test (bit0) of 0x44 register is stored */
  self_test_rslt[2] = BMM150_GET_BITS_POS_0(self_test_rslt[2], BMM150_SELF_TEST);
  
  /* Z-Self-Test (bit0) of 0x46 register is stored */
  self_test_rslt[4] = BMM150_GET_BITS_POS_0(self_test_rslt[4], BMM150_SELF_TEST);

  /* Combine the self test status and store it in the first
  * 3 bits of the status variable for processing
  */
  status = (uint8_t)((self_test_rslt[4] << 2) | (self_test_rslt[2] << 1) | self_test_rslt[0]);

  /* Validate status and store Self test result in "rslt" */
  if (status == BMM150_SELF_TEST_STATUS_SUCCESS)
  {
    /* Self test is success when all status bits are set */
    rslt = BMM150_OK;
  }else{
    if (status == BMM150_SELF_TEST_STATUS_XYZ_FAIL){
      /* Self test - all axis fail condition */
      rslt = BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL;
    }else{
      /* Self test - some axis fail condition */
      rslt = (int8_t)status;
    }
  }
  return rslt;
}

int8_t DFRobot_BMM150::advSelfTest(void)
{
  int8_t rslt;
  uint8_t selfTestCurrent;
  int16_t postiveDataZ;
  int16_t negativeDataZ;

  /* Set the desired power mode ,axes control and repetition settings */
  advSelfTestSet();

  /* Measure the Z axes data with positive self-test current */
  selfTestCurrent = BMM150_ENABLE_POSITIVE_CURRENT;
  advSelfTestMeasurement(selfTestCurrent, &postiveDataZ);
  selfTestCurrent = BMM150_ENABLE_NEGATIVE_CURRENT;
  advSelfTestMeasurement(selfTestCurrent, &negativeDataZ);

  /* Disable self-test current */
  selfTestCurrent = BMM150_DISABLE_SELF_TEST_CURRENT;
  setAdvSelfTestCurrent(selfTestCurrent);
  /* Validate the advanced self test */
  rslt = validateAdvSelfTest(postiveDataZ, negativeDataZ);
  return rslt;
}

void DFRobot_BMM150::advSelfTestSet(void)
{
  struct bmm150Settings settings;

  /* Set the power mode as sleep mode */
  setOperationMode(BMM150_POWERMODE_SLEEP);

  /* Disable XY-axis measurement */
  settings.xyzAxesControl = BMM150_DISABLE_XY_AXIS;
  setControlMeasurementXYZ(&settings);

  settings.zRep = BMM150_SELF_TEST_REP_Z;
  setZRep(&settings);
}

/*!
 * @brief This internal API is used to set the positive or negative value of
 * self-test current and obtain the corresponding magnetometer z axis data
 */
void DFRobot_BMM150::advSelfTestMeasurement(uint8_t selfTestCurrent, int16_t *dataZ)
{
  struct bmm150Settings settings;
  struct bmm150MagData magData;
  setAdvSelfTestCurrent(selfTestCurrent);
  setOperationMode(BMM150_POWERMODE_FORCED);
  getGeomagneticData(&magData);
  *dataZ = magData.z;
}

/*!
 * @brief This internal API is used to get the difference between the
 * Z axis mag data obtained by positive and negative self-test current
 * and validate whether the advanced self test is done successfully or not.
 */
int8_t DFRobot_BMM150::validateAdvSelfTest(int16_t postiveDataZ, int16_t negativeDataZ)
{
  int32_t adv_self_test_rslt;

  // Advanced self test difference between the Z axis mag data obtained by the positive and negative self-test current
  adv_self_test_rslt = postiveDataZ - negativeDataZ;

  // Advanced self test validation ,Value of adv_self_test_rslt should be in between 180-240 micro-tesla */
  if((adv_self_test_rslt > 180) && (adv_self_test_rslt < 240)){
    /* Advanced self test success */
    return BMM150_OK;
  }else{
    /* Advanced self test fail */
    return BMM150_W_ADV_SELF_TEST_FAIL;
  }
}

/*!
 * @brief This internal API is used to set the self test current value in
 * the Adv. ST bits (bit6 and bit7) of 0x4C register
 */
void DFRobot_BMM150::setAdvSelfTestCurrent(uint8_t selfTestCurrent)
{
  uint8_t regData;
  /* Read the 0x4C register */
  getReg(BMM150_REG_OP_MODE, &regData, 1);
  // Set the self test current value in the Adv. ST bits (bit6 and bit7) of 0x4c register
  regData = BMM150_SET_BITS(regData, BMM150_ADV_SELF_TEST, selfTestCurrent);
  setReg(BMM150_REG_OP_MODE, &regData, 1);
}

/*!
 *  @brief Enable or disable the data readly mode pin, configure the polarity of the data ready mode pin
 *
 *  @param modes Enable or disable the pin :
 *      enable   : ENABLE_DRDY
 *      disable  : DISABLE_DRDY  (default mode)
 *  @param polarity  Active level
 *      high     : POKARITY_HIGH  (default active high level )
 *      low      : POKARITY_LOW
 */
void DFRobot_BMM150::setDataReadlyInterruptPin(uint8_t modes ,uint8_t polarity)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(modes == DISABLE_DRDY){
    regData = (regData&0x7F);
  }else{
    regData = (regData|0x80);
  }
  if(polarity == POKARITY_LOW){
    regData = (regData&0xFB);
  }else{
    regData = (regData|0x04);
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  getDataReadlyState();
}

/*!
 *  @brief Get data ready status 
 *
 *  @return status  data readly status
 *      1 is   data is ready
 *      0 is   data is not ready
 */
uint8_t DFRobot_BMM150::getDataReadlyState(void)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_DATA_READY_STATUS, &regData, 1);
  if(regData & 0x01){
    return 1;
  }else{
    return 0;
  }
}

/*!
 *  @brief Enable or disable the interrupt pin, configure the polarity of the interrupt pin
 *
 *  @param modes Enable or disable the pin :
 *      enable   : ENABLE_INTERRUPT_PIN
 *      disable  : DISABLE_INTERRUPT_PIN  (default mode)
 *  @param polarity  Active level
 *      high     : POKARITY_HIGH  (default active high level )
 *      low      : POKARITY_LOW
 */
void DFRobot_BMM150::setInterrputPin(uint8_t modes ,uint8_t polarity)
{
  uint8_t regData = 0;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(modes == DISABLE_INTERRUPT_PIN){
    regData = (regData&0xBF);
  }else{
    regData = (regData|0x40);
  }
  if(polarity == POKARITY_LOW){
    regData = (regData&0xFE);
  }else{
    regData = (regData|0x01);
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

/*!
 *  @brief Set the channel and value of the low threshold interrupt 
 *
 *  @param channelX  channel x selection:
 *      enable x  : LOW_INTERRUPT_X_ENABLE
 *      disable x : LOW_INTERRUPT_X_DISABLE
 *  @param channelY  channel y selection:
 *      enable y  : LOW_INTERRUPT_Y_ENABLE
 *      disable y : LOW_INTERRUPT_Y_DISABLE
 *  @param channelY  channel y selection:
 *      enable z  : LOW_INTERRUPT_Z_ENABLE
 *      disable z : LOW_INTERRUPT_Z_DISABLE
 *  @param  lowThreshold is low threshold
 */
void DFRobot_BMM150::setLowThresholdInterrupt(uint8_t channelX ,uint8_t channelY, uint8_t channelZ , int8_t lowThreshold)
{
  uint8_t regData = 0;
  uint8_t temp     = 0;
  if(lowThreshold < 0){
    temp = (uint8_t)(lowThreshold*-1) | 0x80;
  }else{
    temp = (uint8_t)lowThreshold;
  }
  setReg(BMM150_REG_LOW_THRESHOLD ,&temp, 1);
  getReg(BMM150_REG_INT_CONFIG ,&regData, 1);
  if(channelX == LOW_INTERRUPT_X_DISABLE){
    regData |= 0x01;
  }else{
    regData &= 0xFE;
  }
  if(channelY == LOW_INTERRUPT_Y_DISABLE){
    regData |= 0x02;
  }else{
    regData &= 0xFC;
  }
  if(channelZ == LOW_INTERRUPT_Z_DISABLE){
    regData |= 0x04; 
  }else{
    regData &= 0xFB;
  }
  setReg(BMM150_REG_INT_CONFIG ,&regData, 1);
  setInterrputPin(ENABLE_INTERRUPT_PIN);
  getLowThresholdInterrputState();
}


/*!
 *  @brief Get the channel low threshold Interrupt status 
 *
 *  @return status  interrupt status
 *      1-7 is  interrupt
 *      0 is  no interrupt
 */
uint8_t DFRobot_BMM150::getLowThresholdInterrputState(void)
{
  uint8_t regData = 0;
  uint8_t state = getDataReadlyState();
  if(state == 1){
    getReg(BMM150_REG_INTERRUPT_STATUS, &regData, 1);
    return regData&0x07;
  }else{
    return 0;
  }
  
}

/*!
 *  @brief Set the channel and value of the high threshold interrupt 
 *
 *  @param channelX  channel x selection:
 *      enable x  : HIGH_INTERRUPT_X_ENABLE
 *      disable x : HIGH_INTERRUPT_X_DISABLE
 *  @param channelY  channel y selection:
 *      enable y  : HIGH_INTERRUPT_Y_ENABLE
 *      disable y : HIGH_INTERRUPT_Y_DISABLE
 *  @param channelY  channel y selection:
 *      enable z  : HIGH_INTERRUPT_X_ENABLE
 *      disable z : HIGH_INTERRUPT_X_DISABLE
 *  @param  highThreshold is high threshold
 */
void DFRobot_BMM150::setHighThresholdInterrupt(uint8_t channelX ,uint8_t channelY, uint8_t channelZ , int8_t highThreshold)
{
  uint8_t regData = 0;

  uint8_t temp     = 0;
  if(highThreshold<0){
    temp = (uint8_t)(highThreshold*-1) | 0x80;
  }else{
    temp = (uint8_t)highThreshold;
  }
  setReg(BMM150_REG_HIGH_THRESHOLD ,&temp, 1);
  getReg(BMM150_REG_HIGH_THRESHOLD ,&regData, 1);
  getReg(BMM150_REG_INT_CONFIG ,&regData, 1);
  if(channelX == HIGH_INTERRUPT_X_DISABLE){
    regData |= 0x08;
  }else{
    regData &= 0xF7;
  }
  if(channelY == HIGH_INTERRUPT_Y_DISABLE){
    regData |= 0x10;
  }else{
    regData &= 0xEF;
  }
  if(channelZ == HIGH_INTERRUPT_Z_DISABLE){
    regData |= 0x20; 
  }else{
    regData &= 0xDF;
  }
  setReg(BMM150_REG_INT_CONFIG ,&regData, 1);
  getReg(BMM150_REG_INT_CONFIG ,&regData, 1);
  setInterrputPin(ENABLE_INTERRUPT_PIN);
  getHighThresholdInterrputState();
}

/*!
 *  @brief Get the channel ? high threshold Interrupt status 
 *
 *  @return status  interrupt status
 *      1-7 is  interrupt
 *      0 is  no interrupt
 */
uint8_t DFRobot_BMM150::getHighThresholdInterrputState(void)
{
  uint8_t regData = 0;
  uint8_t state = getDataReadlyState();
  if(state == 1){
    getReg(BMM150_REG_INTERRUPT_STATUS, &regData, 1);
    return (regData&0x38)>>3;
  }else{
    return 0;
  }
}

/*!
 *  @brief Set interrupt latch mode 
 *      After the latch is enabled, only the data in the 0x4A register will be refreshed.
 *      No latch, data is refreshed in real time.
 *
 *  @param modes  Latched or not latched 
 *      latch    : INTERRUPUT_LATCH_ENABLE  (dafault interrupt latch)
 *      no latch : INTERRUPUT_LATCH_DISABLE
 */
void DFRobot_BMM150::setInterruputLatch(uint8_t modes)
{
  uint8_t regData = 0;
  int8_t  rslt;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(modes == INTERRUPUT_LATCH_DISABLE){
    regData &= 0xFD;
  }else{
    regData |= 0x02;
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

/*!
 *  @brief Set the measurement of the xyz axis channel 
 *
 *  @param channelX  channel x selection:
 *      enable x  : MEASUREMENT_X_ENABLE  (Default x-axis channel enabled)
 *      disable x : MEASUREMENT_X_DISABLE
 *  @param channelY  channel y selection:
 *      enable y  : MEASUREMENT_Y_ENABLE  (Default y-axis channel enabled)
 *      disable y : MEASUREMENT_Y_DISABLE
 *  @param channelY  channel y selection:
 *      enable z  : MEASUREMENT_Z_ENABLE  (Default z-axis channel enabled)
 *      disable z : MEASUREMENT_Z_DISABLE
 */
void DFRobot_BMM150::setMeasurementXYZ(uint8_t channelX ,uint8_t channelY, uint8_t channelZ)
{
  uint8_t regData = 0;
  int8_t  rslt;
  getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
  if(channelX == MEASUREMENT_X_DISABLE){
    regData |= 0x08;
  }else{
    regData &= 0xF7;
  }
  if(channelY == MEASUREMENT_Y_DISABLE){
    regData |= 0x10;
  }else{
    regData &= 0xEF;
  }
  if(channelZ == MEASUREMENT_Z_DISABLE){
    regData |= 0x20; 
  }else{
    regData &= 0xDF;
  }
  setReg(BMM150_REG_AXES_ENABLE, &regData, 1);
}

/*!
 *  @brief Get the measurement status of the xyz axis channel 
 *
 *  @param channel  channel ? selection:
 *      channel x : CHANNEL_X
 *      channel y : CHANNEL_Y
 *      channel z : CHANNEL_Z
 *  @return status  measurement status
 *      1 is  enable measurement
 *      0 is  disable measurement
 */
uint8_t DFRobot_BMM150::getMeasurementStateXYZ(uint8_t channel)
{
  uint8_t regData = 0;

  switch(channel)
  {
    case CHANNEL_X:
      getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
      if((regData&0x08) == 0) return 1;
      break;
    case CHANNEL_Y:
      getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
      if((regData&0x10) == 0) return 1;
      break;
    case CHANNEL_Z:
      getReg(BMM150_REG_AXES_ENABLE, &regData, 1);
      if((regData&0x20) == 0) return 1;
      break;
    default:
      return 1;
      break;
  }
  return 0;
}

/*!
 *  @brief Set data overrun interrupt 
 *
 *  @param modes  enable or disable overrun
 *      enable  : DATA_OVERRUN_ENABLE
 *      disable : DATA_OVERRUN_DISABLE  (dafault disable overrun)
 */
void DFRobot_BMM150::setDataOverrun(uint8_t modes)
{
  uint8_t regData = 0;

  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(modes == DATA_OVERRUN_DISABLE){
    regData &= 0x7F;
  }else{
    regData |= 0x80;
  }
  setReg(BMM150_REG_INT_CONFIG, &regData, 1);
}

/*!
 *  @brief get data overrun interrupt state
 *
 *  @return status  data overrun status
 *      1 is  data overrun
 *      0 is  not data overrun
 */
uint8_t DFRobot_BMM150::getDataOverrunState(void)
{
  uint8_t regData = 0;
  int8_t  rslt;
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(regData|0x80){
    return 1;
  }else{
    return 0;
  }
}

/*!
 *  @brief Set data overflow interrupt pin
 *
 *  @param modes  enable or disable overflow pin
 *      enable  : OVERFLOW_INT_ENABLE
 *      disable : OVERFLOW_INT_DISABLE  (dafault disable overflow)
 */
void DFRobot_BMM150::setOverflowPin(uint8_t modes)
{
  uint8_t regData = 0;
  int8_t  rslt;
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(modes == OVERFLOW_INT_DISABLE){
    regData &= 0xBF;
  }else{
    regData |= 0x40;
  }
  setReg(BMM150_REG_INT_CONFIG, &regData, 1);
}

/*!
 *  @brief get data overflow interrupt state
 *
 *  @return status  data overflow status
 *      1 is  overflow
 *      0 is  not overflow
 */
uint8_t DFRobot_BMM150::getOverflowState(void)
{
  uint8_t regData = 0;
  int8_t  rslt;
  getReg(BMM150_REG_INT_CONFIG, &regData, 1);
  if(regData|0x40){
    return 1;
  }else{
    return 0;
  }
}


DFRobot_BMM150_I2C::DFRobot_BMM150_I2C(TwoWire *pWire, uint8_t addr)
{
  _pWire = pWire;
  this->_I2C_addr = addr;
}

bool DFRobot_BMM150_I2C::begin()
{
  _pWire->begin();
  _pWire->beginTransmission(_I2C_addr);
  if(_pWire->endTransmission() == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void DFRobot_BMM150_I2C::writeData(uint8_t Reg ,uint8_t *Data ,uint8_t len)
{
  _pWire->beginTransmission(this->_I2C_addr);
  _pWire->write(Reg);
  for(uint8_t i = 0; i < len; i++)
  {
    _pWire->write(Data[i]);
  }
  _pWire->endTransmission();
}

int16_t DFRobot_BMM150_I2C::readData(uint8_t Reg,uint8_t *Data,uint8_t len)
{
  int i=0;
  _pWire->beginTransmission(this->_I2C_addr);
  _pWire->write(Reg);
  if(_pWire->endTransmission() != 0)
  {
    return -1;
  }
  _pWire->requestFrom((uint8_t)this->_I2C_addr,(uint8_t)len);
  while (_pWire->available())
  {
    Data[i++]=_pWire->read();
  }
  return 0;
}

#if 1
DFRobot_BMM150_SPI::DFRobot_BMM150_SPI(SPIClass *pSpi, uint8_t csPin)
{
  _pSpi = pSpi;
  _csPin = csPin;
}

bool DFRobot_BMM150_SPI::begin(void)
{
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, 1);
  _pSpi->begin();
  //_pSpi->setBitOrder(LSBFIRST);
  //_pSpi->setDataMode(SPI_MODE3);
  return true;
}

void DFRobot_BMM150_SPI::writeData(uint8_t Reg ,uint8_t *Data ,uint8_t len)
{
  digitalWrite(_csPin, 0);
  _pSpi->transfer(Reg&0x7F);
  _pSpi->transfer(Data[0]);
  digitalWrite(_csPin, 1);
  for(uint8_t i = 1; i < len; i++){
    _pSpi->transfer(Data[i]);
  }
}

int16_t DFRobot_BMM150_SPI::readData(uint8_t Reg,uint8_t *Data,uint8_t len)
{
  digitalWrite(_csPin, 0);
  _pSpi->transfer(Reg|0x80);
  Data[0] = _pSpi->transfer(0xFF);
  for(uint8_t i = 1; i < len; i++){
    Data[i] = _pSpi->transfer(0xFF);
  }
  digitalWrite(_csPin, 1);
  return 0;
}
#endif