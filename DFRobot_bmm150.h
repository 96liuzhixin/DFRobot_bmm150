/*!
 * @file  DFRobot_bmm150.h
 * @brief Defines the infrastructure of the DFRobot_bmm150 class
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version     V1.0
 * @date        2020-6-8
 * @url         https://github.com/DFRobot/DFRobot_bmm150
 */
#ifndef __DFRobot_bmm150_H__
#define __DFRobot_bmm150_H__
#include "bmm150_defs.h"
#include "Arduino.h"
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

class DFRobot_bmm150{
public:
  DFRobot_bmm150();
  ~DFRobot_bmm150();
  bool    sensorInit(void);
  uint8_t getChipID(void);
  void    softReset(void);
  int8_t  selfTest(uint8_t testMode);
  void    setOperationMode(uint8_t opMode);
  uint8_t getOperationMode(void);
  void    setPresetMode(uint8_t presetMode);
  void    getGeomagneticData(struct bmm150MagData *magData);
  void    setRate(uint8_t rate);
  uint8_t getRate(void);
  void    setDataReadlyInterruptPin(uint8_t modes ,uint8_t polarity);
  uint8_t getDataReadlyState(void);
  void    setMeasurementXYZ(uint8_t channelX ,uint8_t channelY, uint8_t channelZ);
  uint8_t getMeasurementStateXYZ(uint8_t channel);
  void    setDataOverrun(uint8_t modes);
  uint8_t getDataOverrunState(void);
  void    setOverflowPin(uint8_t modes);
  uint8_t getOverflowState(void);
  void    setLowThresholdInterrupt(uint8_t channelX ,uint8_t channelY, uint8_t channelZ , int8_t lowThreshold);
  uint8_t getLowThresholdInterrputState(void);
  void    setHighThresholdInterrupt(uint8_t channelX ,uint8_t channelY, uint8_t channelZ , int8_t highThreshold);
  uint8_t getHighThresholdInterrputState(void);
  void    setInterrputPin(uint8_t modes ,uint8_t polarity=POKARITY_HIGH);
  void    setInterruputLatch(uint8_t modes);
  struct  bmm150TrimRegister _trimData;
protected:
  void    getTrimData(void);
  void    setReg(uint8_t regAddr ,uint8_t *regData ,uint8_t len);
  void    getReg(uint8_t regAddr ,uint8_t *regData ,uint8_t len);
  void    setOdrXYZ(const struct bmm150Settings *settings);
  void    setOdr(const struct bmm150Settings *settings);
  void    setXYRep(const struct bmm150Settings *settings);
  void    setZRep(const struct bmm150Settings *settings);
  void    setPowerControlBit(uint8_t powerBit);
  void    writeOpMode(uint8_t opMode);
  void    suspendToSleepMode(void);
  void    setControlMeasurementXYZ(const struct bmm150Settings *settings);
  int16_t compensateX(int16_t magDataX, uint16_t dataRhall);
  int16_t compensateY(int16_t magDataY, uint16_t dataRhall);
  int16_t compensateZ(int16_t magDataZ, uint16_t dataRhall);
  int8_t  normalSelfTest(void);
  uint8_t enableNormalSelfTest(void);
  int8_t  validatNormalSelfTest(void);
  int8_t  advSelfTest(void);
  void    advSelfTestSet(void);
  void    advSelfTestMeasurement(uint8_t selfTestCurrent, int16_t *dataZ);
  int8_t  validateAdvSelfTest(int16_t postiveDataZ, int16_t negativeDataZ);
  void    setAdvSelfTestCurrent(uint8_t selfTestCurrent);
  virtual void writeData(uint8_t Reg ,uint8_t *Data ,uint8_t len) = 0;
  virtual int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len) = 0;
private:
};

class DFRobot_bmm150_I2C:public DFRobot_bmm150{
public:
  DFRobot_bmm150_I2C(TwoWire *pWire=&Wire ,uint8_t addr = 0x75);
  bool begin(void);
protected:
  virtual void     writeData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  virtual int16_t  readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
private:
  TwoWire *_pWire;
  uint8_t _I2C_addr;
};


class DFRobot_bmm150_SPI:public DFRobot_bmm150{
public:
  bool begin(void);
  DFRobot_bmm150_SPI(SPIClass *spi=&SPI, uint8_t csPin=10);
protected:
  virtual void     writeData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  virtual int16_t  readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);

private:
  SPIClass *_pSpi;
  uint8_t _csPin;
};

#endif