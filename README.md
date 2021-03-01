# DFRobot_bmm150
DFRobot's bmm150

## DFRobot_bmm150 Library for Arduino
---------------------------------------------------------
Arduino library is provided for wireless communication

## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_bmm150.
Download the zip file first to use this library and uncompress it to a folder named DFRobot_bmm150.

## Methods

```C++

  /**
   * @brief Initialize spI and CS pins or initialize i2c
   * @return true begin success false begin error
   */
  bool begin(void);

  /**
   * @brief get bmm150 chip id
   *
   * @return chip id
   */
  uint8_t DFRobot_bmm150::getChipID(void)

  /**
   * @brief soft reset
   */
  void DFRobot_bmm150::softReset(void)

  /**
   * @brief set bmm150 self test
   *
   * @param self mode:
   *      BMM150_SELF_TEST_NORMAL
   *      BMM150_SELF_TEST_ADVANCED
   * @retval  0       BMM150_OK
   * @retval  1       BMM150_W_NORMAL_SELF_TEST_YZ_FAIL
   * @retval  2       BMM150_W_NORMAL_SELF_TEST_XZ_FAIL
   * @retval  3       BMM150_W_NORMAL_SELF_TEST_Z_FAIL
   * @retval  4       BMM150_W_NORMAL_SELF_TEST_XY_FAIL
   * @retval  5       BMM150_W_NORMAL_SELF_TEST_Y_FAIL
   * @retval  6       BMM150_W_NORMAL_SELF_TEST_X_FAIL
   * @retval  7       BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL
   * @retval  8       BMM150_W_ADV_SELF_TEST_FAIL
   */
  int8_t DFRobot_bmm150::selfTest(uint8_t testMode)

  /**
   * @brief set bmm150 operation mode
   *
   * @param op mode
   *      BMM150_POWERMODE_NORMAL
   *      BMM150_POWERMODE_FORCED  (After a single measurement, return to sleep mode)
   *      BMM150_POWERMODE_SLEEP
   *      BMM150_POWERMODE_SUSPEND
   */
  void DFRobot_bmm150::setOperationMode(uint8_t opMode)

  /**
   * @brief get bmm150 operation mode
   *
   * @return op mode
   *      BMM150_POWERMODE_NORMAL
   *      BMM150_POWERMODE_FORCED
   *      BMM150_POWERMODE_SLEEP
   *      BMM150_POWERMODE_SUSPEND
   */
  uint8_t DFRobot_bmm150::getOperationMode(void)

  /**
   * @brief set bmm150 operation mode
   *
   * @param preset mode
   *      BMM150_PRESETMODE_LOWPOWER
   *      BMM150_PRESETMODE_REGULAR
   *      BMM150_PRESETMODE_HIGHACCURACY
   *      BMM150_PRESETMODE_ENHANCED
   */
  void DFRobot_bmm150::setPresetMode(uint8_t presetMode)

  /**
   * @brief read bmm150 x y z data
   *
   * @param struct bmm150message data 
   */
  void DFRobot_bmm150::getGeomagneticData(struct bmm150MagData *magData)

  /**
   * @brief set rate
   *
   * @param rate
   *      BMM150_DATA_RATE_10HZ   (default rate)
   *      BMM150_DATA_RATE_02HZ
   *      BMM150_DATA_RATE_06HZ
   *      BMM150_DATA_RATE_08HZ
   *      BMM150_DATA_RATE_15HZ
   *      BMM150_DATA_RATE_20HZ
   *      BMM150_DATA_RATE_25HZ
   *      BMM150_DATA_RATE_30HZ
   */
  void DFRobot_bmm150::setRate(uint8_t rate)

  /**
   * @brief get rate
   *
   * @return rate
   *      BMM150_DATA_RATE_10HZ
   *      BMM150_DATA_RATE_02HZ
   *      BMM150_DATA_RATE_06HZ
   *      BMM150_DATA_RATE_08HZ
   *      BMM150_DATA_RATE_15HZ
   *      BMM150_DATA_RATE_20HZ
   *      BMM150_DATA_RATE_25HZ
   *      BMM150_DATA_RATE_30HZ
   */
  uint8_t DFRobot_bmm150::getRate(void)

  /**
   * @brief Enable or disable the data readly mode pin, configure the polarity of the data ready mode pin
   *
   * @param modes Enable or disable the pin :
   *      enable   : ENABLE_DRDY
   *      disable  : DISABLE_DRDY  (default mode)
   *  @param polarity  Active level
   *      high     : POKARITY_HIGH  (default active high level )
   *      low      : POKARITY_LOW
   */
  void DFRobot_bmm150::setDataReadlyInterruptPin(uint8_t modes ,uint8_t polarity)

  /**
   * @brief Get data ready status 
   *
   * @return status  data readly status
   *      1 is  data is readly
   *      0 is  data is not readly
   */
  uint8_t DFRobot_bmm150::getDataReadlyState(void)

  /**
   * @brief Set the measurement of the xyz axis channel 
   *
   * @param channelX  channel x selection:
   *      enable x  : MEASUREMENT_X_ENABLE  (Default x-axis channel enabled)
   *      disable x : MEASUREMENT_X_DISABLE
   * @param channelY  channel y selection:
   *      enable y  : MEASUREMENT_Y_ENABLE  (Default y-axis channel enabled)
   *      disable y : MEASUREMENT_Y_DISABLE
   * @param channelY  channel y selection:
   *      enable z  : MEASUREMENT_Z_ENABLE  (Default z-axis channel enabled)
   *      disable z : MEASUREMENT_Z_DISABLE
   */
  void DFRobot_bmm150::setMeasurementXYZ(uint8_t channelX ,uint8_t channelY, uint8_t channelZ)

  /**
   * @brief Get the measurement status of the xyz axis channel 
   *
   * @param channel  channel ? selection:
   *      channel x : CHANNEL_X
   *      channel y : CHANNEL_Y
   *      channel z : CHANNEL_Z
   * @return status  interrupt status
   *      1 is  enable measurement
   *      0 is  disable measurement
   */
  uint8_t DFRobot_bmm150::getMeasurementStateXYZ(uint8_t channel)

  /**
   * @brief Set data overrun interrupt 
   *
   * @param modes  enable or disable overrun
   *      enable  : DATA_OVERRUN_ENABLE
   *      disable : DATA_OVERRUN_DISABLE  (dafault disable overrun)
   */
  void DFRobot_bmm150::setDataOverrun(uint8_t modes)

  /**
   * @brief get data overrun interrupt state
   *
   * @return status  data overrun status
   *      1 is  data overrun
   *      0 is  not data overrun
   */
  uint8_t DFRobot_bmm150::getDataOverrunState(void)

  /**
   * @brief Set data overflow interrupt pin
   *
   * @param modes  enable or disable overflow pin
   *      enable  : OVERFLOW_INT_ENABLE
   *      disable : OVERFLOW_INT_DISABLE  (dafault disable overflow)
   */
  void DFRobot_bmm150::setOverflowPin(uint8_t modes)

  /**
   * @brief get data overflow interrupt state
   *
   * @return status  data overflow status
   *      1 is  overflow
   *      0 is  not overflow
   */
  uint8_t DFRobot_bmm150::getOverflowState(void)

  /**
   * @brief Set the channel and value of the high threshold interrupt 
   *
   * @param channelX  channel x selection:
   *      enable x  : LOW_INTERRUPT_X_ENABLE
   *      disable x : LOW_INTERRUPT_X_DISABLE
   * @param channelY  channel y selection:
   *      enable y  : LOW_INTERRUPT_Y_ENABLE
   *      disable y : LOW_INTERRUPT_Y_DISABLE
   * @param channelY  channel y selection:
   *      enable z  : LOW_INTERRUPT_X_ENABLE
   *      disable z : LOW_INTERRUPT_X_DISABLE
   * @param  lowThreshold is low threshold
   */
  void DFRobot_bmm150::setLowThresholdInterrupt(uint8_t channelX ,uint8_t channelY, uint8_t channelZ , int8_t lowThreshold)

  /**
   * @brief Get the channel low threshold Interrupt status 
   *
   * @return status  interrupt status
   *      1-7 is  interrupt
   *      0 is  no interrupt
   */
  uint8_t DFRobot_bmm150::getLowThresholdInterrputState(void)

  /**
   * @brief Set the channel and value of the high threshold interrupt 
   *
   * @param channelX  channel x selection:
   *      enable x  : HIGH_INTERRUPT_X_ENABLE
   *      disable x : HIGH_INTERRUPT_X_DISABLE
   * @param channelY  channel y selection:
   *      enable y  : HIGH_INTERRUPT_Y_ENABLE
   *      disable y : HIGH_INTERRUPT_Y_DISABLE
   * @param channelY  channel y selection:
   *      enable z  : HIGH_INTERRUPT_X_ENABLE
   *      disable z : HIGH_INTERRUPT_X_DISABLE
   * @param  highThreshold is high threshold
   */
  void DFRobot_bmm150::setHighThresholdInterrupt(uint8_t channelX ,uint8_t channelY, uint8_t channelZ , int8_t highThreshold)

  /**
   * @brief Get the channel ? high threshold Interrupt status 
   *
   * @return status  interrupt status
   *      1-7 is  interrupt
   *      0 is  no interrupt
   */
  uint8_t DFRobot_bmm150::getHighThresholdInterrputState(void)

  /**
   * @brief Enable or disable the interrupt pin, configure the polarity of the interrupt pin
   *
   * @param modes Enable or disable the pin :
   *      enable   : ENABLE_INTERRUPT_PIN
   *      disable  : DISABLE_INTERRUPT_PIN  (default mode)
   * @param polarity  Active level
   *      high     : POKARITY_HIGH  (default active high level )
   *      low      : POKARITY_LOW
   */
  void DFRobot_bmm150::setInterrputPin(uint8_t modes ,uint8_t polarity)

  /**
   * @brief Set interrupt latch mode 
   *      After the latch is enabled, only the data in the 0x4A register will be refreshed.
   *      No latch, data is refreshed in real time.
   *
   * @param modes  Latched or not latched 
   *      latch    : INTERRUPUT_LATCH_ENABLE  (dafault interrupt latch)
   *      no latch : INTERRUPUT_LATCH_DISABLE
   */
  void DFRobot_bmm150::setInterruputLatch(uint8_t modes)
```
## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno  |      √       |             |            | 
Leonardo  |      √       |             |            | 
Meag2560 |      √       |             |            | 
ESP32 |      √       |             |            | 

## History

- date 2020-6-8
- version V1.0

## Credits

Written by ZhixinLiu(zhixin.liu@dfrobot.com), 2020. (Welcome to our [website](https://www.dfrobot.com/))