# DFRobot BMM150 concentration sensor

This RaspberryPi BMM150 sensor board can communicate with RaspberryPi via I2C or spi.<br>
The BMM150 is capable of obtaining triaxial geomagnetic data.<br>

## DFRobot BMM150 Library for RaspberryPi

Provide the Raspberry Pi library for the DFRobot_bmm150 module.

## Table of Contents

* [Summary](#summary)
* [Feature](#feature)
* [Installation](#installation)
* [Methods](#methods)
* [History](#history)
* [Credits](#credits)

## Summary

BMM150 module.

## Feature

1. This module can obtain high threshold and low threshold geomagnetic data. <br>
2. Geomagnetism on three(xyz) axes can be measured.<br>
3. This module can choose I2C or SPI communication mode.<br>


## Installation

This Sensor should work with DFRobot_BMM150 on RaspberryPi. <br>
Run the program:

```
$> python data_readly_interrupt.py
$> python high_threshold_interrupt.py
$> python low_threshold_interrupt.py
$> python self_test.py
$> python soft_reset.py
```

## Methods

```py

  def sensor_init(self):
    '''
      @brief init sensor
      @return 0  is init success
              -1 is init failed
    '''

  def get_chip_id(self):
    '''
      @brief get chip id
      @return chip id
    '''

  def soft_reset(self):
    '''
      @brief soft reset
    '''

  def self_test(self):
    '''
      @brief set bmm150 self test
      @retval
        SELF_TEST_XYZ_FALL               = 0
        SELF_TEST_YZ_FAIL                = 1
        SELF_TEST_XZ_FAIL                = 2
        SELF_TEST_Z_FAIL                 = 3
        SELF_TEST_XY_FAIL                = 4
        SELF_TEST_Y_FAIL                 = 5
        SELF_TEST_X_FAIL                 = 6
        SELF_TEST_XYZ_OK                 = 7
    '''

  def set_power_bit(self ,ctrl):
    '''
      @brief set power bit
      @param ctrl is enable/disable power
        DISABLE_POWER is disable power
        ENABLE_POWER  is enable power
    '''

  def get_power_bit(self):
    '''
      @brief get power bit
      @return power bit
        DISABLE_POWER is disable power
        ENABLE_POWER  is enable power
    '''

  def set_operation_mode(self ,modes):
    '''
      @brief set opration mode
      @param modes is operation mode
        POWERMODE_NORMAL
        POWERMODE_FORCED
        POWERMODE_SLEEP
        POWERMODE_SUSPEND
    '''

  def get_operation_mode(self):
    '''
      @brief get opration mode
      @return modes is operation mode
        POWERMODE_NORMAL
        POWERMODE_FORCED
        POWERMODE_SLEEP
        POWERMODE_SUSPEND
    '''

  def set_rate(self ,rates):
    '''
      @brief set rate
      @param rate
        RATE_10HZ        #(default rate)
        RATE_02HZ
        RATE_06HZ
        RATE_08HZ
        RATE_15HZ
        RATE_20HZ
        RATE_25HZ
        RATE_30HZ
    '''

  def get_rate(self):
    '''
      @brief get rates
      @return rates
        RATE_10HZ        #(default rate)
        RATE_02HZ
        RATE_06HZ
        RATE_08HZ
        RATE_15HZ
        RATE_20HZ
        RATE_25HZ
        RATE_30HZ
    '''

  def set_preset_mode(self ,modes):
    '''
      @brief set preset mode
      @param modes 
        PRESETMODE_LOWPOWER
        PRESETMODE_REGULAR
        PRESETMODE_HIGHACCURACY
        PRESETMODE_ENHANCED      
    '''

  def set_xy_rep(self ,modes):
    '''
      @brief set xy rep
      @param modes
        REPXY_LOWPOWER
        REPXY_REGULAR
        REPXY_ENHANCED
        REPXY_HIGHACCURACY
    '''

  def set_z_rep(self ,modes):
    '''
      @brief set z rep
      @param modes
        REPZ_LOWPOWER
        REPZ_REGULAR
        REPZ_ENHANCED
        REPZ_HIGHACCURACY
    '''

  def get_geomagnetic(self):
    '''
      @brief get geomagnetic
    '''

  def set_data_readly_interrupt_pin(self ,modes ,polarity):
    '''
      @brief Enable or disable the data readly mode pin, configure the polarity of the data ready mode pin
      @param modes Enable or disable the pin :
        enable   : ENABLE_DRDY
        disable  : DISABLE_DRDY  (default mode)
      @param polarity  Active level
        high     : POKARITY_HIGH  (default active high level )
        low      : POKARITY_LOW
    '''

  def get_data_readly_state(self):
    '''
      @brief Get data ready status 
      @return status  data readly status
        1 is   data is ready
        0 is   data is not ready
    '''

  def set_measurement_xyz(self ,channel_x ,channel_y ,channel_z):
    '''
      @brief set measurement xyz
      @param channel_x  channel x selection:
        MEASUREMENT_X_ENABLE  (Default x-axis channel enabled)
        MEASUREMENT_X_DISABLE
      @param channel_y  channel y selection:
        MEASUREMENT_Y_ENABLE  (Default y-axis channel enabled)
        MEASUREMENT_Y_DISABLE
      @param channel_z  channel z selection:
        MEASUREMENT_Z_ENABLE  (Default z-axis channel enabled)
        MEASUREMENT_Z_DISABLE
    '''

  def get_measurement_state_xyz(self ,channel):
    '''
      @brief get measurement xyz
      @param channel  channel ? selection:
        CHANNEL_X
        CHANNEL_Y
        CHANNEL_Z
      @return 
        1 is  enable measurement
        0 is  disable measurement
    '''

  def set_interrupt_pin(self ,modes ,polarity):
    '''
      @brief Enable or disable the interrupt pin, configure the polarity of the interrupt pin
      @param modes Enable or disable the pin :
        enable   : ENABLE_INTERRUPT_PIN
        disable  : DISABLE_INTERRUPT_PIN  (default mode)
      @param polarity  Active level
        high     : POKARITY_HIGH  (default active high level )
        low      : POKARITY_LOW
    '''

  def set_interruput_latch(self ,modes):
    '''
      @brief Set the channel and value of the low threshold interrupt
      @param channelX  channel x selection:
        enable x  : LOW_INTERRUPT_X_ENABLE
        disable x : LOW_INTERRUPT_X_DISABLE
      @param channelY  channel y selection:
        enable y  : LOW_INTERRUPT_Y_ENABLE
        disable y : LOW_INTERRUPT_Y_DISABLE
      @param channelZ  channel z selection:
        enable z  : LOW_INTERRUPT_Z_ENABLE
        disable z : LOW_INTERRUPT_Z_DISABLE
      @param  low_threshold is low threshold
    '''

  def set_low_threshold_interrupt(self ,channel_x ,channel_y ,channel_z ,low_threshold):
    '''
      @brief Set the channel and value of the low threshold interrupt
      @param channelX  channel x selection:
        enable x  : LOW_INTERRUPT_X_ENABLE
        disable x : LOW_INTERRUPT_X_DISABLE
      @param channelY  channel y selection:
        enable y  : LOW_INTERRUPT_Y_ENABLE
        disable y : LOW_INTERRUPT_Y_DISABLE
      @param channelZ  channel z selection:
        enable z  : LOW_INTERRUPT_Z_ENABLE
        disable z : LOW_INTERRUPT_Z_DISABLE
      @param  low_threshold is low threshold
    '''

  def get_low_threshold_interrupt_state(self):
    '''
      @brief  Get the channel low threshold Interrupt status 
      @return status  interrupt status
         1-7 is  interrupt
         0 is  no interrupt
    '''

  def set_high_threshold_interrupt(self ,channel_x ,channel_y ,channel_z ,high_threshold):
    '''
      @brief Set the channel and value of the high threshold interrupt
      @param channelX  channel x selection:
        enable x  : HIGH_INTERRUPT_X_ENABLE
        disable x : HIGH_INTERRUPT_X_DISABLE
      @param channelY  channel y selection:
        enable y  : HIGH_INTERRUPT_Y_ENABLE
        disable y : HIGH_INTERRUPT_Y_DISABLE
      @param channelZ  channel z selection:
        enable z  : HIGH_INTERRUPT_Z_ENABLE
        disable z : HIGH_INTERRUPT_Z_DISABLE
      @param  high_threshold is high threshold
    '''

  def get_high_threshold_interrupt_state(self):
    '''
      @brief  Get the channel low threshold Interrupt status 
      @return status  interrupt status
         1-7 is  interrupt
         0 is  no interrupt
    '''

```
## History

March 3, 2021 - Version 1.0 released.

## Credits

Written by ZhixinLiu(zhixin.liu@dfrobot.com), 2021. (Welcome to our website)