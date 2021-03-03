# -*- coding:utf-8 -*-
"""
  @file data_readly_interrupt.py
  @brief get geomagnetic data
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
  version  V1.0
  date  2021-03-02
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_bmm150
"""
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
from DFRobot_bmm150 import *

bmm150 = DFRobot_bmm150_I2C (0x01 ,0x13)         # bus default use I2C1 , iic address is 0x13
#bmm150 = DFRobot_bmm150_SPI (27)                # default use spi0 ,cs pin is 27 

def setup():
  while ERROR == bmm150.sensor_init():
    print "sensor init error ,please check connect"
  '''
      POWERMODE_NORMAL
      POWERMODE_FORCED
      POWERMODE_SLEEP
      POWERMODE_SUSPEND
  '''
  bmm150.set_operation_mode(POWERMODE_NORMAL)

  '''
      PRESETMODE_LOWPOWER
      PRESETMODE_REGULAR
      PRESETMODE_HIGHACCURACY
      PRESETMODE_ENHANCED
  '''
  bmm150.set_preset_mode(PRESETMODE_LOWPOWER)

  '''
    Enable or disable the pin :
      ENABLE_DRDY
      DISABLE_DRDY  (default mode)

    polarity  Active level
      POKARITY_HIGH  (default active high level )
      POKARITY_LOW
  '''
  bmm150.set_data_readly_interrupt_pin(ENABLE_DRDY ,POKARITY_HIGH)

def loop():
  if bmm150.get_data_readly_state() == 1:
    rslt = bmm150.get_geomagnetic()
    print "mag x = %d ut"%rslt[0]
    print "mag y = %d ut"%rslt[1]
    print "mag z = %d ut"%rslt[2]
    print ""
  else:
    time.sleep(1)
  time.sleep(1)

if __name__ == "__main__":
  setup()
  while True:
    loop()