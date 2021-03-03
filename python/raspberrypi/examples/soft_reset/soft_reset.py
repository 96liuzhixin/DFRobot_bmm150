# -*- coding:utf-8 -*-
"""
  @file soft_reset.py
  @brief after software reset ,resume sleep mode ,(Suspended mode cannot be reset)
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
  version  V1.0
  date  2021-03-03
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
  while -1 == bmm150.sensor_init():
    print "sensor init error ,please check connect"

  '''
      POWERMODE_NORMAL
      POWERMODE_FORCED
      POWERMODE_SLEEP
      POWERMODE_SUSPEND
  '''
  bmm150.set_operation_mode(POWERMODE_NORMAL)

  mode = bmm150.get_operation_mode()
  if mode == POWERMODE_NORMAL:
    print "normal mode"
  elif mode == POWERMODE_SLEEP:
    print "sleep mode"
  elif mode == POWERMODE_SUSPEND:
    print "suspend mode"
  else:
    print "forced mode"
  time.sleep(1)
  # After software reset ,resume sleep mode ,(Suspended mode cannot be reset)
  bmm150.soft_reset()

def loop():
  mode = bmm150.get_operation_mode()
  if mode == POWERMODE_NORMAL:
    print "normal mode"
  elif mode == POWERMODE_SLEEP:
    print "sleep mode"
  elif mode == POWERMODE_SUSPEND:
    print "suspend mode"
  else:
    print "forced mode"
  time.sleep(1)
  exit()

if __name__ == "__main__":
  setup()
  while True:
    loop()