# -*- coding:utf-8 -*-
""" 
  @file self_test.py
  @brief self test
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

  value = bmm150.self_test()
  if value == SELF_TEST_XYZ_OK:
    print "self test success"
  elif value == SELF_TEST_YZ_FAIL:
    print "self test yz fail"
  elif value == SELF_TEST_XZ_FAIL:
    print "self test xz fail"
  elif value == SELF_TEST_Z_FAIL:
    print "self test z fail"
  elif value == SELF_TEST_XY_FAIL:
    print "self test xy fail"
  elif value == SELF_TEST_Y_FAIL:
    print "self test y fail"
  elif value == SELF_TEST_X_FAIL:
    print "self test x fail"
  else:
    print "self test error"

def loop():
  exit()

if __name__ == "__main__":
  setup()
  while True:
    loop()