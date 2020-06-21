#!/usr/bin/env python
# NO Python3!


import time
import math 
import mraa

I2C_ADDRESS = 0x40
INA219_REG_CALIBRATION = 0x05
ina219_calValue =4096
MAX_EXPECTED_CURRENT_A = 2.0
Rshunt = 0.01
Current_LSB = MAX_EXPECTED_CURRENT_A / (2^15)
print("Current_LSB",Current_LSB )


Calib_Value = math.trunc(0.04096 / (Current_LSB * Rshunt))
print("Calib_Value ",Calib_Value ,"b:", "{0:b}".format(Calib_Value))


VBUS_MAX = 32	#V	(Assumes 32V, can also be set to 16V)
VSHUNT_MAX = 0.32 #	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
RSHUNT = 0.1		#	(Resistor value in ohms)

#1. Determine max possible current
MaxPossible_I = VSHUNT_MAX / RSHUNT
#MaxPossible_I = 3.2A

#2. Determine max expected current
MaxExpected_I = 1.0  #A

#3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
MinimumLSB = MaxExpected_I/32767
#MinimumLSB = 0.0000305             #(30.5uA per bit)
MaximumLSB = MaxExpected_I/4096
MaximumLSB = 0.000244              #(244uA per bit)

#4. Choose an LSB between the min and max values
 #  (Preferrably a roundish number close to MinLSB)
CurrentLSB = 0.0000400 #(40uA per bit)

#5. Compute the calibration register
Cal = math.trunc(0.04096 / (Current_LSB * RSHUNT))
Cal = 10240  #(0x2800)

ina219_calValue = 10240



# initialise I2C
x = mraa.I2c(0)
x.address(I2C_ADDRESS)


def calibrate():
	x.writeWordReg(0x05,(int)(2*Calib_Value))
	
def getCurrent():
	"""
	int16_t Adafruit_INA219::getCurrent_raw() {
	  uint16_t value;

	  // Sometimes a sharp load will reset the INA219, which will
	  // reset the cal register, meaning CURRENT and POWER will
	  // not be available ... avoid this by always setting a cal
	  // value even if it's an unfortunate extra step
	  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

	  // Now we can safely read the CURRENT register!
	  wireReadRegister(INA219_REG_CURRENT, &value);

	  return (int16_t)value;
	}
	"""
	calibrate()
	ina219_currentDivider_mA = 10 # Current LSB = 100uA per bit (1000/100 = 10)
  
	curr_raw = x.readWordReg(0x04)
	return  curr_raw / ina219_currentDivider_mA 




print("Start readings_________________")
calib0 = x.readWordReg(0x00)
print("reg 0x00:", "{0:b}".format(calib0))

		
while 1==1:
	  
	shV = x.readWordReg(0x01)
	print("reg 0x01 Shunt voltage:", shV)
	  
	volts = (x.readWordReg(0x02) >>3) *0.004 #return (int16_t)((value >> 3) * 4)
	print("reg 0x02 Volts", volts)



	"""
	The Power register and Current register default to 0 because the Calibration register defaults to 0, yielding a zero current value until the
	Calibration register is programmed
	"""
	pwr = x.readWordReg(0x03)
	print("reg 0x03  pwr:", pwr)
	 
	curr = getCurrent()
	print("reg 0x04 current:", curr)

	calib5 = x.readWordReg(0x05)
	print("reg 0x05:", "{0:b}".format(calib5))



	calib5 = x.readWordReg(0x05)
	print("reg 0x05:", "{0:b}".format(calib5))
	print("---------------------------------------")
	time.sleep(3)
	
