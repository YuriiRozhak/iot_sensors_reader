import time
import adafruit_dht
import board
import RPi.GPIO as GPIO
from smbus2 import SMBus

import sys
import math
import operator

t = 22 # assume current temperature. Recommended to measure with DHT22
h = 65 # assume current humidity. Recommended to measure with DHT22

# GPIO Pin
LIGHT_SENSOR_PIN = 24
MOTION_SENSOR_PIN = 27
BUZZER_PIN = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(LIGHT_SENSOR_PIN, GPIO.IN)

GPIO.setup(MOTION_SENSOR_PIN, GPIO.IN)

# Initial the dht device, with data pin connected to:
# dhtDevice = adafruit_dht.DHT22(board.D4)

# you can pass DHT22 use_pulseio=False if you wouldn't like to use pulseio.
# This may be necessary on a Linux single board computer like the Raspberry Pi,
# but it will not work in CircuitPython.
dhtDevice = adafruit_dht.DHT22(board.D17, use_pulseio=False)

def invert_bit(value):
    return value ^ 1

def to_boolean(value):
    return value == 1

def boolean_to_string(value):
    return "True" if value else "False"


# The load resistance on the board
RLOAD = 10.0
# Calibration resistance at atmospheric CO2 level
RZERO = 76.63
# Parameters for calculating ppm of CO2 from sensor resistance
PARA = 116.6020682
PARB = 2.769034857

# Parameters to model temperature and humidity dependence
CORA = 0.00035
CORB = 0.02718
CORC = 1.39538
CORD = 0.0018
CORE = -0.003333333
CORF = -0.001923077
CORG = 1.130128205

# Atmospheric CO2 level for calibration purposes
ATMOCO2 = 397.13

"""
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
"""

def getCorrectionFactor(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG):
    # Linearization of the temperature dependency curve under and above 20 degree C
    # below 20degC: fact = a * t * t - b * t - (h - 33) * d
    # above 20degC: fact = a * t + b * h + c
    # this assumes a linear dependency on humidity
    if t < 20:
        return CORA * t * t - CORB * t + CORC - (h-33.)*CORD
    else:
        return CORE * t + CORF * h + CORG

"""
@brief  Get the resistance of the sensor, ie. the measurement value

@return The sensor resistance in kOhm
"""

def getResistance(value_pin,RLOAD):
    return ((1023./value_pin) - 1.)*RLOAD

"""
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
"""

def getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD):
    return getResistance(value_pin,RLOAD) / getCorrectionFactor(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG)

"""
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
"""

def getPPM(PARA,RZERO,PARB,value_pin,RLOAD):
    return PARA * math.pow((getResistance(value_pin,RLOAD)/RZERO), -PARB)

"""
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
"""

def getCorrectedPPM(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,PARA,RZERO,PARB):
    return PARA * math.pow((getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD)/RZERO), -PARB)

"""
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
"""

def getRZero(value_pin,RLOAD,ATMOCO2,PARA,PARB):
    return getResistance(value_pin,RLOAD) * math.pow((ATMOCO2/PARA), (1./PARB))

"""
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
"""

def getCorrectedRZero(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,ATMOCO2,PARA,PARB):
    return getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD) * math.pow((ATMOCO2/PARA), (1./PARB))

"""
Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, 
a value of fromHigh to toHigh, values in-between to values in-between, etc.

# Arduino: (0 a 1023)
# Raspberry Pi: (0 a 26690)

More Info: https://www.arduino.cc/reference/en/language/functions/math/map/
"""

def map(x,in_min,in_max,out_min,out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def getCorrectedPPMRes(x):
    value_ads = x # value obtained by ADS1115
    value_pin = map((value_ads - 565), 0, 26690, 0, 1023) # 565 / 535 fix value
    rzero = getRZero(value_pin,RLOAD,ATMOCO2,PARA,PARB)
    correctedRZero = getCorrectedRZero(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,ATMOCO2,PARA,PARB)
    resistance = getResistance(value_pin,RLOAD)
    ppm = getPPM(PARA,RZERO,PARB,value_pin,RLOAD)
    correctedPPM = getCorrectedPPM(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,PARA,RZERO,PARB)
    return correctedPPM

def read_sensors():
    with SMBus(1) as bus:
        #MQ-3 connected to AIN1 on YL-40
        device_address = 0x48  # Your ADC device address

        while True:
            try:
                time.sleep(1)
                #get readings from sensors
                alco_data = bus.read_word_data(0x48, 0x41)
                gases_data = bus.read_word_data(0x48, 0x40)
                sensor_value = GPIO.input(LIGHT_SENSOR_PIN)  # Read digital value (0 or 1)

                gassesPPM = getCorrectedPPMRes(gases_data)
                concentration = (9.95 / 4096.0) * alco_data + 0.05
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                light_sensor_value = GPIO.input(LIGHT_SENSOR_PIN)
                motion_sensor_value = GPIO.input(MOTION_SENSOR_PIN)
                motion_sensor_bool =  boolean_to_string(to_boolean(motion_sensor_value))
                light_sensor_bool = boolean_to_string(to_boolean(invert_bit(light_sensor_value)))
                temperature_c = dhtDevice.temperature
                temperature_f = temperature_c * (9 / 5) + 32
                humidity = dhtDevice.humidity

                if sensor_value == 0:
                    print("Light Sensor value is 0! Deactivating buzzer...")
                    GPIO.setup(BUZZER_PIN, GPIO.IN)

                if sensor_value == 1:
                    print("Light Sensor value is 1! Activating buzzer...")
                    GPIO.setup(BUZZER_PIN, GPIO.OUT)

                print(f"[{timestamp}] Sensor Value: {sensor_value}")
                # Output data to screen
                #print("Raw_ADC : %.d" %alco_data)
                print("Gasses  : %.2f ppm" %gassesPPM)
                print("Alcohol : %.2f mg/l" %concentration)

                # Print the values to the serial port

                print(
                    "{} Temp: {:.1f} F / {:.1f} C  Humidity: {}%   DANGER_GASSES:{} ppm ALCOHOL: {:.2f} mg/l LIGHT: {}  MOTION: {}".format(
                        timestamp, temperature_f, temperature_c, humidity,gassesPPM, concentration, light_sensor_bool, motion_sensor_bool
                    )
                )

            except RuntimeError as error:
                # Errors happen fairly often, DHT's are hard to read, just keep going
                print(error.args[0])
                time.sleep(2.0)
                continue
            except Exception as error:
                dhtDevice.exit()
                raise error


# Run the logger
if __name__ == "__main__":
    try:
        read_sensors()
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
