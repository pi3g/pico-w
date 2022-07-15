import time
import board
import busio
import adafruit_bme680
import json

class BME68X:
    def __init__(self, SCL=board.GP1, SDA=board.GP0):
        # Initialize I2C
        # busio.I2C(SCL, SDA)
        self.i2c = busio.I2C(board.GP1, board.GP0)

        # Create BME68X object
        self.bme68x = adafruit_bme680.Adafruit_BME680_I2C(self.i2c)

        # change this to match the location's pressure (hPa) at sea level
        # bme68x.sea_level_pressure = 1013.25
        self.bme68x.sea_level_pressure = 1013

        # You will usually have to add an offset to account for the temperature of
        # the sensor. This is usually around 5 degrees but varies by use. Use a
        # separate temperature sensor to calibrate this one.
        self.temperature_offset = -9
        self.data = {}
        
    def read_data(self):
        self.data = {
            'T': self.bme68x.temperature + self.temperature_offset,
            'G': self.bme68x.gas,
            'H': self.bme68x.relative_humidity,
            'P': self.bme68x.pressure,
            'A': self.bme68x.altitude,
            }
        
        return self.data
    
    def load_data(self, file_name):
        try:
            with open (file_name, 'r') as file:
                data_list = json.load(file)
            
            return data_list
        except Exception as e:
            print(file_name + ' not found, created ' + file_name)
            
            return []

    def save_data(self, file_name):
        data_list = self.load_data(file_name)
        data_list.append(self.read_data())
        with open(file_name, 'w') as file:
            json.dump(data_list, file)
        
        return data_list[-1]