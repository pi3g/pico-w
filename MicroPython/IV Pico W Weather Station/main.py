import network
import machine
import time
import socket
import bme68x

def blink_onboard_led(num_blinks, delta_t):
    led = machine.Pin('LED', machine.Pin.OUT)
    for i in range(num_blinks):
        led.on()
        time.sleep(delta_t)
        led.off()
        time.sleep(delta_t)

data = {
    'T': 0,
    'P': 0,
    'A': 0,
    'H': 0,
    'G': 0,
    }

def print_data(data):
    print("\nTemperature: %0.1f C" % data['T'])
    print("Gas: %d ohm" % data['G'])
    print("Humidity: %0.1f %%" % data['H'])
    print("Pressure: %0.3f hPa" % data['P'])
    print("Altitude = %0.2f meters" % data['A'])
        
# Function to load in html page    
def get_html(html_name):
    with open(html_name, 'r') as file:
        html = file.read()
        
    return html

# Set up SoftAP
essid = 'Pico-W-Weather-Sation'
password = '#FreeThePicoW'

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=essid, password=password)

while ap.active() == False:
    pass

print('Connection successfull')
print(ap.ifconfig())

# Create sensor object
sensor = bme68x.BME68X()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(3)
blink_onboard_led(3, 0.2)

while True:
    conn, addr = s.accept()
    print('Got a connection from %s' % str(addr))
    request = conn.recv(1024)
    # print('Content = %s' % str(request))
    
    # Load html and replace with current data 
    response = get_html('index.html')
    data = sensor.save_data('data.json')
    blink_onboard_led(1, 0.2)
    response = response.replace('id_temp', str(data['T']))
    response = response.replace('id_pres', str(data['P']))
    response = response.replace('id_alti', str(data['A']))
    response = response.replace('id_humi', str(data['H']))
    response = response.replace('id_gas', str(data['G']))
    print_data(data)
    conn.send(response)
    conn.close()
