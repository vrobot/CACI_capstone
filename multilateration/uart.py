import serial

def get_uart(port, baud, timeout_val):
    ser = serial.Serial(port, baud, timeout=timeout_val)
    
    while True:
        data = ser.readline()
        data_sensor = data.decode('utf8')
        print(data_sensor)