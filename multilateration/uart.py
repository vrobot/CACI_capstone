import serial

def get_uart(port, baud, timeout_val):
    
    #ser = serial.Serial(port, baud, timeout=timeout_val)
    blank = bytearray('', "ascii")

    sound_data = blank

    meta_start = bytearray('ABABABAB', "ascii")
    meta_end = bytearray('CDCDCDCD', "ascii")

    sound_start = bytearray('EFEFEFEF', "ascii")
    sound_end = bytearray('GHGHGHGH', "ascii")

    transmit_end = bytearray('IJIJIJIJ', "ascii")
    
    print(meta_start)

    last_8 = None
'''
    while True:
        data = ser.readline()

        if data != blank:

            print(data)
            sound_data.extend(data)
            print(sound_data)

            if len(data) >= 8:
'''



get_uart(port="COM3", baud=115200, timeout_val=0)