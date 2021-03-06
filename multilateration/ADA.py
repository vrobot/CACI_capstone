import serial
from scipy import signal
import numpy as np

from datetime import datetime

import sympy as sp
import scipy
import matplotlib.pyplot as plt
import random
import math
import sys
import ast

from itertools import combinations

from sympy import Eq, solve_linear_system, Matrix
from sympy.interactive import printing

from scipy.optimize import least_squares

import csv

printing.init_printing(use_unicode=True)

# Random Seed
random.seed(69)


from pynmeagps import NMEAReader
from math import radians, cos, sin, asin, sqrt



def find_str(s, start, end):
    substart = s.find(start) + len(start)
    subend = s.find(end)

    substring = s[substart:subend]

    return substring

def twos_complement(hexstr, bits):

    value = int(hexstr,16)

    if value & (1 << (bits-1)):

        value -= 1 << bits

    return value

def GPS_distance(lat1, lat2, lon1, lon2):
     
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
      
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a))
    
    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371
      
    # calculate the result
    return(c * r * 1000)

def get_GPS(port, baud, timeout_val):
    ser = serial.Serial(port, baud, timeout=timeout_val)
    blank = ''
    
    gps_data = blank

    GPS_start = 'acacacac'
    GPS_end = 'adadadad'
    # read serial port until stop character is hit
    while True:
        input_data = ser.readline()
        data = input_data.hex()

        if data != blank:
            #print(data)
            gps_data = gps_data + data
        
        if gps_data.find(GPS_end) != -1:
            break

    raw_gps = find_str(gps_data, GPS_start, GPS_end)
    raw_gps = raw_gps.lstrip('0')

    gps_processed = NMEAReader.parse(bytes.fromhex(raw_gps).decode('utf-8'))

    x = float(gps_processed.payload[2])/100
    frac_x, whole_x = math.modf(x)
    x = whole_x + (frac_x * 100) / 60

    y = float(gps_processed.payload[0])/100
    frac_y, whole_y = math.modf(y)
    y = whole_y + (frac_y * 100) / 60

    if gps_processed.payload[1] == 'S':
        y = -1*y

    if gps_processed.payload[3] == 'W':
        x = -1*x

    output_gps = [x, y]

    return output_gps

def get_uart(port, baud, timeout_val):
    
    ser = serial.Serial(port, baud, timeout=timeout_val)
    blank = ''

    sound_data = blank

    meta_start  = 'abababab'
    meta_end    = 'cdcdcdcd'

    sound_start = 'cdcdcdcd'
    sound_end   = 'efefefef'
    
    #print(meta_start)

    last_8 = None

    # read serial port until stop character is hit
    while True:
        input_data = ser.readline()
        data = input_data.hex()
        #data = bytes(input_data)
        if data != blank:
            #print(data)
            sound_data = sound_data + data
        
        if sound_data.find(sound_end) != -1:
            break

    meta = find_str(sound_data, meta_start, meta_end)
    sound = find_str(sound_data, sound_start, sound_end)
    #print('meta_data: ', meta)
    #print('sound_data: ', sound)

    return meta, sound

# meta_data_formatting: 
#    timer = first 8 char
#    name = next 8 char
#    gps = the rest
def parse_uart(meta, sound, save_to_file):

    # convert raw time to usable time
    raw_time = meta[:8]
    reversed_time = ''
    
    # reverse the order of the bytes
    for code in (raw_time[i:i+2] for i in range(len(raw_time)-2, -1, -2)):
        reversed_time += code

    # convert to samples
    output_time = int(reversed_time, 16)
    output_time /= 90000000.0
    

    raw_name = meta[8:16]
    output_name = raw_name.replace('0', '')

    # convert Raw gps Data
    raw_gps = meta[16:]

    raw_gps = raw_gps.lstrip('0')
    #raw_gps = '$' + raw_gps

    gps_processed = NMEAReader.parse(bytes.fromhex(raw_gps).decode('utf-8'))
    

    x = float(gps_processed.payload[2])/100
    frac_x, whole_x = math.modf(x)
    x = whole_x + (frac_x * 100) / 60

    y = float(gps_processed.payload[0])/100
    frac_y, whole_y = math.modf(y)
    y = whole_y + (frac_y * 100) / 60

    if gps_processed.payload[1] == 'S':
        y = -1*y

    if gps_processed.payload[3] == 'W':
        x = -1*x

    #print(x)
    #print(y)

    output_gps = [x, y]

    #convert raw sound to usable sound
    output_sound = [0]*(int(len(sound) / 4))

    # loop through grouping of two bytes and then reverse their order: abcd -> cdab
    j = 0
    for a, b, c, d in (sound[i:i+4] for i in range(0, len(sound), 4)):
        code = c+d+a+b
        output_sound[j] = twos_complement(code, 16)
        j += 1

    #print(output_sound)
    #print(len(output_sound))
    #print(raw_time)
    #print(output_time)
    return [output_name, output_time, output_gps, output_sound]


def adjust_time(timea, timeb):
    if ((timea - timeb) > 45000000):
        timeb += 90000000
    if ((timeb - timea) > 45000000):
        timea += 90000000

    return timea, timeb
# add padding to make the timestamps line up
#NOTE: use sound in secs not samples
def pad_sound(time_a, sample_a, time_b, sample_b, sample_rate):
    
    time_a, time_b = adjust_time(time_a, time_b)

    offset = time_a - time_b
    offset *= sample_rate
    offset = int(offset)
    #print('offset: ', offset)

    if(offset > 0):
        padding = [0]*abs(offset)
        sample_a = padding + sample_a
        sample_b = sample_b + padding

    if(offset < 0):
        padding = [0]*abs(offset)
        sample_b = padding + sample_b
        sample_a = sample_a + padding

    #print(len(sample_a))
    #print(len(sample_b))

    return sample_a, sample_b

def TDOA(signal_a, signal_b, sample_rate):
    #print(signal_a)
    #print(signal_b)
    c = signal.correlate(signal_a, signal_b, mode="full", method="fft")
    max_ind = np.argmax(np.abs(c))
    shift = (len(signal_b)-1) - max_ind
    return shift / sample_rate

#returns the difference in ToA of a and b: Ta - Tb
def cross_correlation(time_a, sample_a, time_b, sample_b, sample_rate):
    #print(sample_a)
    #print(sample_b)
    sample_a, sample_b = pad_sound(time_a, sample_a, time_b, sample_b, sample_rate)

    difference = TDOA(sample_a, sample_b, sample_rate)

    return difference

def LST(nodes, times, v):

    #print(nodes)
    #print(times)

    #print(np.array(nodes))
    #print(np.array([times]).T)

    nodes = np.concatenate((np.array(nodes), np.array([times]).T), axis=1).tolist()
    #nodes = zip(nodes,times)
    #print(nodes)
    def equations(guess):
            x, y, r = guess

            output = []
            
            #print(nodes)
            for combo in combinations(nodes, 2):
                #print('COMBO: ', combo)
                n1 = combo[0]
                n2 = combo[1]
                x1 = n1[0]
                y1 = n1[1]
                x2 = n2[0]
                y2 = n2[1]
                tdoa = n1[2] - n2[2]
                ddoa = v * tdoa
                output.append((math.sqrt((x - x1)**2 + (y - y1)**2) - math.sqrt((x - x2)**2 + (y - y2)**2)) - (ddoa - r))

            return tuple(output)

    mean_x = np.mean([row[0] for row in nodes])
    mean_y = np.mean([row[1] for row in nodes])

    inital_guess = (mean_x, mean_y, 0)

    final_guess = least_squares(equations, inital_guess)
    #print(final_guess)

    return [final_guess.x[0], final_guess.x[1]]


#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
# Universal Variables
SAVE_TO_FILE = True
READ_FROM_FILE = int(sys.argv[3])
FILE_NAME = ''
if (READ_FROM_FILE):
    FILE_NAME = sys.argv[4] #'34.41486_-119.7352/test_result_17-05-22_15-50-37.csv'
SPEED_OF_SOUND = 343.0 / 111000.0
SHOW_MAP = True
NUM_NODES = int(sys.argv[2])
SAMPLE_RATE = 46875
PORT = sys.argv[1]

#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------

# test data
#look in loop beloew to switch between test and uart input

#formatting is [output_name, output_time, output_gps, output_sound]
node_list = []
true_sound = []

if READ_FROM_FILE:
    node_list = list(csv.reader(open(FILE_NAME)))
    true_sound = [ast.literal_eval(node_list[0][0]), ast.literal_eval(node_list[0][1])]
    print(true_sound)
    node_list.pop(0)

    for i in range(NUM_NODES):
        for j in range(4):
            res = ast.literal_eval(node_list[i][j])

            node_list[i][j] = res

else:
    print("Format: [Latititude, Longitude]")
    true_sound = get_GPS(port=PORT, baud=115200, timeout_val=0)
    print("True sound: [" + str(true_sound[1]) + ", " + str(true_sound[0]) + "]")

    for i in range(NUM_NODES):
    
        # uart input
        meta, sound = get_uart(port=PORT, baud=115200, timeout_val=0)
    
        # test input
        #meta = m[i].lower()
        #sound = s[i].lower()
    

        node_list.append(parse_uart(meta, sound, SAVE_TO_FILE))
        print('node ', str(node_list[i][0]), ' recieved: [' + str(node_list[i][2][1]) + ", " + str(node_list[i][2][0]) + "]")
    node_list[0].append(0)
#print(node_list)

# write to file
if SAVE_TO_FILE and not(READ_FROM_FILE):
    now = datetime.now()
    # dd/mm/YY H:M:S
    dt_string = now.strftime("%d-%m-%y_%H-%M-%S")

    file_name = 'test_result_' + dt_string + '.csv'
    print("date and time =", file_name)
 
    # data to be written row-wise in csv file
 
    # opening the csv file in 'w+' mode
    file = open(file_name, 'w+', newline ='')
 
    # writing the data into the file
    with file:   
        write = csv.writer(file)
        write.writerow(true_sound)
        write.writerows(node_list)


#adjust for fact that longitude is terrible
long_multiplier = 180 / (math.sqrt(180**2 - node_list[0][2][1]**2) + .00000001)

node_locs = []
for node in node_list:

    x = node[2][0]
    y = node[2][1]

    x_mod = x*long_multiplier
    y_mod = y
    node_locs.append([x_mod, y_mod])


node_times = [0]

for node in node_list[1:]:
    tdoa = cross_correlation(node_list[0][1], node_list[0][3], node[1], node[3], SAMPLE_RATE)
    node_times.append(tdoa)

#print(node_locs)
#print(node_times)

pred_x, pred_y = LST(node_locs, node_times, SPEED_OF_SOUND)

pred_x = pred_x / long_multiplier

#frac_x, whole_x = math.modf(pred_x)
#pred_x_decimal = whole_x + (frac_x * 100) / 60

#frac_y, whole_y = math.modf(pred_y)
#pred_y_decimal = whole_y + (frac_y * 100) / 60


print('predicited latititude: ', pred_y)
print('predicited longitude: ', pred_x)
error = GPS_distance(pred_y, true_sound[1], pred_x, true_sound[0])
print("====================================")
print('ERROR DISTANCE: ', error, 'm')
print("====================================")

#x, y
#true_sound = [-119.86310, 034.41375]

# BBox = (-119.86416, -119.86271, 34.41415, 34.41333) IV park
if SHOW_MAP:
    BBox = (-119.73578, -119.73453, 34.41449, 34.41561) # Girsh park
    pmap = plt.imread('google_elings_park_map.JPG')
#map = plt.imread('IV_park_map.png')

#print(node_locs)
for i, n in enumerate(node_locs):
    node_locs[i][0] = n[0] / long_multiplier

    #frac_x, whole_x = math.modf(n[0])
    #node_locs[i][0] = whole_x + (frac_x * 100) / 60
    
    #frac_y, whole_y = math.modf(n[1])
    #node_locs[i][1] = whole_y + (frac_y * 100) / 60


point_size = 40
font_size = 8

for i in range(NUM_NODES):
    mark = '$'+ str(i) + '$'
    plt.scatter(node_locs[i][0], node_locs[i][1], s = point_size, color = 'black', marker=mark)

#plt.scatter(node_locs[0][0], node_locs[0][1], color = 'black', marker='$A$')
#plt.scatter(node_locs[1][0], node_locs[1][1], color = 'black', marker='$B$')
#plt.scatter(node_locs[2][0], node_locs[2][1], color = 'black', marker='$C$')

plt.scatter(true_sound[0], true_sound[1], s = point_size, color = 'red', marker='o', label='True sound location')
plt.scatter(pred_x, pred_y, s = point_size, color = 'blue', marker='X', label='Predicted sound location')

#whiffs on drawn graph
if SHOW_MAP:
    plt.scatter(-999, -999, color = 'black', marker = '$N$', label='nodes')

    plt.imshow(pmap, zorder=0, extent = BBox, aspect= 'equal')

plt.legend(loc="upper left", fontsize=font_size)

plt.axis('off')

manager = plt.get_current_fig_manager()
manager.full_screen_toggle()

#plt.show()