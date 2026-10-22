import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
# Matches your terminal command port and baud rate
ser = serial.Serial('/dev/ttyUSB0', 115200)
plt.ion()

# === Data Storage ===
NUM_SAMPLES = 100
time_array = []
temp_data = []
x_data = []
y_data = []
z_data = []
cnt = 0

# === Plotting Function ===
# drawnow expects a function with zero arguments
def plot_data():
    plt.clf()
    plt.title('Gyroscope & Temperature Data')
    plt.grid(True)
    plt.xlabel('Samples')
    plt.ylabel('Values')

    plt.plot(time_array, temp_data, 'r.-', label='Temperature')
    plt.plot(time_array, x_data, 'b.-', label='X dps')
    plt.plot(time_array, y_data, 'g.-', label='Y dps')
    plt.plot(time_array, z_data, 'm.-', label='Z dps')

    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    line = ser.readline().decode('utf-8').strip()
    
    # 1. Parse the data first
    try:
        temp, x, y, z = map(float, line.split(','))
    except Exception as e:
        continue  # Skip incomplete lines silently
        
    # 2. Append to ALL lists only after a successful parse
    time_array.append(cnt)
    cnt += 1
    
    temp_data.append(temp)
    x_data.append(x)
    y_data.append(y)
    z_data.append(z)
    
    # Optional: Keep memory clean by limiting plot to last NUM_SAMPLES
    if len(time_array) > NUM_SAMPLES:
        time_array.pop(0)
        temp_data.pop(0)
        x_data.pop(0)
        y_data.pop(0)
        z_data.pop(0)
    
    # 3. Plot
    drawnow(plot_data)
