import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200) # (Change to ttyACM0 if needed)
plt.ion()

# === Plot Styles to Rotate Through ===
plot_styles = ['b.-', 'g.-', 'm.-', 'r.-', 'y.-', 'k.-']

# === Data Storage ===
NUM_SAMPLES = 100
time_ms = []
cnt = 0
all_signals = []
# Updated to match Lab 11 Stage B data:
signals = ["Acc Angle", "Gyro Rate", "Filtered Angle", "PID Out"] 

# === Plotting Function ===
def makeFig(title, xLabel, yLabel, *args):
    plt.clf()
    ax1 = plt.subplot(2, 1, 1)
    ax1.set_title(title)
    ax1.grid(True)
    ax1.set_ylabel("Angle (deg)")
    ax2 = plt.subplot(2, 1, 2)
    ax2.grid(True)
    ax2.set_xlabel(xLabel)
    ax2.set_ylabel("Rate / PID")
    for i, arg in enumerate(args):
        target = ax1 if i in (0, 2) else ax2     # acc angle + filtered angle on top
        target.plot(time_ms, arg[0], arg[1], label=arg[2])
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')
# === Main Loop ===
while True:
    
    # --- THE FIX: Fast-forward to real-time! ---
    # If there is a pile-up of data in the buffer, burn through it 
    # and throw it away until we reach the newest data.
    while sinWaveData.in_waiting > 100:
        sinWaveData.readline()
        
    while sinWaveData.in_waiting == 0:
        pass  # Wait for fresh data

    try:
        line = sinWaveData.readline().decode().strip()
        values = line.split(',')
        print(values)
        
        if len(values) != 4:
            print("skipped:", line)
            continue

        # FIX 1: Change int to float so it doesn't crash on decimals
        parsed_vals = [float(v) for v in values] 

        if len(all_signals) == 0:
            all_signals = [[] for _ in range(4)]

        for i, val in enumerate(parsed_vals):
            all_signals[i].append(val)

        time_ms.append(cnt)
        cnt += 1

        args = []
        # FIX 2: Loop from 0 to 4 to plot ALL signals (not 1 to 4)
        for i in range(4): 
            style = plot_styles[i % len(plot_styles)]
            label = signals[i]
            args.append((all_signals[i], style, label))

        title = "Lab 11: Complementary Filter & PID"
        drawnow(lambda: makeFig(title, "Time (ticks)", "Value", *args))
        plt.pause(0.0001)

        if len(time_ms) > NUM_SAMPLES:
            time_ms.pop(0)
            for signal in all_signals:
                signal.pop(0)

    except Exception as e:
        # Actually print the error so it doesn't fail silently
        print(f"Error parsing data: {e}") 
        pass
