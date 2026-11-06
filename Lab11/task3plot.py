import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB1', 115200) # (Change to ttyACM0 if needed)
plt.ion()

# === Plot Styles to Rotate Through ===
plot_styles = ['b.-', 'g.-', 'm.-', 'c.-', 'y.-', 'k.-']

# === Data Storage ===
NUM_SAMPLES = 100
time_ms = []
cnt = 0
all_signals = []
signals = ["Temperature", "X", "Y", "Z"]

# === Plotting Function ===
def makeFig(title, xLabel, yLabel, *args):
    plt.clf()
    plt.title(title)
    plt.grid(True)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)

    for arg in args:
        plt.plot(time_ms, arg[0], arg[1], label=arg[2])

    plt.legend(loc='upper left')

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

        if len(values) != 4:
            continue

        parsed_vals = [int(v) for v in values]

        if len(all_signals) == 0:
            all_signals = [[] for _ in range(4)]

        for i, val in enumerate(parsed_vals):
            all_signals[i].append(val)

        time_ms.append(cnt)
        cnt += 1

        args = []
        for i in range(1, 4):
            style = plot_styles[(i-1) % len(plot_styles)]
            label = signals[i]
            args.append((all_signals[i], style, label))

        title = "Live Angles (Pitch & Roll)"
        drawnow(lambda: makeFig(title, "Time (ticks)", "Degrees", *args))
        plt.pause(0.0001)

        if len(time_ms) > NUM_SAMPLES:
            time_ms.pop(0)
            for signal in all_signals:
                signal.pop(0)

    except Exception:
        pass
