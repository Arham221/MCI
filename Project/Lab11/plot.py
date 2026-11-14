import collections
import serial
import matplotlib.pyplot as plt

PORT = '/dev/ttyUSB0'
BAUD = 115200
N = 300                                   # samples kept on screen
names = ["Acc Angle", "Gyro Rate", "Filtered Angle", "PID Out"]

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print("Port opened, waiting for data...")

data = [collections.deque(maxlen=N) for _ in names]

plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.set_title("Lab 11: Complementary Filter & PID")
ax1.set_ylabel("Angle (deg)")
ax2.set_ylabel("Rate / PID")
ax2.set_xlabel("Samples")
ax1.grid(True); ax2.grid(True)

lines = []
for i, nm in enumerate(names):
    ax = ax1 if i in (0, 2) else ax2      # angles on top, rate and PID below
    (ln,) = ax.plot([], [], label=nm)
    lines.append(ln)
ax1.legend(loc="upper left")
ax2.legend(loc="upper left")

first = True
while plt.fignum_exists(fig.number):
    got = False
    while ser.in_waiting:                 # read everything that has arrived
        raw = ser.readline().decode(errors="ignore").strip()
        parts = raw.split(",")
        if len(parts) != 4:
            continue
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue
        for d, v in zip(data, vals):
            d.append(v)
        got = True
        if first:
            print("First sample received:", vals)
            first = False

    if got:
        for ln, d in zip(lines, data):
            ln.set_data(range(len(d)), list(d))
        for ax in (ax1, ax2):
            ax.relim()
            ax.autoscale_view()
    plt.pause(0.05)

ser.close()