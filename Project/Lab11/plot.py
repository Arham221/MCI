"""
Lab 11 live plotter + command console.

    python3 plot.py                 # auto-detects the serial port
    python3 plot.py /dev/ttyUSB0    # or give it explicitly

Plots whatever comma-separated numbers the board sends (4 columns in mode 0,
7 in mode 1, 6 in mode 2). Lines starting with '#' are printed, not plotted.
Type commands in this terminal and press Enter, e.g.:
    e        arm / disarm          p 60   d 1.5   i 0     set gains
    z        set-point = current angle                    m 300  motor test (m 0 stops)
    v 1      raw IMU mode          v 2    PID terms       ?      show parameters

Needs: pip install pyserial matplotlib
"""
import collections
import sys
import threading

import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports

BAUD = 115200
N = 400  # samples on screen (20 Hz -> 20 s)
NAMES = {
    4: ["acc angle (deg)", "gyro rate (deg/s)", "filtered angle (deg)", "PID out (counts)"],
    7: ["ax (g)", "ay (g)", "az (g)", "gx (deg/s)", "gy (deg/s)", "gz (deg/s)", "loop (us)"],
    6: ["angle (deg)", "setpoint (deg)", "P", "I", "D", "PID out"],
}
TOP = {4: [0, 2], 7: [0, 1, 2], 6: [0, 1]}  # which columns go on the upper axes


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = [p.device for p in serial.tools.list_ports.comports()
             if "USB" in p.device or "ACM" in p.device or "COM" in p.device]
    if not ports:
        sys.exit("No serial port found - pass it as an argument.")
    return ports[0]


def console(ser):
    """Send whatever the user types to the board."""
    for line in sys.stdin:
        ser.write((line.strip() + "\n").encode())


def main():
    port = find_port()
    ser = serial.Serial(port, BAUD, timeout=0.05)
    print(f"Opened {port} @ {BAUD}. Type commands + Enter (e, p 60, d 1.5, z, ?...)")
    threading.Thread(target=console, args=(ser,), daemon=True).start()

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    ncols, data, lines = None, [], []

    def setup(n):
        nonlocal data, lines
        names = NAMES.get(n, [f"ch{i}" for i in range(n)])
        top = TOP.get(n, list(range(n)))
        data = [collections.deque(maxlen=N) for _ in range(n)]
        for ax in (ax1, ax2):
            ax.cla()
            ax.grid(True)
        lines = [(ax1 if i in top else ax2).plot([], [], label=names[i])[0] for i in range(n)]
        ax1.legend(loc="upper left")
        ax2.legend(loc="upper left")
        ax1.set_title("Lab 11 - tilt estimation and PID")
        ax2.set_xlabel("sample (20 Hz)")

    buf = b""
    while plt.fignum_exists(fig.number):
        buf += ser.read(ser.in_waiting or 1)
        *rows, buf = buf.split(b"\n")
        for raw in rows:
            text = raw.decode(errors="ignore").strip()
            if not text:
                continue
            if text.startswith("#"):
                print(text)
                continue
            try:
                vals = [float(v) for v in text.split(",")]
            except ValueError:
                continue
            if len(vals) != ncols:
                ncols = len(vals)
                setup(ncols)
            for dq, v in zip(data, vals):
                dq.append(v)
        if lines:
            for ln, dq in zip(lines, data):
                ln.set_data(range(len(dq)), dq)
            for ax in (ax1, ax2):
                ax.relim()
                ax.autoscale_view()
        plt.pause(0.03)
    ser.close()


if __name__ == "__main__":
    main()
