"""PC side of the oscilloscope.  pip install pyserial matplotlib numpy
Run:  python scope.py COM5      (Linux/Mac: python scope.py /dev/ttyUSB0)
Keys: space = pause/resume, s = save CSV, f = toggle FFT, up/down = zoom time axis"""
import sys, time, collections
import numpy as np, serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT, BAUD, FS, VREF = (sys.argv[1] if len(sys.argv) > 1 else "COM5"), 230400, 10000.0, 3.3
data = collections.deque(maxlen=20000)
state = dict(ser=None, hi=None, paused=False, fft=False, win=2000, total=0, t0=time.time(), rate=0.0)

def reopen():
    try:
        state["ser"] = serial.Serial(PORT, BAUD, timeout=0)
        state["hi"] = None
    except serial.SerialException:
        state["ser"] = None

def pump():
    ser = state["ser"]
    if ser is None:
        reopen(); return
    try:
        raw = ser.read(ser.in_waiting or 1)
    except (serial.SerialException, OSError):
        ser.close(); state["ser"] = None; return   # cable pulled: retry next frame
    for b in raw:
        if b & 0x80:                       # high byte (resynchronises us every sample)
            state["hi"] = b & 0x3F
        elif state["hi"] is not None:      # low byte
            data.append(((state["hi"] << 6) | b) * VREF / 4095.0)
            state["hi"] = None; state["total"] += 1
        # else: stray low byte after reconnect -> dropped

fig, ax = plt.subplots(figsize=(10, 5))
line, = ax.plot([], [], lw=1)
info = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", family="monospace")
ax.set_xlabel("time (ms)"); ax.set_ylabel("volts"); ax.grid(True)

def measure(v):
    vpp, avg = v.max() - v.min(), v.mean()
    mid = (v.max() + v.min()) / 2
    idx = np.where((v[:-1] < mid) & (v[1:] >= mid))[0]        # rising crossings
    if len(idx) < 2: return vpp, avg, 0.0
    frac = (mid - v[idx]) / (v[idx + 1] - v[idx])              # linear interpolation
    pos = idx + frac
    return vpp, avg, FS * (len(pos) - 1) / (pos[-1] - pos[0])

def update(_):
    pump()
    now = time.time()
    if now - state["t0"] >= 1.0:
        state["rate"] = state["total"] / (now - state["t0"]); state["total"] = 0; state["t0"] = now
        print(f"samples/s = {state['rate']:.0f}")
    if state["paused"] or len(data) < 200: return line,
    v = np.array(data)[-state["win"]:]
    vpp, avg, f = measure(v)
    if state["fft"]:
        sp = np.abs(np.fft.rfft(v - v.mean())) / len(v); fr = np.fft.rfftfreq(len(v), 1 / FS)
        line.set_data(fr, sp); ax.set_xlim(0, 3000); ax.set_ylim(0, max(sp.max() * 1.1, 1e-3))
        ax.set_xlabel("frequency (Hz)")
    else:
        t = np.arange(len(v)) / FS * 1000
        line.set_data(t, v); ax.set_xlim(0, t[-1]); ax.set_ylim(0, 3.4); ax.set_xlabel("time (ms)")
    info.set_text(f"Vpp={vpp:.3f} V  avg={avg:.3f} V  f={f:.1f} Hz  T={(1000/f if f else 0):.3f} ms  rate={state['rate']:.0f} S/s")
    return line, info

def key(e):
    if e.key == " ": state["paused"] = not state["paused"]
    elif e.key == "f": state["fft"] = not state["fft"]
    elif e.key == "up": state["win"] = max(200, state["win"] // 2)
    elif e.key == "down": state["win"] = min(20000, state["win"] * 2)
    elif e.key == "s": np.savetxt("capture.csv", np.array(data), header="volts", fmt="%.4f"); print("saved capture.csv")

fig.canvas.mpl_connect("key_press_event", key)
ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()
