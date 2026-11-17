#!/usr/bin/env python3
"""Live plotter for the STM32F3 Homework 2 binary ADC stream."""

import argparse
from collections import deque
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

BAUD_RATE = 230400
SAMPLE_RATE_HZ = 10000
ADC_REFERENCE_V = 3.3
WINDOW_SAMPLES = 1000
LINK_TIMEOUT_S = 0.5


def measure_frequency(volts):
    """Frequency from rising crossings of the mid level (with hysteresis). 0.0 if no clear signal."""
    lo, hi = min(volts), max(volts)
    if hi - lo < 0.2:
        return 0.0
    mid, hyst = (lo + hi) / 2, (hi - lo) * 0.1
    crossings, armed = [], False
    for i in range(1, len(volts)):
        if volts[i] < mid - hyst:
            armed = True
        if armed and volts[i - 1] < mid <= volts[i]:
            # linear interpolation gives the crossing between two samples
            crossings.append(i - 1 + (mid - volts[i - 1]) / (volts[i] - volts[i - 1]))
            armed = False
    if len(crossings) < 2:
        return 0.0
    return SAMPLE_RATE_HZ * (len(crossings) - 1) / (crossings[-1] - crossings[0])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    return parser.parse_args()


def main():
    args = parse_args()
    samples = deque([0] * WINDOW_SAMPLES, maxlen=WINDOW_SAMPLES)
    pending_high = None
    last_sample_time = None
    recovery_count = 0

    # Nonblocking reads keep the plot responsive while the board streams.
    with serial.Serial(args.port, BAUD_RATE, timeout=0) as ser:
        fig, ax = plt.subplots()
        line, = ax.plot([], [], linewidth=1)
        ax.set_title("STM32F3 ADC live samples (10 kHz) | WAITING FOR DATA")
        ax.set_xlabel("Time (ms)")
        ax.set_ylabel("Input voltage (V)")
        ax.set_xlim(-100, 0)
        ax.set_ylim(0, ADC_REFERENCE_V)
        ax.grid(True, alpha=0.3)
        time_ms = [(i - WINDOW_SAMPLES + 1) * 1000.0 / SAMPLE_RATE_HZ
                   for i in range(WINDOW_SAMPLES)]

        def update(_frame):
            nonlocal pending_high, last_sample_time, recovery_count
            decoded_any = False
            available = ser.in_waiting
            if available:
                data = ser.read(available)
                for byte in data:
                    if pending_high is None:
                        if byte & 0x80:
                            pending_high = byte & 0x3F
                    elif byte & 0x80:
                        # A new high-byte marker means a previous low byte was lost.
                        pending_high = byte & 0x3F
                    else:
                        raw = (pending_high << 6) | (byte & 0x3F)
                        samples.append(raw * ADC_REFERENCE_V / 4095.0)
                        pending_high = None
                        decoded_any = True

            now = time.monotonic()
            if decoded_any:
                if (last_sample_time is not None and
                        now - last_sample_time >= LINK_TIMEOUT_S):
                    recovery_count += 1
                last_sample_time = now

            if last_sample_time is None:
                link_status = "WAITING FOR DATA"
            elif now - last_sample_time >= LINK_TIMEOUT_S:
                link_status = "NO DATA"
            else:
                link_status = "RECEIVING"
            volts = list(samples)
            freq = measure_frequency(volts) if link_status == "RECEIVING" else 0.0
            ax.set_title(
                f"STM32F3 ADC live samples (10 kHz) | {link_status} | "
                f"recoveries: {recovery_count} | freq: {freq:.1f} Hz"
            )
            line.set_data(time_ms, volts)
            return (line,)

        animation = FuncAnimation(fig, update, interval=30, blit=False,
                                  cache_frame_data=False)
        # Keep a reference alive for the lifetime of the window.
        fig._adc_animation = animation
        plt.show()


if __name__ == "__main__":
    main()
