# Homework 2, Part B: Reliable UART Link and Protocol Analysis

## Bandwidth budget

At 10,000 samples/s and two bytes per sample, the payload is 20,000 bytes/s. With 8N1, every byte uses 1 start bit, 8 data bits, and 1 stop bit, so the wire rate is:

`10,000 samples/s * 2 bytes/sample * 10 bits/byte = 200,000 bit/s`

A baud rate with 10% margin must be at least 220,000 baud. The firmware uses **230,400 baud**, which is 15.2% above the 200,000 baud minimum.

## USART implementation and sample framing

The firmware configures USART1 through STM32 registers. PA9 is USART1_TX on AF7. With the 8 MHz APB2 clock, BRR is rounded from `8,000,000 / 230,400`; the transmitter and USART are enabled, and CR2 is cleared for 1 stop bit. This gives 8N1 with no parity. Transmit uses polling, which the assignment allows.

Each 12-bit ADC result is sent in two bytes:

- First byte: `0x80 | (sample >> 6)`, so bit 7 marks the high byte.
- Second byte: `sample & 0x3F`, with bit 7 clear.

The PC parser searches for a high-byte marker before accepting a low byte. If the stream is interrupted between bytes, it discards the incomplete sample and waits for the next marker, allowing it to regain byte alignment after reconnection.

## Wiring for this test

- Board PA9 / USART1_TX -> USB-UART adapter RXD
- Board GND -> adapter GND
- Adapter USB -> PC
- Leave adapter TXD and VCC disconnected
- Keep the function generator connected to PA0 and board GND; keep PA0 within 0 to 3.3 V

Keep the board's ST-LINK USB connection connected for power/debug. The serial plotter uses the USB-UART adapter's PC port (for example `/dev/ttyUSB0`) at 230400 baud.

## Reliability test to perform on the board

1. Set the generator to a 1 kHz input signal and start the firmware and plotter.
2. Run the stream for 1-2 minutes. Record the start/end times and keep the plotter visible.
3. Disconnect only the PA9-to-adapter-RXD jumper for about 2 seconds. Keep both grounds and the adapter's USB connection in place.
4. Reconnect PA9 to RXD. The plotter should change from `NO DATA` back to `RECEIVING` and increment its `recoveries` count. Save a screenshot showing the resumed waveform/status.

Record the measured result here:

- Run duration: 90 s (809,122 samples received, about 10,000 per second)
- Input waveform and frequency: 1 kHz from the function generator, measured 1000.0 Hz, about 1.2 Vpp (1.4 V to 2.6 V)
- Time disconnected: about 3 s (PA9 jumper pulled by hand, logged gap 3.4 s)
- Did plotter return to `RECEIVING`? Yes, stream resumed by itself at 32.0 s with no restart of board or script
- Recovery count shown: 1 (one deliberate disconnect)
- Screenshot filename: `screenshots/partb_1_receiving_recoveries_0.png` (before the pull, RECEIVING, recoveries: 0, freq: 1000.0 Hz), `screenshots/partb_2_no_data.png` (during the pull, NO DATA) and `screenshots/partb_3_receiving_recoveries_1.png` (after reconnect, RECEIVING, recoveries: 1, freq: 1000.0 Hz)
- Log: `logs/partb_run.txt`. It also shows one unrelated USB-adapter port glitch at 19.4 s, reopened automatically. It was not the deliberate disconnect.
