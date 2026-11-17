# Run the Homework 2 live ADC plot

1. Flash `build/Debug/Homework_02.elf` to the STM32F3 Discovery board.
2. Wire the signal source to PA0 and GND. Keep PA0 between 0 V and 3.3 V.
3. For this project's UART code, connect PA9 (TX) to the 3.3 V USB-UART adapter's RXD and connect board GND to adapter GND. Leave adapter TXD and VCC disconnected. The board USB/ST-LINK connection powers the board.
4. Install Python 3 and create a local environment for the plotter packages in a terminal:

   ```sh
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install -r requirements-plotter.txt
   ```

5. Find the adapter's serial port and run one of these commands from this folder:

   ```sh
   python plotter.py /dev/ttyUSB0
   ```

   On Windows, for example:

   ```powershell
   py plotter.py COM3
   ```

The plotter uses 230400 baud, 8 data bits, no parity, and 1 stop bit (8N1), matching the firmware. It decodes the firmware's two-byte 12-bit ADC format and displays the most recent 100 ms of input voltage. Close the plot window before disconnecting the serial adapter.

To verify acquisition counts, use the debugger's Live Expressions for `samples_last_second`, `adc_overrun_count`, and `adc_drop_count`. Expected values are 10000, 0, and 0 respectively. The live plot does not replace the oscilloscope accuracy measurements required for the report.
