F450 + Pixhawk + Raspberry Pi 3B+ (MAVLink / DroneKit)
A complete, reproducible build of an autonomous quadcopter using an F450 frame, Pixhawk 2.4.8 (ArduCopter), and a Raspberry Pi 3B+ companion computer communicating over MAVLink. This project focuses on wiring and software bring-up with Python scripts using DroneKit and MAVProxy for guided flight, mission upload, and telemetry logging.

⚠️ Safety first: Always perform bench tests with props removed. Use a clear outdoor test area and configure appropriate failsafes.

What You’ll Build
A Pixhawk 2.4.8 based quadcopter on an F450 X-quad airframe with GPS/compass, 30A ESCs, and 10x4.5 props.

A Raspberry Pi 3B+ connected to Pixhawk TELEM2 port via UART, streaming MAVLink messages.

Python DroneKit scripts for connect → arm → takeoff → navigate workflows with logging and MAVProxy utilities.

Hardware and assembly notes are available in docs/assembly.md.

Hardware Overview
Airframe: F450 X-quad, approx. 280 g, powered by 3-4S LiPo batteries for 10-20 min flight endurance.

Autopilot: Pixhawk 2.4.8 running ArduCopter firmware.

Companion Computer: Raspberry Pi 3B+ connected via TTL UART (3.3V logic).

Telemetry and RC: FS-i6X and IA6B radios (500 m+ LOS), optional 915 MHz telemetry radios.

Wiring Pixhawk to Raspberry Pi
Connect Pixhawk TELEM2 TX to Pi GPIO15 (RX)

Connect Pixhawk TELEM2 RX to Pi GPIO14 (TX)

Connect GND to GND

Keep wiring short; Pixhawk and Pi use 3.3 V logic.

Optionally, use a USB-serial adapter (FTDI/CP2102) connected to TELEM2 for USB-based connection.

See docs/wiring.md for detailed diagrams and photos.

Raspberry Pi Setup
Enable UART and disable serial login shell using sudo raspi-config.

After reboot, UART device available at /dev/serial0.

Add your user to the dialout group for serial device permissions.

Use Python 3.9 (DroneKit support) in a virtual environment:

text
python3.9 -m venv ~/dronekit-py39
source ~/dronekit-py39/bin/activate
pip install --upgrade pip
pip install dronekit pymavlink future pyserial
MAVProxy
Install MAVProxy inside the venv or system-wide.

Connect via:

text
mavproxy.py --master=/dev/serial0 --baudrate 57600 --aircraft MyCopter
Use MAVProxy CLI to monitor status, parameters, and run tests.

Pixhawk Configuration
Map AUX outputs to motors if using AUX pins:

text
param set BRD_PWM_COUNT 4
param set SERVO9_FUNCTION 33
param set SERVO10_FUNCTION 34
param set SERVO11_FUNCTION 35
param set SERVO12_FUNCTION 36
reboot
Test motors safely with motortest commands in MAVProxy.

Example Python Scripts
scripts/connect_pixhawk.py: Connects to Pixhawk, streams heartbeat and telemetry data.

scripts/arm_and_takeoff.py: Arms, takes off, holds position, and returns to launch.

Run scripts inside your Python virtual environment.

Bring-up Checklist
Verify RC failsafes, geofence, and RTL altitude.

Complete compass and accelerometer calibrations.

Check telemetry sanity including GPS, EKF status, battery level, and flight mode.

Perform bench tests with props off before flight.

Troubleshooting
No serial link? Confirm UART enabled, serial login disabled, and device permissions.

Permission denied on /dev/serial0? Add user to dialout group.

Motors not responding? Check AUX mappings and BRD_PWM_COUNT settings.
