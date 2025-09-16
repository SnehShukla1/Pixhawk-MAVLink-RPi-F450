text
# F450 + Pixhawk + Raspberry Pi 3B+ (MAVLink / DroneKit)

A complete, reproducible build of an autonomous quadcopter using an **F450 frame**, **Pixhawk 2.4.8 (ArduCopter)**, and a **Raspberry Pi 3B+** companion computer communicating over **MAVLink**. This project focuses on wiring and software bring-up with Python scripts using DroneKit and MAVProxy for guided flight, mission upload, and telemetry logging.

> ⚠️ **Safety first:** Always perform bench tests with props removed. Use a clear outdoor test area and configure appropriate failsafes.

---

## What You’ll Build

- A Pixhawk 2.4.8 based quadcopter on an F450 X-quad airframe with GPS/compass, 30A ESCs, and 10x4.5 props.
- A Raspberry Pi 3B+ connected to Pixhawk TELEM2 port via UART, streaming MAVLink messages.
- Python DroneKit scripts for connect → arm → takeoff → navigate workflows with logging and MAVProxy utilities.
- Hardware and assembly notes are available in `docs/assembly.md`.

---

## Hardware Overview

- **Airframe:** F450 X-quad, approx. 280 g, powered by 3-4S LiPo batteries for 10-20 min flight endurance.
- **Autopilot:** Pixhawk 2.4.8 running ArduCopter firmware.
- **Companion Computer:** Raspberry Pi 3B+ connected via TTL UART (3.3V logic).
- **Telemetry and RC:** FS-i6X and IA6B radios (500 m+ LOS), optional 915 MHz telemetry radios.

---

## Wiring Pixhawk to Raspberry Pi

Connect Pixhawk TELEM2 to Raspberry Pi UART pins as follows:

Pixhawk TELEM2 TX → Pi GPIO15 (RX)
Pixhawk TELEM2 RX → Pi GPIO14 (TX)
Pixhawk GND → Pi GND

text

- Keep wiring short; Pixhawk and Pi use 3.3 V logic.
- Optionally, use a USB-serial adapter (FTDI/CP2102) connected to TELEM2 for USB-based connection.
- See `docs/wiring.md` for detailed diagrams and photos.

---

## Raspberry Pi Setup

Enable UART and disable serial login shell via:

```sudo raspi-config```

text

- Navigate: Interface Options → Serial Port → Login shell over serial? **No** → Enable serial port hardware? **Yes**
- Reboot your Pi.
- UART device is available at `/dev/serial0`.
- Add your user to the dialout group for serial device permissions:

'''sudo usermod -aG dialout $USER'''

text

- Log out and back in (or reboot) for permissions to take effect.
- Use Python 3.9 for DroneKit support. Create a virtual environment and install dependencies:

python3.9 -m venv ~/dronekit-py39
source ~/dronekit-py39/bin/activate
pip install --upgrade pip
pip install dronekit pymavlink future pyserial

text

---

## MAVProxy (handy CLI for testing)

Install MAVProxy inside the virtual environment or globally. Connect using:

mavproxy.py --master=/dev/serial0 --baudrate 57600 --aircraft MyCopter

text

This opens a MAVLink console with modules for status, parameters, and motor testing.

---

## Pixhawk Configuration (AUX motor mapping & tests)

If motors are wired to AUX outputs (not MAIN), map AUX1..AUX4 to motors 1..4 and ensure AUX pins are PWM outputs by running these commands in MAVProxy or via a ground control station:

param set BRD_PWM_COUNT 4
param set SERVO9_FUNCTION 33
param set SERVO10_FUNCTION 34
param set SERVO11_FUNCTION 35
param set SERVO12_FUNCTION 36
reboot

text

Test each motor safely (props off):

motortest 1 1 1200 5
motortest 2 1 1200 5
motortest 3 1 1200 5
motortest 4 1 1200 5

text

The above mapping routes AUX1..AUX4 to Motor1..Motor4 accordingly. For more information about the `BRD_PWM_COUNT` and output functions, consult ArduPilot’s documentation.

---

## Example Python Scripts

### `scripts/connect_pixhawk.py`

#!/usr/bin/env python3
from future import print_function
import time
from dronekit import connect

Connect via Raspberry Pi UART to Pixhawk TELEM2 (default baudrate 57600)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True, timeout=60)

print("Connected. Firmware:", vehicle.version)
print("GPS:", vehicle.gps_0, " EKF OK:", vehicle.ekf_ok)
print("Mode:", vehicle.mode.name, " Armable:", vehicle.is_armable)

Stream a few heartbeats
for _ in range(5):
print("HB: altitude=%.1f m groundspeed=%.1f m/s batt=%.1f%%" %
(vehicle.location.global_relative_frame.alt,
vehicle.groundspeed,
(vehicle.battery.level or -1)))
time.sleep(1)

vehicle.close()

text

Run inside your virtual environment:

sudo ~/dronekit-py39/bin/python scripts/connect_pixhawk.py

text

---

### `scripts/arm_and_takeoff.py`

#!/usr/bin/env python3
from future import print_function
import time
from dronekit import connect, VehicleMode

def wait_until_armable(v):
while not v.is_armable:
print("Waiting for EKF/IMU/GPS... mode=%s fix=%s sat=%s" %
(v.mode.name, getattr(v.gps_0,'fix_type',None), getattr(v.gps_0,'satellites_visible',None)))
time.sleep(1)

def set_mode(v, name):
v.mode = VehicleMode(name)
while v.mode.name != name:
time.sleep(0.2)

def arm_and_takeoff(v, alt_m):
wait_until_armable(v)
set_mode(v, "GUIDED")
v.armed = True
while not v.armed:
time.sleep(0.2)
v.simple_takeoff(alt_m)
while True:
alt = v.location.global_relative_frame.alt or 0.0
print(" Alt: %.1f" % alt)
if alt >= 0.95 * alt_m:
break
time.sleep(0.5)

def main():
v = connect('/dev/serial0', baud=57600, wait_ready=True, timeout=60)
print("Connected:", v.version)
arm_and_takeoff(v, 5.0)
print("Holding position for 10 s ...")
time.sleep(10)
print("RTL")
set_mode(v, "RTL")
time.sleep(2)
v.close()

if name == "main":
main()

text

Run inside your virtual environment:

sudo ~/dronekit-py39/bin/python scripts/arm_and_takeoff.py

text

---

## Bring-up Checklist

- Verify RC failsafe, geofence, and RTL altitude settings.
- Complete compass and accelerometer calibrations.
- Check telemetry sanity: GPS, EKF status, battery, and flight mode.
- Perform bench tests with props off before flight.

---

## Troubleshooting

- **No serial link?** Confirm UART enabled, serial login disabled, device exists at `/dev/serial0`.
- **Permission denied on `/dev/serial0`?** Add your user to `dialout` group and reboot.
- **Motors not responding?** Check AUX pin mappings and `BRD_PWM_COUNT` parameters.

---

## License

Add license information here, e.g., MIT License or any other license you choose.

---

## Contact

For questions or contributions, feel free to open issues or submit pull requests.

---

Feel free to modify or extend this README to suit your specific needs!

---

Would you like me to provide this file in raw Markdown text ready to copy, or any specifi
