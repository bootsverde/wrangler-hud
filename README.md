# Wrangler HUD

An Arduino-based heads-up display for a Jeep Wrangler, showing pitch, roll, heading, and GPS data on a TFT screen.

## Hardware

- **Display:** TFT screen via TFT_eSPI (480×320 landscape)
- **IMU:** LSM6DSOX (pitch, roll, gyro heading)
- **GPS:** NMEA module on Serial1 at 9600 baud
- **Input:** Rotary encoder with push button (pins 2, 3, 6)

## Screens

- **Horizon** — artificial horizon with roll/pitch overlay and danger warning above 35°
- **GPS** — speed, lat/lon, satellites, altitude
- **Info** — raw pitch, roll, heading, and calibration offsets

## Calibration

Select **Calibrate** from the menu to zero out pitch, roll, and heading at the current position.

## Dependencies

Install via Arduino Library Manager:

- `TFT_eSPI`
- `Adafruit LSM6DS`
- `TinyGPSPlus`
