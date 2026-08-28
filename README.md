# Forrest Light Meter

A light meter built on the SparkFun ESP32-C6 dev module, targeting the native
Espressif IDF (ESP-IDF) framework.

## Status

- [x] Read ambient light from a VEML7700 (I2C) and print lux to the serial console
- [ ] Analog photodiode reading scheme
- [ ] LCD display output

## Hardware

- SparkFun ESP32-C6 dev module
- SparkFun VEML7700 ambient light sensor (Qwiic / I2C)

Wired via the board's Qwiic connector, which on SparkFun ESP32-C6 boards maps
to GPIO6 (SDA) / GPIO7 (SCL). If you're on a variant with a different pinout,
update `I2C_SDA_GPIO` / `I2C_SCL_GPIO` in [`main/main.c`](main/main.c).

## Building

Requires the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/get-started/)
toolchain installed and exported into your shell (`export.sh` / `export.ps1`).

```
idf.py set-target esp32c6
idf.py build
idf.py -p <PORT> flash monitor
```

`idf.py monitor` prints the lux readings logged over serial. Exit with `Ctrl+]`.
