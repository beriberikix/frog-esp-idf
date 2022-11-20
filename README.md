# WIP ESP-IDF based Frog for the Ribbit Network

Currently implementing sensors using the `esp-idf-lib` project on a Feather ESP32-S3.

## Status

Implemented:
* LC709203F. Runs a loop that shows, every 10 seconds, temperature, voltage (V), rsoc (battery percentage) and ite(batttery percentage in 0.1% scale).

## Notes

The Feather ESP32-S3 has a batery monitor and I2C power enable pin on IO07. SCL is on IO04 & SDA is on IO03. Use `git submodule update --init --recursive` to fetch libraries as submodules.