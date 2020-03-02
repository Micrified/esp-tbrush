# ESP IoT Toothbrush

An IoT Toothbrush application based on FreeRTOS. This project is designed to run with the following hardware: 

1. TinyPICO Microcontroller (ESP32)
2. Adafruit MPU/IMU 6050

The application is additionally designed to communicate with a mobile application over Bluetooth Low Energy. 

---

## Usage

Make sure you have the `esp-idf` toolchain available. Before building the project, it is recommended to run `git submodule update --init --recursive` to make sure that the toolchain is up-to-date. 

1. Compile with `idf.py build`
2. Flash the project with `idf.py flash`. If a port must be specified (macOS), then use `idf.py -p /dev/tty.SLAB_USBtoUART` (assuming that `SLAB_USBtoUART` is the appropriate port). 
3. Preferrably in another terminal, use `screen` to read output. This should be `screen /dev/tty.SLAB_USBtoUART 115200` for macOS, and `screen /dev/ttyUSB0 115200` for Ubuntu. Again, make sure you specify the same port as used in step (2). 
4. You can exit screen with `ctrl-a` followed by `ctrl \` if you're currently in the screen. Alternatively, you can stop it from another terminal with `screen -ls` and then specifying the PID of the screen session in `screen -XS <pid> quit`. 

This should summarize all necessary steps to develop for this project. When things go arwy, you may try using: 
1. `idf.py clean`
2. `idf.py fullclean`

To rebuild the project from scratch. See the documentation for a more in-depth explanation. 