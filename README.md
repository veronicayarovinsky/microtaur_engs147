# README.md for Micromerge / Microtaur Project


### terminal commands to build, upload & run platformIO code based on platformio.ini configs
Notes:
* `-e` selects a specific named environment from platformio.ini
* `-t` specifies a target (upload, clean, size, etc) within a build

`pio run -e main`                       # compile main build (no upload)
`pio run -e main -t upload`             # compile + upload main build
`pio device monitor`                    # open serial monitor (uses baud rate specified by monitor_speed)
`pio device monitor -e main`
`pio device monitor > specific_log_filename.csv`    # redirect serial monitor output to a file you name

`pio test -e test_motor_pi`             # compile + upload + run one test
`pio test`                              # runs all of the tests (in the order from platformio.ini environments)

Other:
`pio device list`                       show the available serial ports
`pio device monitor > data_log.csv`     manually redirect serial output to .csv file:



