# README.md for Micromerge / Microtaur Project





### platformio terminal commands (for reference)
`pio device monitor`                    # open serial monitor (uses baud rate specified by monitor_speed)
`pio device list`                       show the available serial ports
`pio device monitor > data_log.csv`     manually redirect serial output to .csv file:

test files located in /src/tests/ (files with setup/loop structure not intended for final code) 
to run those: in platformio.ini comment out the line `build_src_filter = -<src/tests/>`
and add a line `build_src_filter = -<main.cpp> +<tests/filename.cpp>`


maze code outputs:
* coordinates of current cell    ex: (3,2)
* direction

