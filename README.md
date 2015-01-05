This is a simple egg timer for the STM32L-DISCOVERY and the 32L152CDISCOVERY
boards:

http://www.st.com/st-web-ui/static/active/en/resource/technical/document/data_brief/DM00027566.pdf

After power on or reset, it counts down from a hard coded time (1 hour by
default). When the time reaches zero, the two LEDs start flashing and the time
starts counting upwards. Hours, minutes and seconds are shown on the LCD. The
USER button delays the countdown by adding a hard coded amount of time (15
minutes by default).

RTC interrupts are used for decreasing the time every second.

The touch slider / 4 touch buttons are not used.

The project compiles on Linux using the GCC open source toolchain and is flashed
to the board using Texane's stlink software: https://github.com/texane/stlink.

The seconds are stored in a volatile variable that gets updated by the RTC
interrupt. In main, a tight loop runs continously that checks the number of
seconds remaining and flashes the LEDs if it's negative. This is not a power
efficient solution. To adjust this project for battery operated usage, the LEDs
should also be flashed using interrupts and the device should be put to sleep
between interrupts.

This project is based on the following project on GitHub:

https://github.com/tomvdb/stm32l1-discovery-basic-template

See the instructions there on how to set up an open source development
environment on Linux. When things are set up properly, build and flash with:

make flash
