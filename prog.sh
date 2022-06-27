#!/bin/sh

DEVICE="attiny85"
CLOCK="8000000"
AVRDUDE="/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avrdude"
AVRDUDECONF="/Applications/Arduino.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf"

$AVRDUDE -B 10 -P usb -c usbasp -p $DEVICE -C $AVRDUDECONF -v -U lfuse:w:0xC2:m -U hfuse:w:0xD7:m -U efuse:w:0xFF:m  -U flash:w:$1
