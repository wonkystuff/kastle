#!/bin/sh

read -n 1 -s -r -p "Press a key to program DRUM LFO"
./prog.sh kastleDRUM_CLK_LFO/kastleDRUM_CLK_LFO.ino.hex

# Use the hex file from the Bastl production repo: https://github.com/bastl-instruments/production/tree/master/attiny
read -n 1 -s -r -p "Press a key to program ARP Oscillator"
./prog.sh kastleArp/kastleArp.hex
