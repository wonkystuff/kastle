#!/bin/sh

read -n 1 -s -r -p "Press a key to program DRUM LFO"
./prog.sh kastleDRUM_CLK_LFO/kastleDRUM_CLK_LFO.ino.hex

read -n 1 -s -r -p "Press a key to program DRUM Oscillator"
./prog.sh kastleDrum/kastleDrum.ino.hex
