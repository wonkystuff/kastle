#!/bin/sh

read -n 1 -s -r -p "Press a key to program KASTLE LFO"
./prog.sh kastleSynth_LFO/kastleSynth_LFO.ino.hex 

read -n 1 -s -r -p "Press a key to program KASTLE Oscillator"
./prog.sh kastleSynthe_VCO_2/kastleSynthe_VCO_2.ino.hex
