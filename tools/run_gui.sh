#!/bin/bash
# Launch the BMI/BMP monitor GUI detached from any terminal,
# so it survives terminal sessions closing.
cd /home/pi/Desktop/STM32FC || exit 1
exec python3 tools/gui.py
