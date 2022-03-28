# LEDs
Library to make shiny lights

## KLEDStrip
This is an singleton (class that should only be instantiated once) that should be put in RobotContainer.
It handles the messaging and fancy color correction.
Call update on this periodically

## KLEDRegion
This is the class to handle one section of your leds. Give it a start, end, Animation, and condition.
It will decide when and what to put on that part of the LEDs

## Animation
any function that takes current time in ticks and outputs the buffer of colors
