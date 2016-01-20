# in4600
MSc Thesis

In LED-energy-slow-update a program can be found which can modulate the LEDs in a couple of power readings.

The energy meter shows the power draw over a period of one second, and updates every second.
So basically it shows the area under the VA over time curve.

To be able to distinguish the LEDs, an 8 bit is used.
But because a 1 draws the same power, no matter its position in the code, only 9 codes can be used, and those may or may not be all distinguishable.
So with zero 1s all the way to eight 1s.


