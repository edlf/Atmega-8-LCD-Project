# ATMega8 LCD Project

A text editor implemented in C for Microcontrollers course

Uses a Batron  BTHQ96040 Graphic LCD, featuring a spectacular 96 x 40 resolution without any colors.
For the brains it uses an ATMega8 with 8kb of program space (1,5kb already taken by the boot loader) and 1kb of ram.

Based on the work by https://sites.google.com/site/donutscience/programming-hacks/graphic-lcd-library-for-avr-barton-and-other. Original source code included.

Code adapted and trimmed to fit in 6,5kb with the added functionality.

A frame buffer was added that fits within the available 1kb of ram.

For even more ram space savings the fonts are read directly from flash.

Since the original code is under LGPL v3 or later. The same license applies.