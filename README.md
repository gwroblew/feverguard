# FeverGuard Pro

## Operation

- Press and hold the button - row of LEDs will blink back and forth.
  - If only the 41.5 red LED blinks, battery is too low.
  - If only the 41 red LED blinks, sensor does not work properly.
- Put sensor close to forehead, wait 2-3 seconds.
- Body temperature LED will blink 20 times.
  - If <35 and 35.5 LEDs blink, the detected temperature was too low.

## Building Binary

- Install ARM GCC cross-compiler.
- Run "make" in "gcc" folder.
- Run "./edbg -t atmel_cm0p -p -f AtmelStart.bin -b -v" to program.
