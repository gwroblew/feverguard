# FeverGuard Pro

## Versions

There are two versions of the card:
 - V1 with Celsius degrees above the LEDs.
   - **IMPORTANT NOTE**: If you got the card before 2023-10-15, your version tends to under detect fever between 37-38C / 98.5-100F. Commercial thermometers tend to
over detect this kind of fever, so I updated the logic in the card.
   - You have to press and hold the button for operation.
 - V2 with Fahrenheit degrees above the LEDs.
   - It has small "V2" sign in the bottom right corner.
   - You only need to press the button once to start.
   - For repeated check, wait for a temperature LED to stop blinking.

## Operation

- Press (and hold for V1 card) the button - row of LEDs will blink back and forth.
  - If only the 41.5 / 108 red LED blinks, battery is too low.
  - If only the 41 / 107 red LED blinks, sensor does not work properly.
- Put sensor close to forehead, wait 2-3 seconds.
- Body temperature LED will blink 10 - 20 times.
  - If <35 and 35.5 / <95 and 96 LEDs blink, the detected temperature was too low.

## Building Binary

- Install ARM GCC cross-compiler.
- Run "make" in "gcc" folder.
- Run "./edbg -t atmel_cm0p -p -f AtmelStart.bin -b -v" to program.
