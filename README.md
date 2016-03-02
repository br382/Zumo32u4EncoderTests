Zumo32u4EncoderTests

Implementing position counting without Zumo libraries,
but using the Arudino IDE.

NOTE:
Currently not working using single pin-interrupt service routines.
This causes pin8, a non-interrupt pin, to not allow left track counting.

By using pin bus interrupts to cover normally non-interrupt pins,
the Zumo can interrupt on pin8 which is a encoder change detection input.

This is a 32u4 based (Leonardo class) microcontroller.
