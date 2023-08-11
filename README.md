# Servo2Stepper

This is a simple program that drives the position of a stepper motor based on a servo-style PWM input signal.

The code is written for a 'ProMicro' 16 MHz 32u4 breakout board, with an A4988 stepper driver, a cheap servo tester, and a KY-024 Hall Effect sensor board. Wiring info is included.

Libraries required:
'FastAccelStepper'
'FIR' - Finite Impulse Response filter to reduce input noise from the servo tester. See https://fiiir.com/ to generate your own coefficients.
