# Bitbanged I2C for Raspberry Pi

This is a bit-banged I2C driver in user-space, working on the Raspberry Pi 3.

It relies on the BCM2835 library to toggle the GPIO pins (but does not use its I2C functions).

The main file (i2c.c) can be compiled. It scans four addresses and reports whether its those addresses were ACK'd.

### Limitations

- The driver does not work for 16-bit+ sizes of data.
- It is somewhat flaky.

### Good to know

- A lot of this implementation came from reading over https://github.com/electronicayciencia/wPi_soft_i2c. Huge kudos to them and their writeup!
