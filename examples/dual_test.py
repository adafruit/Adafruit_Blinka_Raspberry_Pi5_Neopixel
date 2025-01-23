# Test for https://github.com/adafruit/Adafruit_Blinka_Raspberry_Pi5_Neopixel/issues/3
# With the bug NOT fixed, some of the pixels would be corrupted.
#
# This corruption wasn't seen (or wasn't seen as much) if the Python program
# had any sleeps between updates, such as the dual_animation example. Thus,
# a dedicated reproducer for the bug is desired.

import time

import adafruit_pixelbuf
import board
from adafruit_raspberry_pi5_neopixel_write import neopixel_write

NEOPIXEL1 = board.D13
NEOPIXEL2 = board.D12
num_pixels = 96

class Pi5Pixelbuf(adafruit_pixelbuf.PixelBuf):
    def __init__(self, pin, size, **kwargs):
        self._pin = pin
        super().__init__(size=size, **kwargs)

    def _transmit(self, buf):
        neopixel_write(self._pin, buf)

pixels1 = Pi5Pixelbuf(NEOPIXEL1, num_pixels, auto_write=True, byteorder="BGR")
pixels2 = Pi5Pixelbuf(NEOPIXEL2, num_pixels, auto_write=True, byteorder="BGR")

while True:
    pixels1.fill(0x1)
    pixels2.fill(0x100)
    time.sleep(.1)

    pixels1.fill(0x100)
    pixels2.fill(0x10000)
    time.sleep(.1)
