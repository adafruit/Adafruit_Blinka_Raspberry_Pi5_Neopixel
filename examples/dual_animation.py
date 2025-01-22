
import adafruit_pixelbuf
import board
from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_led_animation.animation.rainbowchase import RainbowChase
from adafruit_led_animation.animation.rainbowcomet import RainbowComet
from adafruit_led_animation.animation.rainbowsparkle import RainbowSparkle
from adafruit_led_animation.sequence import AnimationSequence
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

def make_animation(pixels):
    rainbow = Rainbow(pixels, speed=0.02, period=2)
    rainbow_chase = RainbowChase(pixels, speed=0.02, size=5, spacing=3)
    rainbow_comet = RainbowComet(pixels, speed=0.02, tail_length=7, bounce=True)
    rainbow_sparkle = RainbowSparkle(pixels, speed=0.02, num_sparkles=15)


    animations = AnimationSequence(
        rainbow,
        rainbow_chase,
        rainbow_comet,
        rainbow_sparkle,
        advance_interval=5,
        auto_clear=True,
        random_order=True,
    )
    return animations

animation1 = make_animation(pixels1)
animation2 = make_animation(pixels2)

try:
    while True:
        animation1.animate()
        animation2.animate()
finally:
    pixels1.fill(0)
    pixels1.show()
    pixels2.fill(0)
    pixels2.show()
