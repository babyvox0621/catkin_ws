#

import pygame.mixer
import time

# mixer
pygame.mixer.init()

#music load
pygame.mixer.music.load("kinen.wav")

#music loop
pygame.mixer.music.play(-1)

time.sleep(10)

#end
pygame.mixer.music.stop()

