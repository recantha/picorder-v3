import pygame
import time

pygame.init()
pygame.mixer.init()
scan = pygame.mixer.Sound("tricorder.wav")
scan.set_volume(1.0)

scan.play(-1)

while True:
	print "Playing"
