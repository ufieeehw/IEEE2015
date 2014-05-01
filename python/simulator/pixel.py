#! /usr/bin/env python

import pygame
import random
#dimensions
width = 640
height = 400

bgcolor = 192, 192, 192
linecolor = 255, 0, 0

screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True

last_position = (0,0)
while running:

    dx = int(input("Enter change of angle nob 1: "))
    dy = int(input("Enter change of angle nob 2: "))
    end_position = (last_position[0]+dx, last_position[1]+dy)

    #screen.set_at((x,y), linecolor)
    pygame.draw.line(screen, linecolor, last_position, end_position, 1)
    last_position = end_position;


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(240)

