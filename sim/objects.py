import pygame
from pygame import gfxdraw

class Object:
     def __init__(self, name, color, x, y, width, height):
          self.name = name
          self.color = color
          self.rect = pygame.Rect(x,y,width,height)
