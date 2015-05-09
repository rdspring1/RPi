import pygame
from pygame import gfxdraw

class Object:
     def __init__(self, name, x, y, width, height):
          self.name = name
          self.rect = pygame.Rect(x,y,width,height)
