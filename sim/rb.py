# Scene code from http://www.nerdparadise.com/tech/python/pygame/basics/part7/
# The first half is just boiler-plate stuff...

import pygame
from pygame.locals import *
from pygame import gfxdraw
import random
import os
import datetime
import rbutils
import objects

class SceneBase:
     def __init__(self):
          self.next = self

     def ProcessInput(self, events, pressed_keys):
          raise RuntimeError("uh-oh, you didn't override this in the child class")

     def Update(self, width, height):
          raise RuntimeError("uh-oh, you didn't override this in the child class")

     def Render(self, screen):
          raise RuntimeError("uh-oh, you didn't override this in the child class")

     def SwitchToScene(self, next_scene):
          self.next = next_scene

     def Terminate(self):
          self.SwitchToScene(None)

def run_game(width, height, fps, starting_scene):
     pygame.init()
     screen = pygame.display.set_mode((width, height),RESIZABLE)
     clock = pygame.time.Clock()

     active_scene = starting_scene
     paused = False

     while active_scene != None:
          pressed_keys = pygame.key.get_pressed()

          # Event filtering
          filtered_events = []
          for event in pygame.event.get():
               quit_attempt = False
               if event.type == pygame.QUIT:
                    quit_attempt = True
               elif event.type == pygame.KEYDOWN:
                    alt_pressed = pressed_keys[pygame.K_LALT] or \
                                  pressed_keys[pygame.K_RALT]
                    if event.key == pygame.K_ESCAPE:
                         quit_attempt = True
                    elif event.key == pygame.K_F4 and alt_pressed:
                         quit_attempt = True
                    if event.key == pygame.K_SPACE:
                         paused = not paused
               elif event.type==VIDEORESIZE:
                    screen=pygame.display.set_mode(event.dict['size'],RESIZABLE)
                    pygame.display.flip()

               if quit_attempt:
                    active_scene.Terminate()
               else:
                    filtered_events.append(event)

          active_scene.ProcessInput(filtered_events, pressed_keys)
          if not paused:
               active_scene.Update(screen.get_width(), screen.get_height())
          active_scene.Render(screen)

          active_scene = active_scene.next

          pygame.display.flip()
          clock.tick(fps)

# Game Engine
class RobotScene(SceneBase):
     def __init__(self, numRobots, updater):
          SceneBase.__init__(self)

          self.robots = rbutils.initRobots(numRobots)
          self.updater = updater
          self.isPrinting = False
          self.curFile = ''
          self.objects = []

     def addObject(self, obj):
         self.objects.append(obj)

     def ProcessInput(self, events, pressed_keys):
          for event in events:
               if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                         self.show = not self.show
                    if event.key == pygame.K_o:
                         if os.path.exists("Data"):
                              self.isPrinting = not self.isPrinting
                              if self.isPrinting:
                                   updaterName = self.updater.__name__
                                   fname = updaterName + datetime.datetime.now().strftime('%y_%m_%d-%H_%M_%S')
                                   fname += '.csv'
                                   self.curFile = fname
                                   rbutils.printStartState(fname, updaterName, self.robots)
                         else:
                              print("Directory \"Data\" is missing. Create a folder in this directory called \"Data\" to save results.")
                    if event.key == pygame.K_RETURN:
                         self.isPrinting = False
                         rbutils.randomizeRobots(self.robots)

                    if event.key == pygame.K_1:
                         self.updater = rbutils.randomStep
                         self.isPrinting = False

     def Update(self, width, height):
          self.updater(self.objects, self.robots, width, height, 0, 1, 0, 1)
          if self.isPrinting:
               rbutils.printState(self.curFile,self.robots)

     def Render(self, screen):
          red = (215, 40, 60)
          aqua = (0, 140, 255)
          mint = (120, 210, 170)
          egg = (225, 235, 215)
          black = (0, 0, 0)

          robRadius = 4
          goalRadius = 6
          lineWidth = 1

          screen.fill(egg)
          #pygame.draw.aaline(screen, black, (newx, newy), (ox, oy), lineWidth)

          #draw robots
          for rr in self.robots:
               newx = int(rr.pos[0] * screen.get_width())
               newy = int(rr.pos[1] * screen.get_height())
               pygame.gfxdraw.aacircle(screen, newx, newy, robRadius, black)
               pygame.gfxdraw.filled_circle(screen, newx, newy, robRadius, red)

          #draw objects
          for obj in self.objects:
               pygame.gfxdraw.box(screen, obj.rect, aqua)

          caption = 'Simulator'
          if self.isPrinting:
               caption += ' (Printing to file)'
          pygame.display.set_caption(caption)


#randomStep is run by default
scene = RobotScene(20, rbutils.randomStep)
scene.addObject(objects.Object('A', 100, 100, 50, 50))
scene.addObject(objects.Object('B', 200, 200, 100, 50))
scene.addObject(objects.Object('C', 50, 250, 50, 75))
run_game(360, 360, 60, scene)
