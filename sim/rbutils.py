import pygame
import random
import numpy
import collections
import datetime
from quorum import Action

#contains class information for individual robots
class Robot:
     def __init__(self, rid, x, y, sw, sh, color):
          # constants
          self.commrange = 150
          self.sensor = 50
          self.visibility = 0.15
          self.maxSpeed = 0.005
          self.noise = 0.5
          self.tally = 50
          self.rate = 0
          self.unit_dist = (2)**(1.0/2)
          self.default_color = color
          self.base_time = datetime.datetime(2000,1,1,0,0,0)

          # parameters
          self.rid = rid
          self.pos = [x, y]
          self.dim = [sw, sh]
          self.vel = [0, 0]
          self.color = color
          self.objectList = set() # objects found by this robot
          self.sharedObjects = set() # confirmed objects found by other robots
          self.goalObject = None # object discovered by other robots and tracked by this robot
          self.neighborList = []
          self.messageQueue = []
          self.trackList = {}
          self.action = Action.Explore

     def screenPosition(self):
          return (self.pos[0] * self.dim[0], self.pos[1] * self.dim[1])

     def randomizePosition(self):
          self.pos = [random.random(), random.random()]

     def sensorArea(self):
          offset = self.sensor / 2.0
          x = self.pos[0] * self.dim[0] - offset
          y = self.pos[1] * self.dim[1] - offset
          return pygame.Rect(x, y, self.sensor, self.sensor)

#returns list of "num" random robots
def initRobots(objects, width, height, num):
     red = (215, 40, 60)
     rlist = []
     i = 0
     while i < num:
          pos = [random.random(), random.random()]
          real_pos = (pos[0] * width, pos[1] * height)
          add = True
          for key, obj in objects.iteritems():
               if obj.rect.collidepoint(real_pos):
                    add = False
                    break
          if add:
               rlist.append(Robot(i, pos[0], pos[1], width, height, red))
               i += 1
     return rlist

#prints particular robot information
def printRobots(rlist):
     output = ''
     for rob in rlist:
          output += str(rob.rid) + ','
          output += str(rob.pos[0]) + ','
          output += str(rob.pos[1]) + ','
          output += str(rob.visibility) + ','
          output += str(rob.maxSpeed) + '\n'
     return output

#print information about each robot
def printState(fname, List):
     output = ''
     output += 'robots\n'
     output += printRobots(List)
     f = open("Data/" + fname, "a")
     f.write(output)
     f.close()

#print information about each robot and config information
def printStartState(fname, updater, List):
     f = open("Data/" + fname, "a")
     f.write(updater + ',' + 'robots,' + str(len(List)) + '\n')
     f.close()
     printState(fname, List)

#randomizes position of each robot in list
def randomizeRobots(rlist):
     for rob in rlist:
          rob.randomizePosition()

#increases or decreases visibility
def changeVis(rlist, d):
     newList = []
     for rob in rlist:
          rob.visibility += d
          newList += [rob]
     return newList

def findCliques(objects, rlist):
     objset = {}
     for key, obj in objects.iteritems():
          objset[obj.name] = []
          for rob in rlist:
               if obj.name in rob.objectList:
                    objset[obj.name].append(rob)

     cliques = dict.fromkeys(objset.keys())
     edgeset = []
     for key in objset.iterkeys():
         cliques[key] = []
         while objset[key]:
              cliques[key].append([objset[key].pop()])
              clique = cliques[key][len(cliques[key])-1]
              added = True

              while added:
                   added = False
                   for rob in clique:
                        rpos = rob.screenPosition()
                        for c in objset[key]:
                             dist = numpy.linalg.norm(numpy.array(rpos) - numpy.array(c.screenPosition()))
                             if dist <= rob.commrange:
                                  objset[key].remove(c)
                                  clique.append(c)
                                  edgeset.append((c.screenPosition(), rpos))
                                  added = True
              #print key + str(len(cliques[key])) + "-" + str(len(clique))
     return [cliques, edgeset]
