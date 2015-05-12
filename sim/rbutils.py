import pygame
import random
import numpy
import collections
import datetime

class Message:
     def __init__(self, rid, name, position = [0, 0]):
          self.rid = rid
          self.objectID = name
          self.position = position

class Action:
    Explore, Track, Found = range(3)

#contains class information for individual robots
class Robot:
     def __init__(self, rid, x, y, sw, sh, color):
          # constants
          self.commrange = 300
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
          self.objectList = []
          self.sharedObjects = []
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
          for obj in objects:
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

#add jitter to remove smoothness
def addNoise(rob, speed, nval):
     rob.pos[0] += (2 * random.random() - 1) * speed * nval
     rob.pos[1] += (2 * random.random() - 1) * speed * nval

# Explore
# Track
# Broadcast - Object Found
# After Threshold - Object Location

#robot update function
def updateState(objects, rlist, xlo, xhi, ylo, yhi, threshold = 3):
     for rob in rlist:
          if rob.action == Action.Explore:
               randomMovement(objects, rob, xlo, xhi, ylo, yhi)

     # Update Neighbor List
     updateNeighbors(rlist)
     detectObjects(objects, rlist)

     # Process Messages - Add object to found list if message threshold is exceeded
     processMessages(rlist, threshold)

     complete = True
     for rob in rlist:
          # Check Termination Condition - Action for every robot is found
          num_objects = len(set(rob.objectList) | set(rob.sharedObjects))
          if num_objects != len(objects):
               #print str(rob.rid) + " : " + str(numobjects)
               complete = False

          # Decide action for each robot
          rob.action = Action.Explore
          if len(rob.objectList) > 0:
               for obj in rob.trackList:
                    if rob.trackList[obj]:
                         rob.action = Action.Found


     # Pass communication messages - Pass information about objects directly found by robot
     for rob in rlist:
          for other in rob.neighborList:
               for object_name in rob.objectList:
                    other.messageQueue.append(Message(rob.rid, object_name))
     return complete

def processMessages(rlist, threshold):
     for rob in rlist:
          # determine if neigbors detect other objects
          del rob.sharedObjects[:]
          object_list = {}
          for msg in rob.messageQueue:
               if msg.objectID in object_list:
                    object_list[msg.objectID].append(msg.rid)
               else:
                    object_list[msg.objectID] = [msg.rid]
          del rob.messageQueue[:]
          for obj in object_list:
               if len(object_list[obj]) >= threshold:
                    rob.sharedObjects.append(obj)

          #determine if this robot should track object
          rob.trackList.clear()
          for obj in rob.objectList:
               if obj in object_list and len(object_list[obj]) > (threshold+1):
                    rob.trackList[obj] = rob.rid < sorted(object_list[obj])[threshold]
               else:
                    rob.trackList[obj] = True

def updateNeighbors(rlist):
     index = 0
     for rob in rlist:
          del rob.neighborList[:]
     for rob in rlist:
          for other in range(index, len(rlist)):
               if rob.rid != rlist[other].rid:
                    dist = numpy.linalg.norm(numpy.array(rob.screenPosition()) - numpy.array(rlist[other].screenPosition()))
                    if dist <= rob.commrange:
                         rob.neighborList.append(rlist[other])
                         rlist[other].neighborList.append(rob)

#determine which objects each robot sees
def detectObjects(objects, rlist):
     for rob in rlist:
          del rob.objectList[:]
          rob.color = rob.default_color
          for obj in objects:
               if obj.rect.colliderect(rob.sensorArea()):
                    rob.objectList.append(obj.name)
                    rob.color = obj.color

#moves robots in rlist uniformly randomly within bounded box
def randomMovement(objects, rob, xlo, xhi, ylo, yhi):
     # check object collision
     for obj in objects:
          if obj.rect.collidepoint(rob.screenPosition()):
               rob.vel[0] *= -1.0
               rob.vel[1] *= -1.0
               rob.rate = 0

     # change motion direction randomly
     if rob.rate == rob.tally:
          rob.rate = 0
          rob.vel[0] = (2*random.random() - 1)
          rob.vel[1] = (2*random.random() - 1)
          rob.vel[0] /= rob.unit_dist
          rob.vel[1] /= rob.unit_dist
     else:
          rob.rate += 1

     # update robots position
     xprime = rob.pos[0] + rob.vel[0] * rob.maxSpeed
     yprime = rob.pos[1] + rob.vel[1] * rob.maxSpeed
     rob.pos = [xprime, yprime]

     if rob.pos[0] < xlo: rob.pos[0] = xlo
     if rob.pos[0] > xhi: rob.pos[0] = xhi
     if rob.pos[1] < ylo: rob.pos[1] = ylo
     if rob.pos[1] > yhi: rob.pos[1] = yhi

def findCliques(objects, rlist):
     objset = {}
     for obj in objects:
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
