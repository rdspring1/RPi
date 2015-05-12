import pygame
import random
import numpy
import collections
import datetime
import rbutils
import sys

class Message:
     def __init__(self, rid, name, position = [0, 0]):
          self.rid = rid
          self.objectID = name
          self.position = position

class Action:
    Explore, Track, Found = range(3)

# Explore
# Track
# Broadcast - Object Found
def imageSharing(objects, rlist, xlo, xhi, ylo, yhi, threshold = 3):
     for rob in rlist:
          if rob.action == Action.Explore:
               randomMovement(objects, rob, xlo, xhi, ylo, yhi)
          elif rob.action == Action.Track:
               trackObject(objects, rob, xlo, xhi, ylo, yhi)

     # Update Neighbor List
     updateNeighbors(rlist)
     detectObjects(objects, rlist)

     # Process Messages - Add object to found list if message threshold is exceeded
     processMessages(rlist, threshold)

     complete = True
     for rob in rlist:
          # Check Termination Condition - Action for every robot is found
          num_objects = len(rob.objectList | rob.sharedObjects)
          if num_objects != len(objects):
               #print str(rob.rid) + " : " + str(numobjects)
               complete = False

          # Decide action for each robot
          rob.action = Action.Explore
          if len(rob.objectList) > 0:
               for obj in rob.trackList:
                    if rob.trackList[obj]:
                         rob.action = Action.Found

          if (rob.action != Action.Found) and (rob.goalObject != None):
               rob.action = Action.Track

          if rob.action != Action.Track:
               rob.goalObject = None

     # Pass communication messages - Pass information about objects directly found by robot
     for rob in rlist:
          for other in rob.neighborList:
               for object_name in rob.objectList:
                    other.messageQueue.append(Message(rob.rid, object_name, objects[object_name].rect.center))
     return complete

#robot update function
def binaryObject(objects, rlist, xlo, xhi, ylo, yhi, threshold = 3):
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
          num_objects = len(rob.objectList | rob.sharedObjects)
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
          object_list = {}
          object_position = {}
          for msg in rob.messageQueue:
               if msg.objectID in object_list:
                    object_list[msg.objectID].append(msg.rid)
                    object_position[msg.objectID] = msg.position
               else:
                    object_list[msg.objectID] = [msg.rid]
          del rob.messageQueue[:]

          potentialObjects = []
          rob.sharedObjects.clear()
          for obj in object_list:
               if len(object_list[obj]) >= threshold:
                    rob.sharedObjects.add(obj)
               else:
                    potentialObjects.append(object_position[obj])
          findGoal(rob, potentialObjects)

          #determine if this robot should track object
          rob.trackList.clear()
          for obj in rob.objectList:
               if obj in object_list and len(object_list[obj]) > (threshold+1):
                    rob.trackList[obj] = rob.rid < sorted(object_list[obj])[threshold]
               else:
                    rob.trackList[obj] = True

def findGoal(rob, potentialObjects):
     rpos = numpy.array(rob.screenPosition())
     minDist = sys.float_info.max
     for obj in potentialObjects:
          new_direction = numpy.array(obj) - rpos
          dist = numpy.linalg.norm(new_direction)
          if dist < minDist:
               rob.goalObject = numpy.array(obj)
               minDist = dist

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
          rob.objectList.clear()
          rob.color = rob.default_color
          for key, obj in objects.iteritems():
               if obj.rect.colliderect(rob.sensorArea()):
                    rob.objectList.add(obj.name)
                    rob.color = obj.color

def collisionCheck(objects, rob):
     # check object collision
     for key, obj in objects.iteritems():
          if obj.rect.collidepoint(rob.screenPosition()):
               rob.vel[0] *= -1.0
               rob.vel[1] *= -1.0
               rob.rate = 0
               return True
     return False

def positionBounds(rob, xlo, xhi, ylo, yhi):
     if rob.pos[0] < xlo: rob.pos[0] = xlo
     if rob.pos[0] > xhi: rob.pos[0] = xhi
     if rob.pos[1] < ylo: rob.pos[1] = ylo
     if rob.pos[1] > yhi: rob.pos[1] = yhi

#moves robots in rlist uniformly randomly within bounded box
def randomMovement(objects, rob, xlo, xhi, ylo, yhi):
     collisionCheck(objects, rob)

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
     positionBounds(rob, xlo, xhi, ylo, yhi)

def trackObject(objects, rob, xlo, xhi, ylo, yhi):
     collision = collisionCheck(objects, rob)

     if not collision:
          rpos = numpy.array(rob.screenPosition())
          direction = rob.goalObject - rpos
          direction /= numpy.linalg.norm(direction)
          rob.vel = direction.tolist()

     # update robots position
     xprime = rob.pos[0] + rob.vel[0] * rob.maxSpeed
     yprime = rob.pos[1] + rob.vel[1] * rob.maxSpeed
     rob.pos = [xprime, yprime]
     positionBounds(rob, xlo, xhi, ylo, yhi)
