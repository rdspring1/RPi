import pygame
import random
import numpy

#contains class information for individual robots
class Robot:
     def __init__(self, rid, x, y, color, sensor = 50, tally = 50, maxSpeed = 0.005, noise = 0.5, visibility = 0.15):
          self.rid = rid
          self.pos = [x, y]
          self.vel = [0, 0]
          self.sensor = sensor
          self.visibility = visibility
          self.maxSpeed = maxSpeed
          self.noise = noise
          self.tally = tally
          self.rate = 0
          self.unit_dist = (2)**(1.0/2)
          self.color = color
          self.default_color = color
          self.objectlist = []

     def randomizePosition(self):
          self.pos = [random.random(), random.random()]

     def sensorArea(self, width, height):
          offset = self.sensor / 2.0
          x = self.pos[0] * width - offset
          y = self.pos[1] * height - offset
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
               rlist += [Robot(i, pos[0], pos[1], red)]
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

#robot update function
def updateState(objects, rlist, width, height, xlo, xhi, ylo, yhi):
     randomStep(objects, rlist, width, height, xlo, xhi, ylo, yhi)
     detectObjects(objects, rlist, width, height)

def findCliques(objects, rlist, width, height, commrange = 50):
     objset = {}
     for obj in objects:
          objset[obj.name] = []
          for rob in rlist:
               if obj.name in rob.objectlist:
                    objset[obj.name].append(rob)

     cliques = dict.fromkeys(objset.keys())
     edgeset = []
     for key in objset.iterkeys():
         cliques[key] = []
         for rob in objset[key]:
             if not cliques[key]:
                  cliques[key].append([rob])
             else:
                  updateClique(key, cliques, edgeset, rob, width, height, commrange)
     return [cliques, edgeset]

def updateClique(key, cliques, edgeset, rob, width, height, commrange):
     rpos = ((rob.pos[0] * width), (rob.pos[1] * height))
     for clique in cliques[key]:
          for c in clique:
               cpos = ((c.pos[0] * width), (c.pos[1] * height))
               dist = numpy.linalg.norm(numpy.array(rpos) - numpy.array(cpos))
               if dist <= commrange:
                    clique.append(rob)
                    edgeset.append((cpos, rpos))
                    return
     cliques[key].append([rob])

#determine which objects each robot sees
def detectObjects(objects, rlist, width, height):
     for rob in rlist:
          del rob.objectlist[:]
          sensorArea = rob.sensorArea(width, height)
          rob.color = rob.default_color
          for obj in objects:
               if obj.rect.colliderect(sensorArea):
                    rob.objectlist.append(obj.name)
                    rob.color = obj.color

#moves robots in rlist uniformly randomly within bounded box
def randomStep(objects, rlist, width, height, xlo, xhi, ylo, yhi):
     for rob in rlist:
          # check object collision
          for obj in objects:
               if obj.rect.collidepoint(rob.pos[0]*width, rob.pos[1]*height):
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
