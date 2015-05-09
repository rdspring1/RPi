import random
import numpy

#contains class information for individual robots
class Robot:
     def __init__(self, rid, x, y, tally = 50, maxSpeed = 0.005, noise = 0.5, visibility = 0.15):
          self.rid = rid
          self.pos = [x, y]
          self.vel = [0, 0]
          self.visibility = visibility
          self.maxSpeed = maxSpeed
          self.noise = noise
          self.tally = tally
          self.rate = 0
          self.unit_dist = (2)**(1.0/2)

     def randomizePosition(self):
          self.pos = [random.random(), random.random()]

#returns list of "num" random robots
def initRobots(num):
     rlist = []
     for i in range(num):
          rlist += [Robot(i, random.random(), random.random())]
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

#moves robots in rlist uniformly randomly within bounded box
def randomStep(rlist, xlo, xhi, ylo, yhi):
     for rob in rlist:
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
