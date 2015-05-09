import random

#contains class information for individual robots
class Robot:
     def __init__(self, rid, x, y, maxSpeed = 0.005, noise = 0.5, vis = 0.15, goalDist = 0.01):
          self.rid = rid
          self.x = x
          self.y = y
          self.vis = vis
          self.maxSpeed = maxSpeed
          self.noise = noise
          self.nearSame = []
          self.nearOther = []
          #assume goal list is in order of goals to visit
          self.goalList = []
          self.goalDist = goalDist
          
     def addGoal(self, loc):
          self.goalList += [loc]
          
     def setGoalList(self, gl):
          self.goalList = gl
          
     def getGoalList(self):
          return self.goalList
          
     def clearGoals(self):
          self.goalList = []
          
     def randomizePosition(self):
          self.x = random.random()
          self.y = random.random()
          
class Goal:
     def __init__(self, x, y):
          self.x = x
          self.y = y

#return distance between two robots
def dist(r1, r2):
     return ((r1.x - r2.x)**2 + (r1.y - r2.y)**2)**(1.0/2)

#returns list of "num" random robots     
def initRobots(num):
     rlist = []
     for i in range(num):
          rlist += [Robot(i, random.random(), random.random())]
     return rlist

#define goals for list of robots
#this is currently hard coded     
def setGoals(rlist):
     goalList = [Goal(0.1, 0.1),
                 Goal(0.9, 0.9),
                 Goal(0.1, 0.1),
                 Goal(0.1, 0.9),
                 Goal(0.1, 0.1),
                 Goal(0.9, 0.1)]
     for rob in rlist:
          start = int(random.random() * len(goalList))
          rob.setGoalList(goalList[start:] + goalList[:start])

#prints particular robot information
def printRobots(rlist):
     output = ''
     for rob in rlist:
          output += str(rob.rid) + ','
          output += str(rob.x) + ','
          output += str(rob.y) + ','
          output += str(rob.vis) + ','
          output += str(rob.maxSpeed) + '\n'
          
          output += str(len(rob.nearSame)) + ','         
          sameOut = ''
          for same in rob.nearSame:
               sameOut += str(same.rid) + ','
               #sameOut += str(same.x) + ','
               #sameOut += str(same.y) + ','
          output += sameOut[:-1] + '\n'
          
          output += str(len(rob.nearOther)) + ','
          otherOut = ''
          for other in rob.nearOther:
               otherOut += str(other.rid) + ','
               #otherOut += str(other.x) + ','
               #otherOut += str(other.y) + ','
          output += otherOut[:-1] + '\n'
     return output
          
#print information about each robot
def printState(fname, blueList, redList):
     output = ''
     output += 'blue\n'
     output += printRobots(blueList)
     output += 'red\n'
     output += printRobots(redList)
     f = open("Data/" + fname, "a")
     f.write(output)
     f.close()        
          
#print information about each robot and config information
def printStartState(fname, updater, blueList, redList):
     f = open("Data/" + fname, "a")
     f.write(updater + ',' + 'blue,' + str(len(blueList)) + ',red,' + str(len(redList)) + '\n')
     f.close()
     printState(fname, blueList, redList)

     
#randomizes position of each robot in list  
def randomizeRobots(rlist):
     for rob in rlist:
          rob.randomizePosition()
          
#increases or decreases visibility     
def changeVis(rlist, d):
     newList = []
     for rob in rlist:
          rob.vis += d
          newList += [rob]
     return newList
     
#maintains list of nearest neighbors
def updateNearestNeighbors(blueRobots, redRobots):
     #find nearest blues to blue
     for blue in blueRobots:
          blue.nearSame = []
          for other in blueRobots:
               if other != blue and dist(blue, other) < blue.vis:
                    blue.nearSame += [other]
     #find nearest reds to blue
     for blue in blueRobots:
          blue.nearOther = []
          for other in redRobots:
               if dist(blue, other) < blue.vis:
                    blue.nearOther += [other]
     #find nearest reds to red
     for red in redRobots:
          red.nearSame = []
          for other in redRobots:
               if other != red and dist(red, other) < red.vis:
                    red.nearSame += [other]

#add jitter to remove smoothness                    
def addNoise(rob, speed, nval):
     rob.x += (2 * random.random() - 1) * speed * nval
     rob.y += (2 * random.random() - 1) * speed * nval

#moves robots in rlist uniformly randomly within bounded box
def randomStep(rlist, xlo, xhi, ylo, yhi):
     for rob in rlist:
          dx = (2*random.random() - 1)
          dy = (2*random.random() - 1)
          dist = (2)**(1.0/2)
          dx /= dist
          dy /= dist
          rob.x += dx * rob.maxSpeed
          rob.y += dy * rob.maxSpeed
          
          if rob.x < xlo: rob.x = xlo
          if rob.x > xhi: rob.x = xhi
          if rob.y < ylo: rob.y = ylo
          if rob.y > yhi: rob.y = yhi

#moves robots towards goals in order          
def resourceCollector(rlist, xlo, xhi, ylo, yhi):
     for rob in rlist:
          curGoal = rob.goalList[0]
          if dist(rob, curGoal) < rob.goalDist:
               rob.goalList = rob.goalList[1:] + [rob.goalList[0]]
          dx = curGoal.x - rob.x
          dy = curGoal.y - rob.y
          maxDist = (dx * dx + dy * dy)**(1.0/2)
          dx /= maxDist
          dy /= maxDist
          rob.x += dx * rob.maxSpeed
          rob.y += dy * rob.maxSpeed
          addNoise(rob, rob.maxSpeed, rob.noise)
          
          if rob.x < xlo: rob.x = xlo
          if rob.x > xhi: rob.x = xhi
          if rob.y < ylo: rob.y = ylo
          if rob.y > yhi: rob.y = yhi

#moves robots away from each other     
def disperse(rlist, xlo, xhi, ylo, yhi):
     for rob in rlist:
          if len(rob.nearSame) > 0:
               cenx = sum(near.x for near in rob.nearSame) / len(rob.nearSame)
               ceny = sum(near.y for near in rob.nearSame) / len(rob.nearSame)
               dx = cenx - rob.x
               dy = ceny - rob.y
               maxDist = (dx * dx + dy * dy)**(1.0/2)
               if maxDist > 0:
                    dx /= maxDist
                    dy /= maxDist
                    rob.x -= dx * rob.maxSpeed
                    rob.y -= dy * rob.maxSpeed
                    #addNoise(rob, rob.maxSpeed, rob.noise)
                    
               if rob.x < xlo: rob.x = xlo
               if rob.x > xhi: rob.x = xhi
               if rob.y < ylo: rob.y = ylo
               if rob.y > yhi: rob.y = yhi
               
          
