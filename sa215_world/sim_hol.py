import sys
import os
# sim_hol.py - control file for assignment 1 world

# Check with "locate playerc.py"
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')

import math
import random
from playerc import *

## PARAMETERS ##
T = 120 # time duration
do1 = 4
do2 = 8
do3 = 30
dr1 = 12.5
dr2 = 20

## Get distance between two objects
def getDist(v, o):
  x = o.px - v.px
  y = o.py - v.py
  return math.hypot(x, y)

## Calculate attraction force between two objects
def attract(v, o, d):
  # Force field
  force = 0 # assume too far away
  if d < do1:
    force = (d*(1/do1)) - 1 # slope = 1/do1, (0, -1)
  elif d < do2:
    force = (d*(1/(do2-do1))) - 1  # slope = 1/(do2-do1), (do1, 0), (do2, 1)
  elif d < do3:
    force = 1 # ideal tracking distance
#  elif d < do4: # predictive tracking
#    force = (d*(1/(do4-do3))) + x

  # calculate direction
  x = o.px - v.px
  y = o.py - v.py
  rad = math.atan2(y,x)

  # return x,y force components
  return [force*math.cos(rad), force*math.sin(rad)]

## Calculate repulsion force between two objects
def repulse(posProxy, v):
  frep = [0, 0] # net force field (repulsion)
  for i in range(len(posProxy)):
    if i == v: # skip current robot
      continue
    d = getDist(posProxy[v], posProxy[i])
    force = 0 # too far by default
    if d < dr1:
      force = -1 # too close
    elif d < dr2:
      force = (d*(1/(dr2-dr1))) - (dr2/(dr2-dr1)) # slope = 1/(dr2-dr1), (dr1, -1), (dr2, 0)

    # calculate direction
    x = posProxy[i].px - posProxy[v].px
    y = posProxy[i].py - posProxy[v].py
    rad = math.atan2(y,x)
    frep = [frep[0]+force*math.cos(rad), frep[1]+force*math.sin(rad)]

  # return x,y force components
  return frep

## Calculate Velocity and angle from x and y- components
def calcVel(force):
  # return force vector and angle/heading
  return [math.hypot(force[0],force[1]), math.atan2(force[1],force[0])]

## Calculate intelligent x and y-components of force for free roaming
def freeRoam(v, roam):
  # circle around the middle of the map
  r = (R*1.0)/5
  pt = (R*1.0)/1.25

  if roam==1:
    if math.hypot(v.px-0, v.py-pt) < r:
      x = -pt
      y = 0
      roam = 2
    else:
      x = 0
      y = pt
  elif roam == 2:
    if math.hypot(v.px+pt, v.py-0) < r:
      x = 0
      y = -pt
      roam = 3
    else:
      x = -pt
      y = 0
  elif roam == 3:
    if math.hypot(v.px-0, v.py+pt) < r:
      x = pt
      y = 0
      roam = 4
    else:
      x = 0
      y = -pt
  elif roam == 4:
    if math.hypot(v.px-pt, v.py-0) < r:
      x = 0
      y = pt
      roam = 1
    else:
      x = pt
      y = 0
  else:
    if math.hypot(v.px-pt, v.py-0) < r:
      x = 0
      y = pt
      roam = 1
    elif math.hypot(v.px-0, v.py-pt) < r:
      x = 0
      y = pt
      roam = 2
    elif math.hypot(v.px+pt, v.py-0) < r:
      x = 0
      y = -pt
      roam = 3
    else: # assign this case as default (math.hypot(v.px-0, v.py+pt) < r)
      x = pt
      y = 0
      roam = 4

  # calculate direction
  fx = x - v.px
  fy = y - v.py
  rad = math.atan2(fy,fx)

  return [2.0*math.cos(rad), 2.0*math.sin(rad), roam]


#####################################
########        MAIN       ##########
#####################################

# User inputs
if len(sys.argv) < 4:
  print 'Error: Please enter arguements n, m, and R.'
  exit()
else:
  print 'n = ', sys.argv[1]
  print 'm = ', sys.argv[2]
  print 'R = ', sys.argv[3]

n = int(sys.argv[1]) # num of holonomic robots
m = int(sys.argv[2]) # num of targets
R = int(sys.argv[3]) # Radius of world in meters
random.seed() # initialize random seed

### CLIENTS AND PROXIES ###

# Create client objects
robot_rob = playerc_client(None, 'localhost', 6665)
robot_tgt = playerc_client(None, 'localhost', 6666)

# Connect them to the server
if robot_rob.connect() !=0:
	raise playerc_error_str()
if robot_tgt.connect() !=0:
	raise playerc_error_str()

# Internal Variables/Arrays
posProxy_rob = []
rangerProxy_rob = []
posProxy_tgt = []
blobProxy_tgt = []
vel_tgt =[] # array of fixed target speeds
# array of robots force (x,y) vector summation
fnet = [[0 for x in range(2)] for y in range(m)] # net force on robot

# Simulation proxy
simProxy = playerc_simulation(robot_rob, 0)
if simProxy.subscribe(PLAYERC_OPEN_MODE) != 0:
  raise playerc_error_str()

# Robot proxies
for i in range(m):
  # Position2d
  posProxy_rob.append(playerc_position2d(robot_rob,i))
  if posProxy_rob[i].subscribe(PLAYERC_OPEN_MODE) !=0:
    raise playerc_error_str()
  # Ranger
  #rangerProxy_rob.append(playerc_ranger(robot_rob,i))
  #if rangerProxy_rob[i].subscribe(PLAYERC_OPEN_MODE) !=0:
    #raise playerc_error_str()

# Target proxies
for i in range(n):
  # Position2d
  posProxy_tgt.append(playerc_position2d(robot_tgt,i))
  if posProxy_tgt[i].subscribe(PLAYERC_OPEN_MODE) !=0:
    raise playerc_error_str()
  vel_tgt.append(random.random()*1.5)
  # Blobfinder
  blobProxy_tgt.append(playerc_blobfinder(robot_tgt,i))
  if blobProxy_tgt[i].subscribe(PLAYERC_OPEN_MODE) !=0:
    raise playerc_error_str()


### SETUP ###

# Set robotis initial position randomly
for i in range(m):
  name = "rob" + str(i)
  radius = random.randint(0, R-2)
  angle = random.random()*2*math.pi
  simProxy.set_pose2d(name, radius*math.cos(angle), radius*math.sin(angle), 0)

# Set targets initial position randomly
for i in range(n):
  name = "tgt" + str(i)
  radius = random.randint(0, R-2)
  angle = random.random()*2*math.pi
  simProxy.set_pose2d(name, radius*math.cos(angle), radius*math.sin(angle), 0)

###### Data Processing and Control ######
count = 0
num_tgt_observed = 0.0
rob_tgt = [0]*m # bool to identify no targets found
roaming = [0]*m # bool set to 1 when rob is roaming

# dt loop
for i in range(T):
  # Read new data from server
  if robot_rob.read() == None:
    raise playerc_error_str()
  if robot_tgt.read() == None:
    raise playerc_error_str()

  # Attraction to targets
  for o in range(n):
    # Move the targets randomly
    tgt = posProxy_tgt[o]
    px_est = tgt.px + tgt.vx
    py_est = tgt.py + tgt.vy
    if ((count % 10) == 0) or (math.hypot(px_est, py_est) >= 100):
      rad = random.random() * 2 * math.pi
      while math.hypot(tgt.px + vel_tgt[o]*math.cos(rad), \
			tgt.py + vel_tgt[o]*math.sin(rad)) >= (R-2):
        rad = random.random() * 2 * math.pi
      #posProxy_tgt[o].set_cmd_vel(vel_tgt[o], 0.0, rad, 1)
      tgt.set_cmd_vel(vel_tgt[o]*math.cos(rad), vel_tgt[o]*math.sin(rad), \
			0.0, 1)

    # Count observed targets
    blobs = blobProxy_tgt[o].blobs_count
    if blobs > 0:
      num_tgt_observed += 1

      # add attract force to robots force sum
      for v in range(m):
        d = getDist(posProxy_rob[v], tgt)
        if d < do3:
	        rob_tgt[v] = 1 # at least one target found
          fatt = attract(posProxy_rob[v], tgt, d)
          fnet[v] = [fnet[v][0]+(fatt[0]/blobs), fnet[v][1]+(fatt[1]/blobs)]

  # Repulsion from other robots + movement
  for v in range(m):
    # Repulsion force
    frep = repulse(posProxy_rob, v)
    fnet[v] = [fnet[v][0]+frep[0], fnet[v][1]+frep[1]]
    # Move robot
    rad = math.atan2(fnet[v][1], fnet[v][0])
    fx = min(fnet[v][0], 2.0*math.cos(rad)) if fnet[v][0]>=0 \
	       else max(fnet[v][0], 2.0*math.cos(rad))
    fy = min(fnet[v][1], 2.0*math.sin(rad)) if fnet[v][1]>=0 \
	       else max(fnet[v][1], 2.0*math.sin(rad))

    # Check if free roaming
    if (rob_tgt[v] == 0) and (frep > -0.5):
      # no target found! move around to find target
      fx, fy, roaming[v] = freeRoam(posProxy_rob[v], roaming[v])
    else:
      roaming[v] = 0 # no longer roaming
#   [vx, rad] = calcVel(fnet[v])
#   posProxy_rob[v].set_cmd_vel(vx, 0.0, rad-posProxy_rob[v].pa, 1)
    posProxy_rob[v].set_cmd_vel(fx, fy, 0.0, 1)
    
    fnet[v] = [0, 0] # reset force to zero
    rob_tgt[v] = 0 # reset boolean

  count += 1 ## increment count

################
### CLEAN UP ###
################

# Stop robots and targets
for o in posProxy_tgt:
  o.set_cmd_vel(0.0, 0.0, 0.0, 1)

for v in posProxy_rob:
  v.set_cmd_vel(0.0, 0.0, 0.0, 1)

print "Average number of target observed per dt = %.3f" % (num_tgt_observed/T)

# Clean up (l.unsubcribe is for laser sensor)
for v in posProxy_rob:
  v.unsubscribe()
#for r in rangerProxy_rob:
  #r.unsubscribe()
for o in posProxy_tgt:
  o.unsubscribe()
for b in blobProxy_tgt:
  b.unsubscribe()
robot_rob.disconnect()
robot_tgt.disconnect()

