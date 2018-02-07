# Cooperative Multi-Robot Observation of Moving Targets Scenario
## RUNNING SIMULATION ##
To run the simulation, follow these steps:
1) open terminal
2) go to directory '<location-to-directory>/world' and run
3) > player stageall.cfg
4) open another terminal 
5) go to directory '<location-to-directory>/world' and run
6) > python sim_hol.py n m R
	where n is num of targets [1, 20]
	where m is num of robots [1, 10]
	where R is the radius of the environment in meters
6*) in order to change the radius, the rink.world line size[x y 1]
	where x,y are the circle diameter size (plus 10 for bitmap boundary)
	
## Scenario Description ##
Description: m holonomic robots with 360-degree field of view sensors of range do3 must observe n holonomic targets moving randomly within a circular environment of radius R. The speed of each target is fixed and randomly chosen to be between 0 m/s and 1.5 m/s. The maximum speed of each robot is 2 m/s.

Assumptions: The sensing of each robot is perfect and each robot may communicate with other robots throughout the environment. 

Goal: Maximize the average number of targets that are being observed by at least one robot at each time step dt throughout the mission of length T.

## SIMULATION DESIGN ##
In order to simplify the simulation design, I installed the sensors on the targets instead of the robots. The sensors (ranger or blobfinder) can be easily installed on the robots and the simulation results would remain the same. It was however, simpler for the simulation to put the blobfinders on the target. The simulation results are not affected; however, the design/code is simpler!

If the sensors were installed on the robots, then I would have had to run a for loop to create the B[t] matrix, then another for loop on it to calculate the force summation. With the current design, I can calculate the matric and force summation in one loop.

## Result Analysis ##
As expected, the data collected gives best results in a smaller environment, where the robots can cover the area in relatively good speed. As the environment increased, it became increasingly difficult for the robots to find the more sparse targets. The robots have max speed of 2m/s, which is also too slow for a circle of radius 500m. 

In addition, as the ratio of targets to robots increase, there are less targets observed in average because there are many more targets than robots to find. Due to robot tracking, once a robot finds a target, it will follow it, making it harder to "free roam" and find other targets if they are sparse. However, the average number of targets observed was significantly impacted by the robots and targets initial position. Some simulations will perform significantly better because the robots started near the targets or the targets were congregated in a certain area.

One last observation, for the n/m = 1/5, the outcome was almost completely random! When the target spawns near a robot, the target is tracked for the whole duration for a 100% targets observed. However, if the target was initialized very far away from robots, odds are against the robots to find tit leading to a 0% targets observed. In a smaller aread, the robots almost always find the target though!
