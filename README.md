#  Features

### 1. Path finding:
We uses PRM and A star algorithm to calculate the path for our agent to reach our goal.
<img src="pathfinding.gif" alt="drawing" width="50%"/> <br />
### 2. Shortcut Design:
When the path is found, the agent won't strictly follow the path. It can calculate whether there is a collision between the following node or not. If there is no collision between the agent and the following node, it will skip the current node and go towards the following node.
### 3. Resetting and pause simulation
Press R to reset the simulation, press Space to pause the simulation
<img src="reset and pause.gif" alt="drawing" width="50%"/> <br />
### 4. Adding agents:
The user can add agent anywhere in the map by using the left click of their mouse. The new agent will be randomly given a goal point and PRM will calculate the path immediatly.
<img src="multiagent.gif" alt="drawing" width="50%"/> <br />
### 5. TTC crowd simulation:
When there are multiple agents on the map, and they are about to collide, the TTC can provide forces to pull them away and also make sure they don't collide with the obstacle.

# Difficulties
### 1. The lack of data structures:
the initial design of how to store the path was using JAVA's linkedlist. When the agent gets to a node, we use the linkedlist removefirst() method to pop the node. But, Processing did not inherit this JAVA structure, so, we have to use ArrayList to store the path and use indexes to keep track the nodes.


