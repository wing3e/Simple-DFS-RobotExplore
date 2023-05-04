# Simple-DFS-RobotExplore
In this project, a robot can automatically search for the targets in an unknown map. The core algorithm is DFS, with some other data structures like  double linked list. However, there are many disadvantages in this code, I will continually upgrade it.
# Regular
(1) Consider a simple mobile robot that is able to move along a path that can be walked on a plane and is able to record its own running track. To simplify the problem, it is assumed that in each location, the robot can only move in eight directions: front, back, left, right, front right, back right, front left and back left, and can only perceive the information of eight local locations around it (feasible path, unreachable area or target point). Write a program to read a 100x100 grid point plane map (map file can refer to the following format given: as shown in the figure. Represents the area that cannot be walked, + represents the path that the robot can walk, * represents the exploration target point that the robot wants to find, and # represents the special important target point that must be reached), so that the mobile robot can explore in the area represented by the map according to the perception and movement rules. Requirements: 1) The robot can find all the target points and paths and then return; 2) Draw the goal-road map of the area according to the adventure experience (note that the given ground cannot be drawn directly, because the goal of the robot is to explore, it does not know the global goal and path distribution in advance, and can only reconstruct the map by means of moving and local perception information and recording it), To fulfill our purpose of sending robots to explore the field. Note that the routes in the map are hypothetical paths that may result in closure or inaccessibility.

(2) Think about what data structures can be used to represent the mapped expedition map in addition to the above map file representation method. Try to use different data representation methods to represent the results of robot exploration (for example, dynamic linked list can be used, the exploration target points as nodes, whether there is a direct path between the target points as a link), so that it can be easier and more direct to traverse the target points on the map again; Think about ways to make the journey more visible.

(3) Using the established adventure map as a guide, try to traverse all the target points again in the most energy-efficient way.

(4) Note that the targets and paths in the above map are simple unit points and lines. If the path width, target shape and size are arbitrary, how to realize the function of exploration? At this time, it is more convenient to use the logical connection representation of dynamic linked lists.