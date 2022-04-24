import math
from random import random, randrange
from time import sleep
import numpy as np
from move_robot import move_robot
import sim as vrep # access all the VREP elements
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import collections  as mc
import graph
from classifier import stream_vision_sensor


#Finds closest node for RRT
def find_closest_node(point: list[float], nodes: list[graph.Node]) -> graph.Node:
    closest_node = nodes[0]
    closest_distance = np.linalg.norm(point - np.array((nodes[0].x,nodes[0].y))) 
    for node in nodes:
        node_point = np.array((node.x,node.y))
        distance = np.linalg.norm(point - node_point)
        if(distance < closest_distance):
            closest_node = node
            closest_distance = distance
    return closest_node  

#Reduce false negatives for checking if there is an object at a point
def check_for_object() -> bool:
    err_code, collision1 = vrep.simxCheckCollision(clientID,dummy_handle,vrep.sim_handle_all,vrep.simx_opmode_blocking )
    err_code, collision2 = vrep.simxCheckCollision(clientID,dummy_handle,vrep.sim_handle_all,vrep.simx_opmode_blocking )
    if( collision1 == collision2):
        return collision1
    return check_for_object()


def find_frontier_points() -> list[graph.Node]:
 frontier_nodes = []

 #Perform RRT until 5 frontier nodes are found
 while len(frontier_nodes) < 5:

     #Create random point and find closest node to point
     random_x = np.random.uniform(-5, 5)
     random_y = np.random.uniform(-5, 5)
     random_point = np.array((random_x,random_y))
     closest_node = find_closest_node(random_point,g.nodes)
    

     for region in known_region:

          
      if(closest_node.free and region.get_path().contains_point((closest_node.x,closest_node.y))):  #Check if closest node is not in an object and is within the known region

       norm = math.sqrt((random_x-closest_node.x) ** 2 + (random_y-closest_node.y) ** 2)
       direction = [((random_x-closest_node.x) / norm) *0.5, ((random_y-closest_node.y) / norm) * 0.5] 
      
       new_node_x = closest_node.x + direction[0]
       new_node_y = closest_node.y + direction[1]

       #Sets dummy to new node position and checks for object
       err_code = vrep.simxSetObjectPosition(clientID,dummy_handle,-1,[new_node_x,new_node_y,0.2], vrep.simx_opmode_blocking )
       collision = check_for_object()

       new_node = graph.Node(new_node_x,new_node_y, not collision) #New Node to RRT
       in_known = False

       for region in known_region:  
        if(region.get_path().contains_point((new_node_x,new_node_y))):
         in_known = True  
      
       if(not in_known and not collision): #If new node is not in known and is not in an object then it is a frontier point
        frontier_nodes.append(new_node)

       g.nodes.append(new_node)
       e = graph.Edge(closest_node,new_node) #Create edge from closest node to new node
       g.edges.append(e)
       break
 return frontier_nodes 


def add_known_region() -> None:
 err,position = vrep.simxGetObjectPosition(clientID,robot_handle,-1, vrep.simx_opmode_blocking)
 y = np.array([[position[0] + 1, position[1] + 1], [position[0] + 1, position[1] - 1], [position[0] - 1, position[1] - 1], [position[0] - 1, position[1] + 1]])
 p = Polygon(y, facecolor = 'k')
 ax.add_patch(p)
 known_region.append(p)


def frontier_point_evaluation(paths:list[graph.Node]) -> list[graph.Node]:
    optmized_path = paths[0]
    for path in paths:
        if(len(path) < len(optmized_path)): #Checks for path with least amount of nodes
            optmized_path = path
    return optmized_path        


if __name__ == "__main__":
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

    if clientID != -1:
    
        err_code,dummy_handle = vrep.simxGetObjectHandle(clientID, "/Cuboid[4]", vrep.simx_opmode_blocking) 
        err_code,robot_handle = vrep.simxGetObjectHandle(clientID, "/youBot", vrep.simx_opmode_blocking)
        err_code,fr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fr", vrep.simx_opmode_blocking)
        err_code,fl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fl", vrep.simx_opmode_blocking)
        err_code,rr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rr", vrep.simx_opmode_blocking)
        err_code,rl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rl", vrep.simx_opmode_blocking)

        err_code = vrep.simxSetJointTargetVelocity(clientID,fr_wheel_handle,0,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,fl_wheel_handle,0,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,rr_wheel_handle,0,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,rl_wheel_handle,0,vrep.simx_opmode_streaming)

        #Initialize Graph and append starting ndoe to nodes list


        # Create initial known region
        fig, ax = plt.subplots()
        ax.margins(0.1)
        ax.set_xlim([-5,5])
        ax.set_ylim([-5,5])
        known_region = []
        lines = []
        
        #Iterates through RRT and path planning indefinetly
        while True:

            #Initialize Graph and append starting node to nodes list   
            g = graph.Graph()
            err,position = vrep.simxGetObjectPosition(clientID,robot_handle,-1, vrep.simx_opmode_blocking)
            start_node = graph.Node(position[0],position[1], True)
            g.nodes.append(start_node)   

            add_known_region() #Add location of robot to known region
            frontier_nodes = find_frontier_points() #Use RRT to find frontier points
                
            paths = []

            for i in range(len(frontier_nodes)): #Create a path to starting position for each frontier point
                path = [frontier_nodes[i]]
                next_node = frontier_nodes[i]
                while next_node != g.nodes[0]:
                    for edge in reversed(g.edges):
                        if(edge.node_2 == next_node):
                            if(edge.node_1 != g.nodes[0]):
                                path.append(edge.node_1)
                            next_node = edge.node_1 
                            break    
                paths.append(path)  
            

            optmized_path = frontier_point_evaluation(paths) #Return optimal path to frontier 
            detection = move_robot(clientID,reversed(optmized_path))

            # Start pickup
            if detection:
                print("now go pick it up")
                break
  
