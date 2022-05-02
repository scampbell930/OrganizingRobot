import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import collections  as mc
import graph
import sim as vrep # access all the VREP elements
from move_robot import move_robot


#return edges that contian the neighbors of a node
def neighbor_nodes(node , edges):
    neighbors = []
    for edge in edges:
        if edge.node_1 == node or edge.node_2 == node:
            neighbors.append(edge)
    return neighbors        

def Dijkstra(graph, source, target):
    unvisited = graph.nodes
    visited = []
    neighbors = []
    #Adds source node to visited and sets shortest distance to 0
    for n in unvisited:
        if n.number == source:
            n.shortest_distance = 0
            visited.append(n)
            break
    
    #First iteration starting from source node
    for edge in neighbor_nodes(visited[0].number, graph.edges):
        n_node = None
        n_visited = False
        #Finding which node in the edge is not the source node
        if(edge.node_1 == source):
         n_node = edge.node_2
        else:
         n_node = edge.node_1
        for n in visited:
            if n.number == n_node:
                n_visited = True
                break
        
        if not n_visited:
            for n in unvisited:
                if n.number == n_node:
                    if n.shortest_distance > (visited[0].shortest_distance + edge.distance):
                        n.shortest_distance = (visited[0].shortest_distance + edge.distance)
                        n.previous_node = visited[0].number
                        neighbors.append(n)
                        if n.number == target:  
                            return n
                        break
    
    while neighbors:
     for edge in neighbor_nodes(neighbors[0].number, graph.edges):
        n_node = None
        n_visited = False
        #Finding which node in the edge is not the source node
        if(edge.node_1 == neighbors[0].number):
         n_node = edge.node_2
        else:
         n_node = edge.node_1

        for n in visited:
            if n.number == n_node:
                n_visited = True
                break
                
        if not n_visited:
            for n in unvisited:
                if n.number == n_node:
                    if n.shortest_distance > (neighbors[0].shortest_distance + edge.distance):
                        n.shortest_distance = (neighbors[0].shortest_distance + edge.distance)
                        n.previous_node = neighbors[0].number
                        neighbors.append(n)
                        if n.number == target:  
                            return n
                        break  

     visited.append(neighbors[0])
     neighbors.remove(neighbors[0])

def closest_node(graph_nodes):
    err_code,robot_handle = vrep.simxGetObjectHandle(clientID, "/youBot", vrep.simx_opmode_blocking)
    err,position = vrep.simxGetObjectPosition(clientID,robot_handle,-1, vrep.simx_opmode_blocking)
    print(position)
    c_node = 0
    c_distance = math.dist( [position[0], position[1]] , [graph_nodes[c_node].x,graph_nodes[c_node].y])

    for i in range(len(graph_nodes)):
         
        node_distance = math.dist( [position[0], position[1]] , [graph_nodes[i].x,graph_nodes[i].y])
        if(node_distance < c_distance):
            c_node = i
            c_distance = node_distance
    
    move_robot(clientID, [graph_nodes[c_node]], False)
    return c_node





nodes = []
edges = []

i = 0
for l in open('nodes.txt', 'r'):
  values = l.split()
  new_node = graph.Node(float(values[0]),float(values[1]), True, i)
  nodes.append(new_node)
  i += 1
  

for l in open('edges.txt', 'r'):
  values = l.split()
  new_edge = graph.Edge(float(values[0]),float(values[1]), float(values[2]))
  edges.append(new_edge)

vrep.simxFinish(-1) # just in case, close all opened connections
clientID= vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a

if clientID != -1:
 err_code,robot_handle = vrep.simxGetObjectHandle(clientID, "/youBot", vrep.simx_opmode_blocking)
 err_code,fr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fr", vrep.simx_opmode_blocking)
 err_code,fl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fl", vrep.simx_opmode_blocking)
 err_code,rr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rr", vrep.simx_opmode_blocking)
 err_code,rl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rl", vrep.simx_opmode_blocking)
 err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0, vrep.simx_opmode_streaming)
 err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0, vrep.simx_opmode_streaming)
 err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0, vrep.simx_opmode_streaming)
 err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0, vrep.simx_opmode_streaming)

 g = graph.Graph()
 g.nodes = nodes
 g.edges = edges

 #Define source and destination here
 source = closest_node(nodes)
 destination = 112

 
 final_node = Dijkstra(g,source,destination) #Returns final node and can trace back to source node from there
 path = []
 path.append(final_node)
 prev_node = final_node.previous_node

 print(str(final_node.shortest_distance) + ' ' + '-> Path Length')
 while prev_node != source:
    for n in nodes:
        if prev_node == n.number:
            path.append(n)
            prev_node = n.previous_node
            print(n.number)
            break
 print(source)
 lines = []       
 for i in range(len(path)-1):
   l = [(path[i].x, path[i].y) , (path[i+1].x, path[i+1].y)] 
   print(l)
   lines.append(l)
   

 lines.append([(nodes[source].x,nodes[source].y), (path[len(path)-1].x ,path[len(path)-1].y )])  
 fig, ax = plt.subplots()
 for n in nodes:
    ax.annotate(n.number, (n.x,n.y))
 ax.set_xlim(-5.5, 5.5)
 ax.set_ylim(-5.5,5.5)


 line_segments = mc.LineCollection(lines,
                               linewidths=(0.5, 1, 1.5, 2),
                               linestyles='dashed',
                               color= "black")
                              
                          
 ax.add_collection(line_segments)
 

 plt.show()
 move_robot(clientID,reversed(path),False)