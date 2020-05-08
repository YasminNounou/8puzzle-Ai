import math
import time
import copy
import sys
from queue import Queue
from Node import Node
from aStarNode import aStarNode
from collections import deque
import heapq

#function used for printing path,depth ,nodes, states , running time as output
def print_path(SNode,time0,deep,exploredcount):
    t1=time.time()-time0
    pathNodes = deque()
    path = []
    pathStates=[]
    state=Queue(maxsize=0)

    k = 1
    while SNode.parent!= None:
        pathNodes.append(SNode)
        SNode=SNode.parent
    pathNodes.append(SNode)
    print("\n*************************")
    print("\t  Initial state\n")
    print("\t-----------")
    for i in range(3):
        for j in range(3):
            print("\t|" + str(initial[i][j]) + "|", end="")
        print("")
        print("\t-----------")
    #print(initial)
    while pathNodes:
        neural=pathNodes.pop()
        pathStates.append(copy.copy(neural.state))
        path.append(neural.move)
        state.put(neural.state)

    while not state.empty():
        state1=state.get()
        print("\n*************************")
        print("\t  State " + str(k) + "\n")
        print("\t-----------")
        for i in range(3):
          for j in range(3):
            print("\t|" + str(state1[i][j]) + "|", end="")
          print("")
          print("\t-----------")
        k += 1
    print("\n- PATH: "+ str(path[1:]))
    print("- Running Time:  " + str(t1) + " secs")
    print("- Depth: "+str(deep))
    print("- Number of explored nodes: "+ str(exploredcount))
    return pathStates
#returns list of all path nodes to use it for display
    
#function for indicating neighbors for each node
def find_Neighbors(node):
    index = []
    #list to store strings for possible moves for each node
    possible_moves = []

    for i in range(3):
        for j in range(3):
            if node.state[i][j] == 0:
                #blank box position that can move
                index = [i, j]

    #check if it can move up
    if index[0] - 1 >= 0:
        #take copy of node.state as not to adjust original one
        up = copy.deepcopy(node.state)
        #print(up)
        #swap elements
        up[index[0]][index[1]] = up[index[0] - 1][index[1]]
        up[index[0] - 1][index[1]] = 0
        # append adjusted state to neighbors of original node
        node.neighbors.append(up)
        possible_moves.append("UP")

    # check if it can move down
    if index[0] + 1 <= 2:
        # take another copy of node.state as not to adjust original one
        down = copy.deepcopy(node.state)
        #swap elements to move down
        down[index[0]][index[1]] = down[index[0] + 1][index[1]]
        down[index[0] + 1][index[1]] = 0
        #print(down)
        # append adjusted state to neighbors of original node
        node.neighbors.append(down)
        possible_moves.append("DOWN")
        #print("Go DOWN")

    # check if it can move left
    if index[1] - 1 >= 0:
        left = copy.deepcopy(node.state)
        #swap elements in copy state
        left[index[0]][index[1]] = left[index[0]][index[1] - 1]
        left[index[0]][index[1] - 1] = 0
        # append adjusted state to neighbors of original node
        node.neighbors.append(left)
        possible_moves.append("LEFT")
        #print("Go LEFT")

    # check if it can move right
    if index[1] + 1 <= 2:
        right = copy.deepcopy(node.state)
        #swap elements in copy state to move right
        right[index[0]][index[1]] = right[index[0]][index[1] + 1]
        right[index[0]][index[1] + 1] = 0
        # append adjusted state to neighbors of original node
        node.neighbors.append(right)
        possible_moves.append("RIGHT")
        #print("Go RIGHT")
#return list with all state neighbors and list with possible moves"strings"
    return node.neighbors,possible_moves

def BFS(initial,goal):
    #create false node to return at failuer
    fall=Node(None,404,None,None,None)
#initializecounter and create new node for intiial state
    exploredNodes=0
    #intiialize queueu for BFS algorithm
    frontier=Queue(maxsize=0)
    newNode=Node(None, 0, initial,[],None)
    frontier.put(newNode)
    explored=[]
    while not frontier.empty():
        #o=0
        # pop current node that we will explore from frontier
        current=frontier.get()
        #add it to explored list and increment count
        explored.append(list(current.state))
        exploredNodes+=1
        # if the state of current node is = to goal state then return success
        if current.state==goal:
            return current,current.child,exploredNodes
        #else search for neighbors and possible moves
        current.neighbors,moves=find_Neighbors(current)
        neighbors = current.neighbors
       # print(moves)
    #loop on all neighbors and moves altogether to get the unexplored node and add it to frontier
        for neighbor,move in zip(neighbors,moves):
            
            if neighbor not in explored:
                #print("Neighbor loop")
                # creating new node for the child to add to frontier to explore it later when popped
                child = Node(current, current.child+1, neighbor, [],move)
               # print("CHILD'S MOVE")
               # print(child.move)
                frontier.put(child)
                

    return fall


def DFS(initial, goal):
    # create false node to return at failure
    fall=Node(None,404,None,None,None)
    # initialize counter and create new node for intiial state
    exploredNodes=0
    # intiialize stack for DFS algorithm
    frontier=deque()
    newNode=Node(None, 0, initial,[],None)
    frontier.append(newNode)
    explored=[]

    while frontier:
        #pop current node that we will explore from frontier
        current=frontier.pop()
        # add it to explored list and increment count
        explored.append(current.state)
        exploredNodes+=1
        #if the state of current node is = to goal state then return success
        if current.state==goal:
            return current,current.child,exploredNodes
#else search for neighbors and possible moves
        current.neighbors,moves=find_Neighbors(current)
        neighbors = current.neighbors
        #print(moves)
 # loop on all neighbors and moves altogether to get the unexplored node and add it to frontier
        for neighbor,move in zip(neighbors,moves):
            
            if neighbor not in explored and neighbor not in frontier:
               # print("Neighbor loop")
               #creating new node to add to frontier to explore it later when popped
                child = Node(current, current.child+1, neighbor, [],move)
                frontier.append(child)
                

    return fall

def Astar(initial, goal,choice1):
    # create false node to return at failure
    fall = aStarNode(None, 404, None, None, None,None,None)
    # initialize counter and create new node for intiial state
    new_Node = aStarNode(None, 0, initial, 0, 0, [], [])
    # intiialize heap for A* algorithm
    frontier = []
    explored = []
    exploredNodesCount = 0
#convert frontier into heap that poips the least cost
    heapq.heapify(frontier)
    heapq.heappush(frontier, new_Node)
    while frontier:
    #pop from heap(fronier) the current node that we will explore
    #increment explore dnodes
        current = heapq.heappop(frontier)
        explored.append(current.state)
        exploredNodesCount += 1
    # if the state of current node is = to goal state then return success
        if current.state == goal:
            return current,current.child,exploredNodesCount
    # else search for neighbors and possible moves
        current.neighbors, moves = find_Neighbors(current)
        neighbors = current.neighbors
        for neighbor, move in zip(neighbors, moves):
            if neighbor not in explored and neighbor not in frontier:
                #if node state is hasn't been explored of put in heap start calculating its cost
                if choice1=="1":
                    #if the user chose eucledian at first
                  functionH = 0
                  for i in range(3):
                     for j in range(3):
                         #function given in the pdf
                        functionH = functionH + math.sqrt(pow(i - neighbor[i][j] // 3, 2) + pow(j - neighbor[i][j] - 3 * i, 2))
                  cost = functionH + ((current.functionG)+1)
                  print(cost)
                  child = aStarNode(current, current.child + 1, neighbor,current.functionG+1,cost, [], move)
                  heapq.heappush(frontier, child)
                elif choice1=="2":
                    # if the user chose manhattan at first
                    funtionH = 0
                    for i in range(3):
                        for j in range(3):
                            # function given in the pdf
                            funtionH = funtionH + abs(i - neighbor[i][j] // 3) + abs(j - neighbor[i][j] - 3 * i)
                    cost = funtionH + ((current.functionG)+1)
                    #create child to add it in the heap
                    child = aStarNode(current, current.child + 1, neighbor, current.functionG + 1, cost, [], move)
                    heapq.heappush(frontier, child)
                    #search for the already explored node and compare their cost to keep the least
            elif neighbor in frontier:
                for index, element in frontier:
                    if neighbor == element:
                        frontier[index] = frontier[-1]
                        frontier.pop()
                        heapq.heapify(frontier)
                        updated_cost=element.cost
                        #esle create anothe rone and store in the heap
                if choice1=="1":
                  functionH = 0
                  for i in range(3):
                     for j in range(3):
                        functionH = functionH + math.sqrt(pow(i - neighbor[i][j] // 3, 2) + pow(j - neighbor[i][j] - 3 * i, 2))
                  cost = functionH + ((current.functionG)+1)
                elif choice1=="2":
                    funtionH = 0
                    for i in range(3):
                        for j in range(3):
                            funtionH = funtionH + abs(i - neighbor[i][j] // 3) + abs(j - neighbor[i][j] - 3 * i)
                    cost = funtionH + ((current.functionG)+1)
                if cost>updated_cost:
                    cost=updated_cost
                child = aStarNode(current, current.child + 1, neighbor, current.functionG + 1, cost, [], move)
                heapq.heappush(frontier, child)

    return fall



# Transforms any 1d array to 2d
# @param: array1d (list)
# @returns: list of lists, each list n
def to_matrix(array1d, n):
    return [array1d[i:i+n] for i in range(0, len(array1d), n)]

#***************************____________START__________***************************************

puz_str = input("Welcome \n please input your puzzle comma seperated:")

# Parse input into a list
puzzle = puz_str.split(",")

# loop to trim any input with spaces in list and type cast input
for i in range(0, len(puzzle)):
    puzzle[i] = int(puzzle[i].strip())

initial = to_matrix(puzzle, 3)

#initial = [[1,2,5], [3, 4, 0], [6, 7, 8]]
goal=[[0,1,2],[3,4,5],[6,7,8]]

#initialize variable used to input loop
is_True=True
input1=""
choice=""
depth=0
expcount=0
if goal==initial:
    print("\nEnter different initial state\n")
#let the user choose option for algorithm required for solution
else:
  while(is_True):
    input1 = input("Enter your choice's number:\n 1.BFS\n 2.DFS\n 3.A*\nYour choice:")
    if input1=="1":
        t0 = time.time()        #start time
        successNode,depth,expcount=BFS(initial,goal)
        is_True=False
    elif input1=="2":
       t0 = time.time()        #start time
       successNode,depth,expcount=DFS(initial,goal)
       is_True = False
    elif input1=="3":
        #let user choose type of cost calculation whether manhattan or eucledian
        choice = input("\n 1.Manhattan\t2.Euclidean")
        if choice != "1" and choice != "2":
            print("Invalid choice!! ")
            sys.exit(0)
        t0 = time.time()        #start time
        successNode,depth,expcount= Astar(initial,goal,choice)
        print(successNode.child)
        is_True = False
    else:
        print("Invalid input!")
        #input=input("Choose again")

#print path is a function reponsilble for printing required output
  states=print_path(successNode,t0,depth,expcount)




