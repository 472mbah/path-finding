import sys
import random
import time
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")
from utility.util import sendRaw, convertIntoKeysRaw

def getOptions (current):
    return [
        (current[0]-1, current[1]),  # top
        (current[0], current[1]+1),  # right
        (current[0]+1, current[1]),  # bottom
        (current[0], current[1]-1),  # left    
    ]

def build2DMap (size):
    nodes = {}
    for i in range(size):
        for j in range(size):
            children = []
            if i - 1 >= 0 and (i - 1, j): # top
                children.append((i-1,j))
            if i + 1 < size and (i + 1, j): # bottom
                children.append((i+1,j))  
            if j + 1 < size and (i, j+1): # left
                children.append((i,j+1))                                
            if j - 1 >= size and (i, j-1): # right
                children.append((i,j-1))
            nodes[(i,j)] = { 'children':children, 'parent':None, 'distance':None }

    return nodes

"""
Initialize a grid of cells, where each cell is a closed cell with all walls intact.
Choose a starting cell and mark it as visited.
Create a list of unvisited cells and add all cells in the grid to it.
While there are unvisited cells:
a. Choose a random unvisited cell from the list.
b. Create a list of its unvisited neighbors.
c. If the list is not empty, choose a random neighbor and remove the wall between the current cell and the chosen neighbor.
d. Mark the chosen neighbor as visited and remove it from the unvisited list.
Choose a random starting and ending point, open up the corresponding walls to create entrance and exit.
The final product will be a maze with one or more paths from the entrance to the exit.
"""

def produceMaze ():
    grid = build2DMap(40)
    remaining = {}
    unvistied = list(grid)
    while len(unvistied) > 0:
        nextOne = random.choice(unvistied)
        unvistied.remove(nextOne)
        neighbours = grid[nextOne]['children']
        # filter them
        filtered = []
        for item in neighbours:
            if item in unvistied:
                filtered.append(item)
        if len(filtered):
            unvistied.remove(filtered[0])
            remaining[filtered[0]] = None

    stepDetails = { 'steps':[], 'visited':{}, 'end':(-1, -1) }
    sendRaw(stepDetails, (0,0,0,(20,20)), convertIntoKeysRaw(remaining))
    time.sleep(.5)

def findNodes (pair, connections):
    for k in range(len(connections)):
        arr = connections[k]
        for item in arr:
            if item in pair:
                return k
            # for p in pair:
            #     if item in getOptions(p):
            #         return k
    return -1

def addPair (targetIndex, items, connections):
    for item in items:
        if item not in connections[targetIndex]:
            connections[targetIndex].append(item)

def doubleValues (connections, size):
    doubledBlockers = {}
    for a in range(len(connections)):
        val = connections[a]
        for b in range(len(val)):
            newTup = (connections[a][b][0] * 2, connections[a][b][1] * 2)
            doubledBlockers[newTup] = None
    blockers = {}
    for i in range(size):
        for j in range(size):
            if (i, j) in doubledBlockers:
                blockers[(i, j)] = None
    return blockers

def kruskall ():
    grid = build2DMap(10)
    unvistied = list(grid)

    connections = [ [] ]

    while len(unvistied) > 0:
        nextOne = random.choice(unvistied)
        unvistied.remove(nextOne)
        neighbours = grid[nextOne]['children']
        # filter them
        chosen = None
        for item in neighbours:
            if item in unvistied:
                chosen = item
                break
        
        if chosen == None:
            continue
            
        """
            See if either of the nodes, nextOne or chosen are in a list
            Else create new connection and append it
        """
        pair = [ nextOne, chosen ]
        pairIndex = findNodes(pair, connections)
        if pairIndex > -1:
            addPair (pairIndex, pair, connections)
        else:
            connections.append(pair)
    return doubleValues(connections, 20)
    # return connections


for k in range(10):
    blocks = kruskall()
    # blocks = {}
    # for x in conns:
    #     for k in x:
    #         blocks[k] = None
    stepDetails = { 'steps':[], 'visited':{}, 'end':(-1, -1) }
    sendRaw(stepDetails, (0,0,0,(20,20)), convertIntoKeysRaw(blocks))
    time.sleep(.5)        
    True
