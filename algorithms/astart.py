import math
import sys
# import rospy
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")
from utility.heap import MinHeap

globe = {}

def insertIntoMinHeap (nodes):

    pass

def identifyOpenNodes (nodes):
    opened = []
    for key in nodes:
        if nodes[key]['parent'] != None:
            opened.append(key)
    return opened

def pythagoras(start, end):
    return abs(end[0]-start[0]) + abs(end[1] - start[1])
    # return math.sqrt((end[0]-start[0])**2 + (end[1] - start[1])**2)

def build2DMap (size, barriers):
    nodes = {}
    for i in range(size):
        for j in range(size):
            if ( i, j ) in barriers:
                continue
            children = []
            if i - 1 >= 0 and (i - 1, j) not in barriers: # top
                children.append((i-1,j))
            if i + 1 < size and (i + 1, j) not in barriers: # bottom
                children.append((i+1,j))
            if j + 1 < size and (i, j+1) not in barriers: # left
                children.append((i,j+1))
            if j - 1 >= size and (i, j-1) not in barriers: # right
                children.append((i,j-1))
            nodes[(i,j)] = { 'children':children, 'parent':None, 'distance':None }

    return nodes

def findNextBestNode (nodes, completed):
    bestFit = float('inf')
    nextNode = None
    for node in nodes:
        if node in completed:
            continue
        if nodes[node]['distance'] == None:
            continue
        if nodes[node]['distance'] < bestFit:
            nextNode = node
            bestFit = nodes[node]['distance']
    return nextNode

def findNextBestNodeSecond (nodes, known):
    bestFit = float('inf')
    nextNode = None
    for node in known:
        if nodes[node]['distance'] < bestFit:
            nextNode = node
            bestFit = nodes[node]['distance']
    return nextNode

def astar (start, end, nodes={}, maxIterations=1000):
    completed={}
    filledMatrix = {}
    currentlyAt = start
    nodes[currentlyAt]['distance'] = 0
    it = 0
    while maxIterations >= 0 and currentlyAt != end and currentlyAt != None:
        newDistance = nodes[currentlyAt]['distance'] + 1
        # + pythagoras(end, currentlyAt)

        for childName in nodes[currentlyAt]['children']:
            if nodes[childName]['distance'] == None or nodes[childName]['distance'] > newDistance:
                nodes[childName]['distance'] = newDistance
                nodes[childName]['parent'] = currentlyAt
                filledMatrix[childName] = newDistance

    
        completed[currentlyAt] = None
        if currentlyAt in filledMatrix:
            del filledMatrix[currentlyAt]
        # currentlyAtTemp = findNextBestNode(nodes, completed)
        # if currentlyAtTemp == None:
        #     print('oops')
        # currentlyAt = findNextBestNode(nodes, completed)
        currentlyAt = findNextBestNodeSecond(nodes, filledMatrix)

        it += 1

    return nodes

def extractPath (end, nodes):
    currentlyAt = end
    path = [end]
    while currentlyAt in nodes and nodes[currentlyAt]['parent'] != None:
        path.append(nodes[currentlyAt]['parent'])
        currentlyAt = nodes[currentlyAt]['parent']

    return path[::-1]

def runAstar (start, end, map_={}):
    nodes = astar (start, end, map_)
    return (extractPath(end, nodes), nodes)

# start = '0:0'
# end = '99:5'
# nodes = astar(start, end)
# path = extractPath(end, nodes)
# print(path)