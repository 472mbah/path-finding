import sys
import time
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")
from utility.util import sendRaw, convertIntoKeysRaw
from utility.turn import getDistance
from algorithms.automate.prims import produceMaze


def convertMazeIntoBlocks (maze, dimensions=10, blockMode=True):
    blocks = {}
    start = (0, 0)
    end = (0, 0)

    prevStartDistance = float('inf')
    prevEndDistance = float('inf')

    startPivot = (0, 0)
    endPivot = (dimensions, dimensions)

    for i in range(len(maze)):
        row = maze[i]
           
        for j in range(len(row)):
            value = row[j]
            if value == 'w':
                blocks[f'{i}:{j}' if not blockMode else (i, j)] = None
            else:
                cords = (i, j)
                currDistanceStart = getDistance(cords, startPivot)
                if currDistanceStart < prevStartDistance:
                    start = cords
                    prevStartDistance = currDistanceStart

                currDistanceEnd = getDistance(cords, endPivot)
                if currDistanceEnd < prevEndDistance:
                    end = cords
                    prevEndDistance = currDistanceEnd

    return (start, end, blocks)

def getBorder (d, existing={}, blockMode=True):
    # do top
    
    for i in range(d):
        existing[ f'-1:{i}' if not blockMode else (-1, i) ] = None

    for i in range(d):
        existing[ f'{d}:{i}' if not blockMode else (d, i) ] = None

    for i in range(-1,d+1):
        existing[ f'{i}:{-1}' if not blockMode else (d, i) ] = None 
    for i in range(-1,d+1):
        existing[ f'{i}:{37}' if not blockMode else (d, i) ] = None                
    
    return existing

def buildBlockMaze (d):
    maze = produceMaze(d, d)
    (start, end, maze) = convertMazeIntoBlocks(maze, d)
    return (start, end, getBorder(d, maze))

# # for i in range(40):
# stepDetails = { 'steps':[], 'visited':{}, 'end':(-1, -1) }
# d = 10
# maze = produceMaze(d, d)
# withBorder = getBorder(d, convertMazeIntoBlocks(maze))
# sendRaw(stepDetails, (0,0,0,(20,20)), withBorder)
# time.sleep(.5)