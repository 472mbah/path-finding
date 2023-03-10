'''
Credit to: https://gist.github.com/josiahcarlson/904686
'''
import random
import sys
import math
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")
from utility.turn import getDistance
from utility.util import sendRaw
from algorithms.automate.efficiency import getBorder
random.seed(0.24975077935850032)

'''
0.24975077935850032 -> This is a problem for mg4 for the 17th iteration
This is an implementation of the Recursive Division algorithm for maze
generation, primarily derived from the algorithm presented here:
http://weblog.jamisbuck.org/2011/1/12/maze-generation-recursive-division-algorithm
The input grid can be generated via:
  grid = [width*[0] for i in xrange(height)]
The initial call should be of the form:
  divide(grid, 0, 0, width, height)
The meanings of the entries are the same as is described in Jamis Buck's other
maze articles.
'''

N, S, E, W = 1, 2, 4, 8
HORIZONTAL, VERTICAL = 0, 1

def divide(grid, mx, my, ax, ay):
    dx = ax - mx
    dy = ay - my
    if dx < 2 or dy < 2:
        # make a hallway
        if dx > 1:
            y = my
            for x in range(mx, ax-1):
                grid[y][x] |= E
                grid[y][x+1] |= W
        elif dy > 1:
            x = mx
            for y in range(my, ay-1):
                grid[y][x] |= S
                grid[y+1][x] |= N
        return

    wall = HORIZONTAL if dy > dx else (VERTICAL if dx > dy else random.randrange(2))

    xp = random.randrange(mx, ax-(wall == VERTICAL))
    yp = random.randrange(my, ay-(wall == HORIZONTAL))

    x, y = xp, yp
    if wall == HORIZONTAL:
        ny = y + 1
        grid[y][x] |= S
        grid[ny][x] |= N

        divide(grid, mx, my, ax, ny)
        divide(grid, mx, ny, ax, ay)
    else:
        nx = x + 1
        grid[y][x] |= E
        grid[y][nx] |= W

        divide(grid, mx, my, nx, ay)
        divide(grid, nx, my, ax, ay)

def getColumn (arr, index):
    col = []
    for row in arr:
        col.append(row[index])
    return col

def convertIntoGrid (values):
    newGrid = []
    for k in range(len(values)):
        row = values[k]
        # where do we put a -1?
        newRow = []
        i = 1
        while i < len(row):
            previous = row[i-1]
            current = row[i]
            newRow.append(previous)
            if current!=previous:
                newRow.append(-1)
            i+=1
        newGrid.append(newRow)
    
        if k+1 < len(values):
            midWay = []
            targetIndex = 0
            b = 0
            while b < len(newRow):
                if newRow[b] == -1:
                    midWay.append(-1)
                    b+=1
                    continue
                
                curr = newRow[b]
                next = values[k+1][targetIndex]
                if curr != next:
                    midWay.append(-1)
                else:
                    midWay.append(next)
                targetIndex+=1
                b+=1

            newGrid.append(midWay)


    return newGrid




def convertMazeIntoBlocks (maze, blockMode=True):
    blocks = {}
    start = (0, 0)
    end = (0, 0)

    prevStartDistance = float('inf')
    prevEndDistance = float('inf')      

    startPivot = (0, 0)
    endPivot = (len(maze)*2, len(maze)*2)

    maxRowLength = 0

    for i in range(len(maze)):
        row = maze[i]
        maxRowLength = max(maxRowLength, len(row))
        for j in range(len(row)):
            value = row[j]
            cords = (i, j)

            if value != -1:
                blocks[f'{i}:{j}' if not blockMode else (i, j)] = None

            else:
                currDistanceStart = getDistance(cords, startPivot)
                if currDistanceStart < prevStartDistance:
                    start = cords
                    prevStartDistance = currDistanceStart

                currDistanceEnd = getDistance(cords, endPivot)
                if currDistanceEnd < prevEndDistance:
                    end = cords
                    prevEndDistance = currDistanceEnd

    return (start, end, blocks, (len(maze)-1, maxRowLength))


def produceGrid (dimensions):
    old = dimensions
    dimensions = math.floor((dimensions)/2) + 1
    grid = [dimensions*[0] for _ in range(dimensions)]
    divide(grid, 0, 0, dimensions, dimensions)
    (start, end, maze, foundDimensions) = convertMazeIntoBlocks(convertIntoGrid(grid))
    # end = (end[0]-3, end[1]-3)
    data = getBorder(old, maze) 
    if end in data:
        del data[end]
    return (start, end, data)
        # stepDetails = { 'steps':[], 'visited':{}, 'end':(-1, -1) }
        # sendRaw(stepDetails, (0,0,0,(20,20)), data)
        # time.sleep(0.1)
