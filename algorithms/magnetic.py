from utility.turn import getDistance
from utility.util import addNode, inBucket
import sys
# import rospy
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")

error = False

def runMitad(start, end, blockers, areaSize, blockSize):
    global error

    if inBucket(end, blockers, areaSize, blockSize):
        if not error:
            error = True
        return ([], {})


    visited = {}
    reserves = []
    #end = [9, 9]
    #dimensions = [10, 10]
    #start = [0, 0]
    path = []
    # Ensure options are valid before continuing

    def filterOptions(cords):
        # if inRange(cords, dimensions):
        #     return False
        if inBucket(cords, visited, areaSize, blockSize):
            return False
        if inBucket(cords, blockers, areaSize, blockSize):
            return False
        return True

    def mitad(current):

        if current[0] == end[0] and current[1] == end[1]:
            return True

        options = [
            # (current[0]-1, current[1]-1),  # top left
            # (current[0]-1, current[1]+1),  # top right
            # (current[0]+1, current[1]-1),  # bottom left
            # (current[0]+1, current[1]+1),  # bottom right

            (current[0]-1, current[1]),  # top
            (current[0], current[1]+1),  # right
            (current[0]+1, current[1]),  # bottom
            (current[0], current[1]-1),  # left
        ]

        options = list(filter(filterOptions, options))
        options_size = len(options)
        hasReserves = len(reserves) > 0
        hasOptions = options_size > 0

        if not hasOptions and hasReserves:
            options.append(reserves.pop(0))
            options_size = 1
        elif not hasOptions and not hasReserves:
            return False

        bestFitness = float('inf')
        bestNodeIndex = None

        # Identify which option has the best fitness using pythagoras
        for index in range(options_size):
            pythag = getDistance(options[index], end, False) 
            # + min( end[0] - options[index][0], end[1] - options[index][1] )
            if pythag < bestFitness:
                bestFitness = pythag
                bestNodeIndex = index

        # put remainder of items into the reserves for later use
        for index in range(options_size):
            if index != bestNodeIndex:
                reserves.append(options[index])

        addNode(options[bestNodeIndex], visited, areaSize, blockSize)
        pathResponse = mitad(options[bestNodeIndex])

        if pathResponse:
            addNode(options[bestNodeIndex], visited, areaSize, blockSize)
            path.append(options[bestNodeIndex])
            return True
        return False

    response = mitad(start)
    path.append(start)
    # stepDetails['steps'] = path[::-1] if response else []
    # stepDetails['index'] = 1 if response else len(path)+1
    # stepDetails['visited'] = visited
    return (path[::-1] if response else [], visited)
    error = False

    # return (path[::-1], visited)
