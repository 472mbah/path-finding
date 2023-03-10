from base64 import encode
import sys
import math
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")
from mg2 import runMagnetic
import time
from progress.bar import Bar
from astart import runAstar, build2DMap, identifyOpenNodes
from utility.util import getAreaMeta, convertIntoTupleKeysArray, send, sendRaw
from timeit import default_timer as timer
from algorithms.automate.efficiency import buildBlockMaze as efficiencyGrid
from algorithms.automate.optimality import produceGrid as optimalityGrid
from algorithms.automate.robustness import produceRandomMaze
from algorithms.magnetic import runMitad
from algorithms.mg5 import heuristic as heuristicMagnetic
from algorithms.mg4 import heuristic
from algorithms.mg6 import heuristic as heuristicAreaSize
from algorithms.lee import lee_algorithm
from algorithms.a2 import astar_algorithm
from utility.turn import getDistance
import secrets
import tracemalloc


"""
Robustness ->   Tests on area with 5% randomised obstacles, 10% randomised obstalces and 20% randomised obstacles
                x100 times
                -> produceRandomMaze 

efficiency ->   single route, get to it the quickest (Single maze, just )
                x100 times
                -> efficiencyGrid
                (time to get the solution / sum of total times) x 100
                ( path length / visited nodes ) x 100

optimality ->   Finding the shortest route
                x100 times
                -> optimalityGrid

practicality -> Least amount of right turns and average distance of 
                every point of the graph from the closest obstacle
                Obstacles within a 1 block distances
                
                number right turns : average distance from obstacles 
                
                less battery consumed (something to investigate)

            practicality measured on the robustness and optimality tests

Do a test for a 3 dimensional space ->

"""

areaSize = 2
blockSize = 1
gridSize = 50

# (start, end, barriers) = produceRandomMaze(15, gridSize)
# (start, end, barriers) = efficiencyGrid(gridSize)
(start, end, barriers) = optimalityGrid(gridSize)


def decodeCord (cord, cordSplitter="::"):
    arr = [ int(x) for x in cord.split(cordSplitter)]
    if len(arr)==2:
        return tuple(arr)
    return (float('inf'), float('inf'))

def decodeArray (arr, cordListSplitter=":::", cordSplitter="::"):
    return [ decodeCord(x, cordSplitter) for x in arr.split(cordListSplitter)] 

def encodeCord (cord, cordSplitter="::"):
    try:
        return str(cord[0])+cordSplitter+str(cord[1])
    except:
        True
        return ""

def findClosestObstacle (cord, obstacles):
    keys = list(obstacles)
    min_ = float('inf')
    for ob in keys:
        distance = getDistance(ob, cord, pythagoras=False)
        min_ = min(min_, distance)
    
    return min_

def calculateAverageDistanceFromObstacles (path, obstacles):
    distances = [1, 2]
    for cord in path:
        distances.append(findClosestObstacle(cord, obstacles))
    
    # distances.sort()
    size = len(distances)
    return sum(distances)/size
    # [math.floor(size/2)]


def encodeArray (arr, cordListSplitter=":::", cordSplitter="::"):
    return cordListSplitter.join( [ encodeCord(cord, cordSplitter) for cord in arr ] ) 

def findRightAngleTurns (path):
    numberOfTurns = 0
    for k in range(2, len(path)):
        current = path[k]
        prev = path[k-2]
        difference = (abs(current[0]-prev[0]), abs(current[1]-prev[1]))
        if difference[0] >= 1 and difference[1] >= 1:
            numberOfTurns += 1
    return numberOfTurns


def commentResults (results):
    # print('--------------------------------------------------------------------\n')
    for key in results:
        if key == 'path':
            continue
        print(key.upper()+': '+( str(results[key])))
    print('--------------------------------------------------------------------')


def runSingleTest (algorithm='magnetic', start=(0, 0), end=(1, 1), barriers={}, areaSize=2, blockSize=1, gridSize=100, comment=True):
    originalBarriers = barriers.copy()
    map_ = build2DMap(gridSize, barriers) if algorithm != 'magnetic' else {}
    path = []
    
    barriers = getAreaMeta(barriers, areaSize) if algorithm == 'magnetic' or algorithm == 'lee' or algorithm == 'mg6' else barriers

    if algorithm == 'a2':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        ( path, visits ) = astar_algorithm(originalBarriers, start, end, maxIterations=1000000)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    elif algorithm == 'lee':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        ( path, visits ) = lee_algorithm(originalBarriers, start, end, maxIterations=1000000)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    elif algorithm == 'mg6':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        ( path, visits ) = heuristicAreaSize(start, end, barriers, areaSize, blockSize)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    elif algorithm == 'mg5':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        ( path, visits, message ) = heuristicMagnetic(start, end, barriers)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    elif algorithm == 'mg4':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        ( path, visits ) = heuristic(start, end, barriers)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    elif algorithm == 'magnetic':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        ( path, visits ) = runMagnetic(start, end, barriers, areaSize, blockSize, True)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        visits = convertIntoTupleKeysArray(visits)
    elif algorithm == 'astar':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        (path, nodes) = runAstar(start, end, map_)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        visits = identifyOpenNodes(nodes)
    elif algorithm == 'recursive_bfs':
        tracemalloc.start()
        tracemalloc.take_snapshot()
        startTime = timer()
        (path, visits) = runMitad(start, end, barriers, areaSize, blockSize)
        endTime = timer()
        (_, maxMemory) = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    else:
        startTime = 0
        endTime = 0
    if path == -1 or path == None or ( type(path)==list and not len(path) ):
        return None

    averageBlockersDistance = calculateAverageDistanceFromObstacles(path, originalBarriers)
    # stepDetails = { 'steps':path, 'visited':{}, 'end':end }
    # stepDetails = { 'steps':path, 'visited':{}, 'end':end }
    # sendRaw(stepDetails, (0,0,0,(25,25)), originalBarriers, True)

    # send(stepDetails, (0,0,0,(25,25)), getAreaMeta(barriers, areaSize) if algorithm != 'magnetic' else barriers)
    # time.sleep(2)
    rightAngleTurns = findRightAngleTurns(path)
    duration = endTime - startTime
    results = { 'maxMemory':maxMemory, 'averageDistanceFromBlockers':averageBlockersDistance, 'numberOfRightAngleTurns':rightAngleTurns, 'visitExcess':max(0, len(visits)-len(path)), 'path':path, 'duration':str(duration), 'pathSize':len(path), 'algorithm':algorithm }
    
    if comment:
        commentResults(results)
    return results


"""
    
    testType:
        robustness
        efficiency
        optimality

    algorithms:
        leealgorithm
        magnetic
        astar
        heuristiclee

"""

def runMultipleTest (testType='robustness', robustnessPercentage=5, gridSize=40, areaSize=2, blockSize=1, extension=''):
    foundBarriers = {}
    # , 'heuristiclee', 'leealgorithm'
    # algorithms = ['magnetic', 'astar', 'lee']
    algorithms = ['mg4', 'mg5', 'lee', 'a2']
    # 
    iteration = 100
    metrics = [ 'algorithm', 'numberOfRightAngleTurns', 'visitExcess', 'path', 'duration', 'pathSize' ]
    csv = ','.join(metrics)
    csv+=',start,end,barrierID,roundID'
    bar = Bar('Processing', max=iteration)
    while iteration > 0:
        
        if testType == 'robustness':
            (start, end, barriers) = produceRandomMaze(robustnessPercentage, gridSize)
        elif testType == 'efficiency':
            (start, end, barriers) = efficiencyGrid(gridSize)
        else:
            (start, end, barriers) = optimalityGrid(gridSize)

        startText = encodeCord(start)
        endText = encodeCord(end)

        barrierString = list(barriers)
        barrierString.sort()
        barrierString = tuple(barrierString)

        if barrierString in foundBarriers:
            continue
        barrierID = secrets.token_urlsafe(16)
        foundBarriers[barrierString] = barrierID
        tryAgain = False
        for alg in algorithms:
            try:
                results = runSingleTest (alg, start, end, barriers.copy(), areaSize, blockSize, gridSize, False)
            except:
                results = None
                continue
            if results != None:
                
                if len(results['path']) <= 1:
                    tryAgain = True

                results['path'] = encodeArray(results['path'])
                row = ','.join( [ str(results[field]) for field in metrics ] )
                extension = ','.join([startText, endText, barrierID, str(iteration)])
                row += ','+extension

                csv+='\n'+row
            else:
                continue
        if tryAgain:
            continue
        iteration -= 1
        bar.next()
    bar.finish()

    with open(f'./results/{testType}_{extension}.csv', 'w+') as f:
        f.write(csv)
    with open(f'./results/{testType}_maps.csv', 'w+') as f:
        plain = "id,map"
        for map in foundBarriers:
            plain += f'\n{foundBarriers[map]},{encodeArray(map)}'
        f.write(plain)        

# runMultipleTest(testType='robustness', extension='Test101')
runMultipleTest(testType='optimality', gridSize=gridSize, extension='Test101') #WHY?
# runMultipleTest(testType='efficiency', extension='Test101')
# runMultipleTest(testType='optimality')
# runSingleTest('mg4', start, end, barriers, areaSize, blockSize, gridSize)
# time.sleep(5)

# runSingleTest('mg4', start, end, barriers, areaSize, blockSize, gridSize)
# runSingleTest('mg5', start, end, barriers, areaSize, blockSize, gridSize)
# runSingleTest('lee', start, end, barriers, areaSize, blockSize, gridSize)
# runSingleTest('a2', start, end, barriers, areaSize, blockSize, gridSize)


# runSingleTest('astar', start, end, barriers, areaSize, blockSize, gridSize)
# runSingleTest('lee', (0, 0), (10, 10), {}, areaSize, blockSize, 10)
# runSingleTest('magnetic', start, end, barriers, {}, areaSize, blockSize, gridSize)


