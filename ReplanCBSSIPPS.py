from CBSSIPPS import *
from ALNS import *
from collisionneighbourhood import *
from failureBasedNeighbourhood2 import *
from randomNeighborhood import *
from prioritizedPlanning import *
from Utils import *
from pathlib import Path
from SIPPS import *
import time as timer

# replan untill collision free
def replan(paths, numNeighbourhood, instanceMap, instanceStarts, instanceGoals, ALNS_weight, prevCP, timeLimit, start):
    # select a neighbourhood construction method
        
    # 0: collision, 1: failure, 2: random
    neighbourhood_kind = ALNS(ALNS_weight)

    neighbourhood = []
    if neighbourhood_kind == 0:
        print('collisionNeighbourhood')
        neighbourhood = collisionNeighbourhood(paths, numNeighbourhood, instanceMap)
    elif neighbourhood_kind == 1:
        print('failureNeighbourhood')
        neighbourhood = failureNeighbourhood(paths, numNeighbourhood)
    else:
        print('randomNeighbourhood')
        neighbourhood = randomNeighbourhood(paths, numNeighbourhood)

    #run modified prioritized planning to get replanned paths
    #neighbourhood, newPaths = prioritized_planning(paths, neighbourhood, instanceMap, instanceStarts, instanceGoals)

    neighbourhoodStarts = []
    neighbourhoodGoals = []
    for i in range(len(neighbourhood)):
        neighbourhoodStarts.append(instanceStarts[neighbourhood[i]])
        neighbourhoodGoals.append(instanceGoals[neighbourhood[i]])

    cbs = CBSSolver(instanceMap, neighbourhoodStarts, neighbourhoodGoals, paths, neighbourhood)
    newPaths = cbs.find_solution(timeLimit, start)

    if newPaths == None:
        return None, None

    #construct new paths solution and update variables
    newPathsSolution = copy.copy(paths)
    for i in range(len(neighbourhood)):
        if newPaths[i] != None:
            newPathsSolution[neighbourhood[i]] = newPaths[i]

    numCp_newPathsSolution = sum(deg(newPathsSolution))

    ALNS_weight = updateWeight(ALNS_weight, 0.1, prevCP, numCp_newPathsSolution, neighbourhood_kind)

    if (prevCP >= numCp_newPathsSolution):
        return newPathsSolution, numCp_newPathsSolution

    return paths, prevCP


def LNS2CBS(numNeighbourhood, instanceMap, instanceStarts, instanceGoals, timeLimit):
    paths = list(range(len(instanceGoals)))
    neighbourhood, newPaths = prioritized_planning([], list(range(len(instanceGoals))), instanceMap, instanceStarts, instanceGoals)
    for i in range(len(neighbourhood)):
        paths[neighbourhood[i]] = newPaths[i]

    numCp = 0
    numCp = sum(deg(paths))
    start = timer.time_ns()
    if (numCp == 0):
        return paths, start, 0

    ALNS_weight = [1, 1, 1]
    ALNS_r = 0.1

    replan_counter = 0
    while numCp != 0:
        paths, numCp = replan(paths, numNeighbourhood, instanceMap, instanceStarts, instanceGoals, ALNS_weight, numCp, timeLimit, start)
        replan_counter += 1
        if paths == None:
            return None, None, None

    return paths, start, replan_counter

'''
if __name__ == "__main__":
    numNeighbourhood = 5
    numAgent = 10
    instanceMap, instanceStarts, instanceGoals = loadScen('room-32-32-4-even-11.scen', numAgent)
    paths = LNS2(numNeighbourhood, len(instanceMap[0]), len(instanceMap), instanceMap,
                instanceStarts, instanceGoals)
    print("solution")
    for path in paths:
        print(path)
'''