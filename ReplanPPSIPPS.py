from ALNS import *
from collisionneighbourhood import *
from failureBasedNeighbourhood import *
from randomNeighborhood import *
from Utils import *
from SIPPS import *
import time as timer

def prioritized_planning(paths, neighbourhood, instanceMap, instanceStarts, instanceGoals, directivesQueue):
    #randomize order of neighbourhood
    neighbourhood = priorityList(neighbourhood)

    #paths of agents in neighbourhood => soft obstacles
    #paths of agents not in neighbourhood => hard obstacles

    #build hard constraints table
    neighbourhood_set = set(neighbourhood)
    hard_obstacles = {}
    for i in range(len(paths)):
        if i not in neighbourhood_set:
            add_constraints_from_path(hard_obstacles, paths[i])

    #execute prioritized planning
    # at each iteration, add to soft obstacles using found path
    newPaths = []
    soft_obstacles = {}
    for agent in neighbourhood:
        agentStart = instanceStarts[agent] #coordinates are in (x, y), map indexing is in [y][x]
        agentGoal = instanceGoals[agent]
        directivesQueue.append(['pathing for agent', agent])

        #build heuristics table
        h_values = compute_heuristics(instanceMap, agentGoal)
        agentPath = sipps(instanceMap, agentStart, agentGoal, h_values, hard_obstacles, soft_obstacles, directivesQueue)
        directivesQueue.append(['path done', agent, agentPath])
        newPaths.append(agentPath)
        if agentPath != None:
            add_constraints_from_path(soft_obstacles, agentPath)
    return neighbourhood, newPaths
    #newPaths[i] corresponds to agent at neighbourhood[i]


def add_constraints_from_path(constraint_table, path):
    #add vertex constraint for time 0
    if path[0] not in constraint_table:
        constraint_table[path[0]] = []
    heapq.heappush(constraint_table[path[0]], 0)

    for i in range(1, len(path)):
        #add vertex constraint
        if path[i] not in constraint_table:
            constraint_table[path[i]] = []
        heapq.heappush(constraint_table[path[i]], i)

        #add edge constraint
        if (path[i], path[i-1]) not in constraint_table:
            constraint_table[(path[i], path[i-1])] = []
        heapq.heappush(constraint_table[(path[i], path[i-1])], i)

    return constraint_table


def priorityList(neighbourhood):
    PPlist = []
    while len(neighbourhood) > 0:
        index = randrange(0, len(neighbourhood))
        PPlist.append(neighbourhood.pop(index))
    return PPlist


# replan untill collision free
def replan(paths, numNeighbourhood, instanceMap, instanceStarts, instanceGoals, ALNS_weight, prevCP, directivesQueue):
    # select a neighbourhood construction method

    # 0: collision, 1: failure, 2: random
    neighbourhood_kind = ALNS(ALNS_weight)

    if neighbourhood_kind == 0:
        neighbourhood = collisionNeighbourhood(paths, numNeighbourhood, instanceMap)
        directivesQueue.append(['neighbourhood', 'collision', neighbourhood, ALNS_weight])
    elif neighbourhood_kind == 1:
        neighbourhood = failureNeighbourhood(paths, numNeighbourhood)
        directivesQueue.append(['neighbourhood', 'failure', neighbourhood, ALNS_weight])
    else:
        neighbourhood = randomNeighbourhood(paths, numNeighbourhood)
        directivesQueue.append(['neighbourhood', 'random', neighbourhood, ALNS_weight])

    #run modified prioritized planning to get replanned paths
    neighbourhood, newPaths = prioritized_planning(paths, neighbourhood, instanceMap, instanceStarts, instanceGoals, directivesQueue)

    #construct new paths solution and update variables
    newPathsSolution = copy.copy(paths)
    for i in range(len(neighbourhood)):
        if newPaths[i] != None:
            newPathsSolution[neighbourhood[i]] = newPaths[i]

    numCp_newPathsSolution = sum(deg(newPathsSolution))

    ALNS_weight = updateWeight(ALNS_weight, 0.1, prevCP, numCp_newPathsSolution, neighbourhood_kind)

    if (prevCP >= numCp_newPathsSolution):
        directivesQueue.append(['improved solution'])
        return newPathsSolution, numCp_newPathsSolution

    directivesQueue.append(['no improvement'])
    return paths, prevCP


def LNS2PP(numNeighbourhood, instanceMap, instanceStarts, instanceGoals, directivesQueue):

    paths = list(range(len(instanceGoals)))
    neighbourhood, newPaths = prioritized_planning([], list(range(len(instanceGoals))), instanceMap, instanceStarts, instanceGoals, directivesQueue)
    for i in range(len(neighbourhood)):
        paths[neighbourhood[i]] = newPaths[i]
    directivesQueue.append(['initial paths'])
    

    collisions = degID(paths)
    numCp = 0
    numCp = len(collisions)

    startTime = timer.time_ns()
    if (numCp == 0):
        directivesQueue.append('done')
        return paths, startTime, 0

    ALNS_weight = [1, 1, 1]
    ALNS_r = 0.1

    replan_counter = 0
    while numCp != 0:
        paths, numCp = replan(paths, numNeighbourhood, instanceMap, instanceStarts, instanceGoals, ALNS_weight, numCp, directivesQueue)
        replan_counter += 1
        if paths == None:
            directivesQueue.append('done')
            return None, None, None
    
    directivesQueue.append('done')
    return paths, startTime, replan_counter

# if __name__ == "__main__":
#     numNeighbourhood = 5
#     numAgent = 10
#     instanceMap, instanceStarts, instanceGoals = loadScen(
#         'room-32-32-4-even-1.scen', numAgent)
#     paths = LNS2(numNeighbourhood, len(instanceMap[0]), len(instanceMap), instanceMap,
#                 instanceStarts, instanceGoals)

#     for path in paths:
#         print(path)

