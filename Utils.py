import heapq
import copy
import sys

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0,0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def get_neighbors(curr_loc, my_map):
    next_locs = []
    for dir in range(5):
        next_loc = move(curr_loc, dir)
        
        # check is valid move
        if next_loc[0] >= 0 and next_loc[1] >= 0 and next_loc[0] < len(my_map[0]) and next_loc[1] < len(my_map) and my_map[next_loc[1]][next_loc[0]] == True:
            next_locs.append(next_loc)
    
    return next_locs

def compare(cord1, cord2):

    if cord1[0] == cord2[0] and cord1[1] == cord2[1]:
        return True

    return False

# find degree of each agent in collision graph
# input: path [ [path of agent1], [path of agent2] ...]
# [path of agent1] = [(1,2),(1,3),(1,4)...]
# return: list of degree of each agent
# ex. [1,2,3,4 ...]

def deg(path):
    degList = []
    deg = 0
    for firstPath in range(0, len(path), 1):
        for secondPath in range(firstPath + 1, len(path), 1):
            if(path[firstPath] == path[secondPath]):
                continue
            length = len(path[secondPath]) if len(path[firstPath]) > len(path[secondPath]) else len(path[firstPath])
            for timestep in range(0, length, 1):
                if compare(path[firstPath][timestep], path[secondPath][timestep])\
                    or (timestep > 0 and compare(path[firstPath][timestep-1], path[secondPath][timestep]) and compare(path[firstPath][timestep], path[secondPath][timestep-1])):
                    deg += 1
                    break
        degList.append(deg)
        deg = 0
    return degList

def degID(path):
    degList = []
    for firstPath in range(0, len(path), 1):
        for secondPath in range(firstPath + 1, len(path), 1):
            if(path[firstPath] == path[secondPath]):
                continue
            length = len(path[secondPath]) if len(path[firstPath]) > len(path[secondPath]) else len(path[firstPath])
            for timestep in range(0, length, 1):
                if compare(path[firstPath][timestep], path[secondPath][timestep]):
                    degList.append((path[firstPath][timestep], firstPath, secondPath))
                    break
                elif timestep > 0 and compare(path[firstPath][timestep-1], path[secondPath][timestep]) and compare(path[firstPath][timestep], path[secondPath][timestep-1]):
                    degList.append((path[firstPath][timestep], path[secondPath][timestep], firstPath, secondPath))
                    break
    return degList

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    i = 0
    while len(open_list) > 0:
        i += 1
        (cost, loc, curr) = heapq.heappop(open_list)
        #print(cost, loc, curr)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map[0]) \
                or child_loc[1] < 0 or child_loc[1] >= len(my_map):
                continue
            if not my_map[child_loc[1]][child_loc[0]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values