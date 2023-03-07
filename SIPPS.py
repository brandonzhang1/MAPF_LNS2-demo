import heapq
from Utils import *

# safe_interval_table should look like this:
#       
#       safe_interval_table[location][interval_id] = [low, high)
#       Ditctionary that is consists of: 
#                        safe_interval_table = {
#                                                   'location1': {
#                                                                   [(low, high), (low2, high2), (low3, high3)...]   # id is the position of each interval (they are in chronological order)
#                                                                 },
#                                                   'location2': {
#                                                                   [(low, high), (low2, high2), (low3, high3)...],    

def build_safe_interval_table(my_map, hard_obstacles, soft_obstacles):

    safe_interval_table = dict()
    locations = []
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if my_map[i][j] == True:
                locations.append((j, i))

    for v in locations:
        intervals = []

        hard_times = []
        if v in hard_obstacles:
            hard_times = copy.copy(hard_obstacles[v])
        soft_times = []
        if v in soft_obstacles:
            soft_times = copy.copy(soft_obstacles[v])

        next_time = 0
        next_time_from = 0
        start_from = 0 #0:clear, 1:soft, 2:hard
        interval_low = 0 
        interval_top = 0 #include top time in interval, ie. [low, top] == [low, top+1) == [low, high)

        if len(hard_times) + len(soft_times) == 0:
            intervals.append((0, sys.maxsize, 0))
        else:
            while len(hard_times) + len(soft_times) > 0:
                #get next time and from which heap
                if len(hard_times) == 0:
                    next_time = heapq.heappop(soft_times)
                    next_time_from = 1
                elif len(soft_times) == 0:
                    next_time = heapq.heappop(hard_times)
                    next_time_from = 2
                elif soft_times[0] < hard_times[0]:
                    next_time = heapq.heappop(soft_times)
                    next_time_from = 1
                else:
                    next_time = heapq.heappop(hard_times)
                    next_time_from = 2


                if next_time_from == start_from:

                    if next_time == interval_top + 1: #direct interval continuation
                        interval_top = next_time
                        continue #skip next interval start setup

                    else: #interval gap, obstacle interval ended and intermediate empty interval inferred
                        #if soft interval add [low, top+1), if hard interval do nothing
                        if start_from == 1:
                            intervals.append((interval_low, interval_top+1, 1))
                        
                        #add empty interval
                        intervals.append((interval_top+1, next_time, 0))

                else: #next_time_from != start_from, automatic interval demarcation
                    if start_from == 0: #only happens on first loop iteration setup
                        if next_time != 0: #[0, next_time) is clear, add clear interval, set up interval tracking variables
                            intervals.append((0, next_time, 0))    

                    if start_from == 1: #after first loop
                        intervals.append((interval_low, interval_top+1, start_from))
                    
                    if next_time > interval_top + 1: #intermediate clear interval
                        intervals.append((interval_top+1, next_time, 0))

                # next interval start setup
                start_from = next_time_from
                interval_low = next_time
                interval_top = next_time

            #add interval for last time sequence if soft
            if start_from == 1:
                intervals.append((interval_low, interval_top+1, 1))
            #add interval for last clear to infinity
            intervals.append((interval_top+1, sys.maxsize, 0))

        safe_interval_table[v] = intervals

    return safe_interval_table

def insert_node(node, open_list, key_heap, visited_list, h_values, soft_obstacle, count_tie_break, newOpen):
    h_val = node['h_val']
    c_val = node['c_val']
    f_val = c_val + h_val
    
    
    n_low = node['interval'][0]
    n_high = node['interval'][1]

    node_sig = (node['loc'], node['id'], node['is_goal'])
    node_sig_2 = (c_val, n_low, n_high)

    identical_nodes = []

    if node_sig in visited_list:
        identical_nodes = list(visited_list[node_sig].values())

    for i_node in identical_nodes:
        i_low = i_node[0]['interval'][0]
        i_high = i_node[0]['interval'][1]
        i_c_val = i_node[0]['c_val']
        i_node_sig_2 = (i_c_val, i_low, i_high)
        
        if i_low <= n_low and i_c_val <= c_val: #previous node is dominant, don't insert node
            return
        
        elif n_low <= i_low and c_val <= i_c_val: #node is better than previously added node, remove i_node
            if i_node[1] in open_list:
                open_list.pop(i_node[1])
            visited_list[node_sig].pop(i_node_sig_2)

        elif n_low < i_high and i_low < n_high: #node has different complementary times, update to disjoint intervals and add
            if n_low < i_low:
                n_high = i_low
            else:
                updated_i_node = copy.copy(visited_list[node_sig][i_node_sig_2][0])
                updated_i_node['interval'] = (updated_i_node['interval'][0], n_low)

    combined_heuristic = (f_val, c_val)
    tie_break = 0
    if combined_heuristic in count_tie_break:
        tie_break = count_tie_break[combined_heuristic]
        count_tie_break[combined_heuristic] -= 1
    else:
        tie_break = 0
        count_tie_break[combined_heuristic] = -1
    key = (f_val, c_val, tie_break)

    heapq.heappush(key_heap, key)
    open_list[key] = node
    newOpen.add(node['loc'])
    if node_sig not in visited_list:
        visited_list[node_sig] = {}
    if node_sig_2 not in visited_list[node_sig]:
        visited_list[node_sig][node_sig_2] = {}
    visited_list[node_sig][node_sig_2] = (node, key)

def get_valid_nodes(my_map, curr_loc, low, high, safe_interval_table):
    
    valid_neighbors = []

    neighbor_locations = get_neighbors(curr_loc, my_map)
    
    for next_loc in neighbor_locations:
        #print(next_loc, my_map[next_loc[1]][next_loc[0]])
        for i in range(len(safe_interval_table[next_loc])):
            interval = safe_interval_table[next_loc][i]
        #for interval_id, interval in enumerate(safe_interval_table[next_loc]):  
            #find safe intervals that overlap with current node's interval
            if interval[0] <= high and interval[0] >= low + 1 \
                or interval[1] <= high + 1 and interval[1] > low + 1 \
                or low + 1 >= interval[0] and low + 1 < interval[1] \
                or high >= interval[0] and high < interval[1]:
                valid_neighbors.append((next_loc, i))
    
    return valid_neighbors

def get_earliest_arrival_time(curr_loc, edge, low, high_limit, hard_obstacle, soft_obstacle):
    hard_edge_times = set()
    soft_edge_times = set()

    if hard_obstacle != None and edge in hard_obstacle:
        hard_edge_times = hard_obstacle[edge]

    if soft_obstacle != None and edge in soft_obstacle:
        soft_edge_times = soft_obstacle[edge]

    new_low = low
    while new_low < high_limit:
        if new_low in hard_edge_times or new_low in soft_edge_times:
            new_low += 1
        else:
            return new_low
    return None

def expand_node(my_map, curr_node, open_list, key_heap, visited_list, safe_interval_table, hard_obstacle, soft_obstacle, h_values, count_tie_break, newOpen, newClosed):
    valid_neighbors = []
    curr_loc = curr_node['loc']
    node_low = curr_node['interval'][0]
    node_high = curr_node['interval'][1]
    if curr_loc in newOpen:
        newOpen.remove(curr_loc)
    newClosed.add(curr_loc)

    valid_neighbors = get_valid_nodes(my_map, curr_loc, node_low, node_high, safe_interval_table)
    #print("valid_neighbors", valid_neighbors)
    # Algorithm 2 line 2-3
    for (next_loc, interval_id) in valid_neighbors:
        low = safe_interval_table[next_loc][interval_id][0]
        high = safe_interval_table[next_loc][interval_id][1]
        interval_type = safe_interval_table[next_loc][interval_id][2]

        high_limit = min(node_high+1, high)
        low_limit = max(node_low+1, low)

        earliest_low = get_earliest_arrival_time(curr_loc, (curr_loc, next_loc), low_limit, high_limit, hard_obstacle, soft_obstacle) # uncolide with hard, soft obstacle

        if earliest_low != None:
            #add node for next_loc with [earliest_low, high) as interval
            #check for number of collisions during transit time
            # if curr_node is in soft obstacle interval, c_val += earliest_low - node_low
            c_val = curr_node['c_val']
            #temp = safe_interval_table[curr_node['loc']][curr_node['id']]
            if safe_interval_table[curr_node['loc']][curr_node['id']][2] == 1:
                c_val += earliest_low - node_low
            else:
                c_val += 1
            next_node = {'loc': next_loc, 'c_val': c_val, 'h_val': h_values[next_loc], 
                        'interval': (earliest_low, high), 'id': interval_id, 'is_goal': False, 
                        'parent': curr_node}
            insert_node(next_node, open_list, key_heap, visited_list, h_values, soft_obstacle, count_tie_break, newOpen)



def get_c_future(curr_loc, soft_obstacle, n_low):
    c_future = 0
    
    if curr_loc in soft_obstacle:
        temp_times = copy.copy(soft_obstacle[curr_loc])
        
        while len(temp_times) > 0:
            time = heapq.heappop(temp_times)
            if time > n_low:
                c_future = len(temp_times) + 1    # Since all of rest of times in temp_times will be greater than n_low from now, 
                break                             # thus len(temp_times) + 1 (+1 for counting popped time just now).
    
    return c_future

def get_path(goal_node):
    #start from goal node going up to parent node
    #add location of goal node
    path = [goal_node['loc']]
    prevNode = goal_node
    node = goal_node['parent']
    while node != None:
        time = prevNode['interval'][0] - node['interval'][0]
        for i in range(time):
            path.append(node['loc'])
        prevNode = node
        node = node['parent']
    path.reverse()
    return path

def sipps(my_map, start_loc, goal_loc, h_values, hard_obstacle, soft_obstacle, directivesQueue):    

    safe_interval_table = build_safe_interval_table(my_map, hard_obstacle, soft_obstacle)  #my_map is avaialble paths excluding walls
    #print(safe_interval_table)

    root = {'loc': start_loc, 'c_val': 0, 'h_val': h_values[start_loc], 'interval': safe_interval_table[start_loc][0], 'id': 0, 'is_goal': False, 'parent': None}
    lower_bound_timestep = 0

    if goal_loc in hard_obstacle:
        lower_bound_timestep = max(hard_obstacle[goal_loc]) + 1

    key_heap = []
    count_tie_break = {}
    count_tie_break[(0, 0)] = -1 #f_val, c_val    
    open_list = {}
    # (f_val, c_val, -tiebreak) tiebreak: later nodes of same value first
    heapq.heappush(key_heap, (0, 0, 0))
    open_list[(0, 0, 0)] = root
    visited_list = {}
    visited_list[(root['loc'], root['id'], root['is_goal'])] = {}
    visited_list[(root['loc'], root['id'], root['is_goal'])][(root['c_val'], root['interval'][0], root['interval'][1])] = (root, (0, 0, 0))

    last_c_val = 0
    newOpen = set()
    newClosed = set()
    isocostContours = []

    closed_list = []
    while len(key_heap) > 0:
        currKey = heapq.heappop(key_heap)
        if currKey not in open_list:
            continue
        curr = open_list[currKey]
        #print(curr['c_val'])
        if curr['c_val'] > last_c_val:
            #print(len(key_heap))
            isocostContours.append((newOpen, newClosed))
            newOpen = set()
            newClosed = set()

        if curr['is_goal']:
            directivesQueue.append(['isocost contours', isocostContours])
            return get_path(curr)

        if curr['loc'] == goal_loc and curr['interval'][0] >= lower_bound_timestep:
            c_future = get_c_future(curr['loc'], soft_obstacle, curr['interval'][0])
            
            if c_future == 0:
                directivesQueue.append(['isocost contours', isocostContours])
                return get_path(curr)

            updated_node = curr.copy()
            updated_node['is_goal'] = True
            updated_node['c_val'] = curr['c_val'] + c_future
            insert_node(updated_node, open_list, key_heap, visited_list, h_values, soft_obstacle, count_tie_break, newOpen)

        expand_node(my_map, curr, open_list, key_heap, visited_list, safe_interval_table, hard_obstacle, soft_obstacle, h_values, count_tie_break, newOpen, newClosed)
        last_c_val = curr['c_val']


        
    # If there is no solution, return None to track of agents who does not have solutions when finding initial paths
    return None