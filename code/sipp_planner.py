import math
import heapq

#MIGHT KEEP THIS VERSION INSTEAD!!!!

class State_Map():
    ##############################
    #           Class for managing existing states in the search tree. Maintained by a dictionary with key/val pair (loc, interval): Dict(State data)
    #           Functions:
    #               get_state:   Checks if key already exists, if so returns the state, otherwise generates a new state with the passed values
    #               clear_state: If a state exists, removes it from the dict.         
    def __init__(self):
        self.state_dict = {}

    def get_state(self, loc, timestep, interval):
        if (loc, interval) not in self.state_dict:
            self.state_dict[(loc, interval)] = {'loc':loc,
                                                'timestep':timestep,
                                                'interval':interval,
                                                'g':float('inf'),
                                                'f':float('inf'),
                                                'parent_state':None}
        return self.state_dict[(loc, interval)]
    
    def clear_state(self, loc, interval):
        if (loc, interval) in self.state_dict:
            self.state_dict.pop((loc, interval))


class CFG_MAP():
    ##############################
    #           Class for managing configurations indexed by (x,y) loc on the map. Maintained by a dictionary with key/val pair (loc): Dict(cfg data)
    #           Functions:
    #               init(): generates an empty dict, but we can also pass a list of unsafe intervals pre-defined, which is a dict with key value pair (x,y): [interval1, interval2...]
    #                       that specifies locations with unsafe intervals. In this case we'd retrieve the configuration at (x,y) with get_cfg and split its intervals according to the intervals of the unsafe list.
    #               get_cfg():  simply checks if a configuration already exists in the dict at (x,y). If it does it's returned, if it doesn't a default (0, inf) interval is generated and returned.
    #               split():    splits a configurations interval list. For example, if (t, t+z) is passed against a default interval list (0, float('inf')) we return  (0, t-1), (t+z+1, float('inf')) 
    def __init__(self, unsafe_intervals = {}):
        self.cfg_dict = {}
        if unsafe_intervals:
            for loc in unsafe_intervals:
                cfg = self.get_cfg(loc)
                for interval in unsafe_intervals[loc]:
                    self.split(cfg, interval)
            
    def get_cfg(self, loc):
        if loc not in self.cfg_dict:
            self.cfg_dict[loc] = {'intervals':[(0,float('inf'))]}
        return self.cfg_dict[loc]
    
    def split(self, cfg, collision_interval):
        start, end = collision_interval
        if start > end:
            raise ValueError("Start of collision interval cannot be greater than end")
        if start < 0 or end < 0:
            raise ValueError("Intervals cannot be negative")
        i = 0
        while(i < len(cfg['intervals'])):
            s,e = cfg['intervals'][i]
            #if the collision interval is bigger than an entire interval
            if start <= s and end >= e:
                cfg['intervals'].pop(i)
                continue
            elif start > s and start <= e and end >= e:
                e = start - 1
                if s <= e:
                    cfg['intervals'][i] = (s,e)
                else:
                    cfg['intervals'].pop(i)
                    continue
            elif end < e and end >= s and start <= s:
                s = end + 1
                if s <= e:
                    cfg['intervals'][i] = (s,e)
                else:
                    cfg['intervals'].pop(i)
                    continue
            elif start > s and end < e:
                #in this case we need to partition the interval
                partition_1 = (s, start - 1)
                partition_2 = (end + 1, e)

                cfg['intervals'].insert(i, partition_1)
                cfg['intervals'].insert(i+1, partition_2)
                cfg['intervals'].pop(i+2)
                i+=2
                continue
            
            i += 1

    
    
#Helper functions for managing the priority queue used in SIPP. The state_dat argument is a tuple ordered by (f, heuristic, loc, state)
def push_pq(open_list, state_dat):
    heapq.heappush(open_list, state_dat)
def pop_pq(open_list):
    _, _, _, curr_state = heapq.heappop(open_list)
    return curr_state

def move(my_map, loc):
    #Calculates valid move locations, cross references the map to make sure a move location is valid
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    valid_moves = []
    for i in range(len(directions)):
        new_loc = (loc[0] + directions[i][0], loc[1] + directions[i][1])
        if new_loc[0] < 0 or new_loc[0] >= len(my_map) or new_loc[1] < 0 or new_loc[1] >= len(my_map[0]):
            continue
        if my_map[new_loc[0]][new_loc[1]]:
            continue
        valid_moves.append(new_loc)
    return valid_moves

#Returns the heuristic value for loc, calculates from the single_agent_planner function
def get_heuristic(loc, h_vals):
    return h_vals[loc]
    
def get_path_sipp(my_map, start_loc, goal_loc, h_vals, unsafe_interval_list):
    #A* with SI, implements the required functionality of the algorithm psuedocode described in the paper
    open_list = []

    cfg_map = CFG_MAP(unsafe_interval_list)
    state_map = State_Map()

    
    root_cfg = cfg_map.get_cfg(start_loc)
    root_state = state_map.get_state(start_loc, 0, root_cfg['intervals'][0])
    root_state['g'] = 0
    root_state['f'] = get_heuristic(start_loc, h_vals)
    push_pq(open_list, (root_state['f'], root_state['interval'], start_loc, root_state))
    while len(open_list) > 0:
        curr_state = pop_pq(open_list)
        
        if curr_state['loc'] == goal_loc and curr_state['interval'][1] == float('inf'):
            print("Found path!")
            return reconstruct_path(curr_state, start_loc)
        
        successors = get_successors(my_map, cfg_map, state_map, curr_state)
        for succ_state in successors:
            if succ_state['g'] > curr_state['g'] + get_cost(curr_state, succ_state):
                succ_state['g'] = curr_state['g'] + get_cost(curr_state, succ_state)
                succ_state['parent_state']= curr_state
                
                updateTime(curr_state, succ_state)
                succ_state['f'] = succ_state['g'] + get_heuristic(succ_state['loc'], h_vals)
                push_pq(open_list, (succ_state['f'], succ_state['interval'], succ_state['loc'], succ_state))

    return None

#getSuccessor(s) function described in the paper. Makes sure potential successors intervals overlap with the parent successors interval
def get_successors(my_map, cfg_map, state_map, state):
    successors = []
    valid_moves = move(my_map, state['loc'])
    m_time = 1 
    
    for mov in valid_moves:
        cfg = cfg_map.get_cfg(mov)

        start_t = state['timestep'] + m_time
        end_t = state['interval'][1]  + m_time 
    
        for interval in cfg['intervals']:
            if interval[0] > end_t or interval[1] < start_t:
                continue
    
            t = max(start_t, interval[0])
            succ = state_map.get_state(mov, t, interval)
            successors.append(succ)
    return successors
#gets the diff between parent and child arrival time to add to the edge cost
def get_cost(state, succ_state):
    return abs(succ_state['timestep'] - state['timestep'])
#relaxes arrival time if a shorter path is found
def updateTime(state, succ_state):
    start_t = state['timestep'] + 1
    t = max(start_t, succ_state['interval'][0])
    succ_state['timestep'] = t
#reconstructs and returns the path
def reconstruct_path(state, start_loc):
    path = []

    curr_state = state

    t = curr_state['timestep']
    path.append(curr_state['loc'])

    while(curr_state['loc'] != start_loc):
        curr_state = curr_state['parent_state']

        for i in range(curr_state['timestep'], t):
            path.append(curr_state['loc'])
        t = curr_state['timestep']
    
    return path[::-1]






        