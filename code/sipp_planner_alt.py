import math
import heapq

#MIGHT KEEP THIS VERSION INSTEAD!!!!

class State_Map():
    ##############################
    #           Class for managing existing states in the search tree. Maintained by a dictionary with key/val pair (loc, interval): State Object
    #           Functions:
    #               get_state:   Checks if key already exists, if so returns the state, otherwise generates a new state with the passed values
    #               clear_state: If a state exists, removes it from the dict.         
    def __init__(self):
        self.state_dict = {}

    def get_state(self,cfg, interval, timestep):
        if (cfg, interval) not in self.state_dict:
            self.state_dict[(cfg, interval)] = {'cfg':cfg,
                                                'timestep':timestep,
                                                'interval':interval,}
        return self.state_dict[(cfg, interval)]
    
    def clear_state(self, loc, interval):
        if (loc, interval) in self.state_dict:
            self.state_dict.pop((loc, interval))

class CFG():
    ##############################
    #           Class for managing the safe interval list of each (x,y) location on the map, initially [(0, inf)]. We call this a configuration.
    #           The split function takes an interval on time (t, t + i) and splits the existing to each end of the interval passed.
    #           For example if the only interval in the list is (0,inf), it would be split into (0, t-1) and (t+i+1, inf)       
    def __init__(self, loc):
        self.loc = loc
        self.intervals = [(0, float('inf'))]
        self.g = float('inf')
        self.f = float('inf')

        self.parent_state = None
        
        
 

    def split(self, collision_interval):
        start, end = collision_interval
        if start > end:
            raise ValueError("Start of collision interval cannot be greater than end")
        if start < 0 or end < 0:
            raise ValueError("Intervals cannot be negative")
        i = 0
        while(i < len(self.intervals)):
            s,e = self.intervals[i]
            #if the collision interval is bigger than an entire interval
            if start <= s and end >= e:
                self.intervals.pop(i)
                continue
            elif start > s and start <= e and end >= e:
                e = start - 1
                if s <= e:
                    self.intervals[i] = (s,e)
                else:
                    self.intervals.pop(i)
                    continue
            elif end < e and end >= s and start <= s:
                s = end + 1
                if s <= e:
                    self.intervals[i] = (s,e)
                else:
                    self.intervals.pop(i)
                    continue
            elif start > s and end < e:
                #in this case we need to partition the interval
                partition_1 = (s, start - 1)
                partition_2 = (end + 1, e)

                self.intervals.insert(i, partition_1)
                self.intervals.insert(i+1, partition_2)
                self.intervals.pop(i+2)
                i+=2
                continue
            
            i += 1       

class CFG_MAP():
    ##############################
    #           Class for managing configurations. One alternative way would be to create a CFG Object for every (x,y) index at the start,
    #           but this allows us to generate configurations as they're needed. The cfg_dict has key value pair (x,y): CFG Object.
    #           The init function generates an empty dict, but we can also pass a list of unsafe intervals pre-defined, which is a dict with key value pair (x,y): [interval1, interval2...]
    #           that specifies locations with unsafe intervals. In this case we'd retrieve the configuration at (x,y) with get_cfg and split its intervals according to the intervals of the unsafe list.
    #           
    #           The get_cfg function simply checks if a configuration already exists in the dict at (x,y). If it does it's returned, if it doesn't a default (0, inf) interval is generated and returned
    def __init__(self, unsafe_intervals = {}):
        self.cfg_dict = {}
        if unsafe_intervals:
            for loc in unsafe_intervals:
                cfg = self.get_cfg(loc)
                for interval in unsafe_intervals[loc]:
                    cfg.split(interval)
            
    def get_cfg(self, loc):
        if loc not in self.cfg_dict:
            self.cfg_dict[loc] = CFG(loc)
        return self.cfg_dict[loc]
    
    
#Helper functions for managing the priority queue used in SIPP. The state_dat argument is a tuple ordered by (f, heuristic, loc, state)
def push_pq(open_list, state_dat):
    heapq.heappush(open_list, state_dat)
def pop_pq(open_list):
    _, _, _, curr_state = heapq.heappop(open_list)
    return curr_state

class SIPP():
    ##############################
    # Safe Interval Path Planner
    #           my_map               - List of lists specifying obstacle positions
    #           start_loc            - agent start position
    #           goal_loc             - agent goal position
    #           unsafe_interval_list - dict of unsafe intervals with key/value pair (x,y):[interval1, interval2...]
    #           Functions:
    #               get_heuristic:    Calculates the heuristic value for any (x,y) location.
    #                                 Simply defined as the straight line distance between the point and the goal location. 
    #                                 This is admissible and consistent, but there may be a better heuristic.
    #               
    #               move:             Move takes a given location and returns the adjacent locations which might be viable for generating successor states. 
    #                                 It uses the map to check whether an adjacent location overlaps an obstacle, or whether we've gone out of bounds.
    #                                 Valid directions are considered N,E,S,W
    #               
    #               get_path_sipp:    A* implementation using safe intervals. Works identically to usual a* implementations except for a few key differences.
    #                                 The search space is over the safe intervals of the configurations defined by each (x,y) location on the map.
    #                                 The code for this function, and for get_successors is taken from the paper by: (). Link: ()
    #               
    #               get_successors:   Function takes a state and returns a list of legitimate successor states. 
    #                                 To do this, it calculates the potential moves from the states location, finds the configration list for each of those moves, 
    #                                 and creates a successor for each of the configuration intervals if the interval overlaps with the parent states interval.
    #                                 To calculate the exact time of arrival for the successor state, we take the minimum t that the parent and successor nodes overlap on.
    #               
    #               get_cost:         The cost function is used in get_path_sipp to update the g value for each successor node.
    #                                 In this case, cost = the time difference between parent and successor
    #               
    #               updateTime:       This function finds a new minimal time for a successor node if it's relaxed (the algorithm found a shoter path to a successor that already exists)
    #               
    #               reconstruct_path: The function takes the node at the goal location and works back through parent nodes until it reaches the start location. 
    #                                 It's slightly different from ordinary A* as each State might correspond to more than one time interval, in which case we need the total time between a node and its parent.
                    
    def __init__(self, my_map, start_loc, goal_loc, h_vals, unsafe_interval_list):
        self.my_map = my_map
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.h_vals = h_vals

        self.cfg_map = CFG_MAP(unsafe_interval_list)
        self.state_map = State_Map()
      
    def move(self, loc):
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        valid_moves = []
        for i in range(len(directions)):
            new_loc = (loc[0] + directions[i][0], loc[1] + directions[i][1])
            if new_loc[0] < 0 or new_loc[0] >= len(self.my_map) or new_loc[1] < 0 or new_loc[1] >= len(self.my_map[0]):
                continue
            if self.my_map[new_loc[0]][new_loc[1]]:
                continue
            valid_moves.append(new_loc)
        return valid_moves
     
    def get_path_sipp(self):
        open_list = []
        
        root_cfg = self.cfg_map.get_cfg(self.start_loc)
        root_cfg.g = 0
        root_cfg.f = self.h_vals[self.start_loc]
        root_state = self.state_map.get_state(root_cfg, root_cfg.intervals[0], 0)
        push_pq(open_list, (root_cfg.f, self.h_vals[self.start_loc], self.start_loc, root_state))
        
  

        while len(open_list) > 0:
            curr_state = pop_pq(open_list)
            curr_cfg = curr_state['cfg']
            
            if curr_cfg.loc == self.goal_loc and curr_state['interval'][1] == float('inf'):
                print("Found path!")
                return self.reconstruct_path(curr_state)
            
            successors = self.get_successors(curr_state)
            for succ_state in successors:
                succ_cfg = succ_state['cfg']
                if succ_cfg.g > curr_cfg.g + self.get_cost(curr_state, succ_state):
                    succ_cfg.g = curr_cfg.g + self.get_cost(curr_state, succ_state)
                    succ_cfg.parent_state = curr_state
                    
                    self.updateTime(curr_state, succ_state)
                    succ_cfg.f = succ_cfg.g + self.h_vals[succ_cfg.loc]
                    push_pq(open_list, (succ_cfg.f, self.h_vals[succ_cfg.loc], succ_cfg.loc, succ_state))

        return None
    
    def get_successors(self, state):
        successors = []
        valid_moves = self.move(state['cfg'].loc)
        m_time = 1 
        
        for mov in valid_moves:
            cfg = self.cfg_map.get_cfg(mov)

            start_t = state['timestep'] + m_time
            end_t = state['interval'][1]  + m_time 
        
            for interval in cfg.intervals:
                if interval[0] > end_t or interval[1] < start_t:
                    continue
        
                t = max(start_t, interval[0])
                succ = self.state_map.get_state(cfg, interval, t)
                successors.append(succ)
        return successors
    
    def get_cost(self, state, succ_state):
        return abs(succ_state['timestep'] - state['timestep'])

    def updateTime(self, state, succ_state):
        start_t = state['timestep'] + 1
        t = max(start_t, succ_state['interval'][0])
        succ_state['timestep'] = t
       
    def reconstruct_path(self, state):
        path = []

        curr_state = state
        curr_cfg = curr_state['cfg']

        t = curr_state['timestep']
        path.append(curr_cfg.loc)

        while(curr_cfg.loc != self.start_loc):
            curr_state = curr_cfg.parent_state
            curr_cfg = curr_state['cfg']

            for i in range(curr_state['timestep'], t):
                path.append(curr_cfg.loc)
            t = curr_state['timestep']
        
        return path[::-1]
 





        