import math
import heapq



class State():
    ##############################
    #           Class for managing Nodes (labelled States) for the safe interval search.
    #           As the Search space is over 'safe' time intervals the fields that uniquely identify a state are (loc, interval)
    #           The 'timestep' property is calculated by taking the minimum t of overlap between a State's interval and a parent's interval
    def __init__(self, loc = (-1,-1), timestep = 0, interval = (0, float('inf'))):
        self.loc = loc
        self.timestep = timestep
        self.interval = interval

        self.g = float('inf')
        self.f = float('inf')
        self.parent_state = None
 
class State_Map():
    ##############################
    #           Class for managing existing states in the search tree. Maintained by a dictionary with key/val pair (loc, interval): State Object
    #           Functions:
    #               get_state:   Checks if key already exists, if so returns the state, otherwise generates a new state with the passed values
    #               clear_state: If a state exists, removes it from the dict.         
    def __init__(self):
        self.state_dict = {}

    def get_state(self, loc, timestep, interval):
        if (loc, interval) not in self.state_dict:
            self.state_dict[(loc, interval)] = State(loc, timestep, interval)
        return self.state_dict[(loc, interval)]
    
    def clear_state(self, loc, interval):
        if (loc, interval) in self.state_dict:
            self.state_dict.pop((loc, interval))

class CFG():
    ##############################
    #           Class for managing the safe interval list of each (x,y) location on the map, initially [(0, inf)]. We call this a configuration.
    #           The split function takes an interval on time (t, t + i) and splits the existing to each end of the interval passed.
    #           For example if the only interval in the list is (0,inf), it would be split into (0, t-1) and (t+i+1, inf)       
    def __init__(self):
        self.intervals = [(0, float('inf'))]
 

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
            self.cfg_dict[loc] = CFG()
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
                    
    def __init__(self, my_map, start_loc, goal_loc, unsafe_interval_list):
        self.my_map = my_map
        self.start_loc = start_loc
        self.goal_loc = goal_loc

        self.cfg_map = CFG_MAP(unsafe_interval_list)
        self.state_map = State_Map()
 
    def get_heuristic(self, loc):
        return math.sqrt(pow(self.goal_loc[0] - loc[0], 2) + pow(self.goal_loc[1] - loc[1], 2))
      
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
        root_state = self.state_map.get_state(self.start_loc, 0, root_cfg.intervals[0])
        root_state.g = 0 
        root_state.f = root_state.g + self.get_heuristic(root_state.loc)

        push_pq(open_list, (root_state.f, self.get_heuristic(root_state.loc), root_state.loc, root_state))
        
  

        while len(open_list) > 0:
            curr_state = pop_pq(open_list)
            self.state_map.clear_state(curr_state.loc, curr_state.interval)
            
            if curr_state.loc == self.goal_loc and curr_state.interval[1] == float('inf'):
                print("Found path!")
                return self.reconstruct_path(curr_state)
            
            successors = self.get_successors(curr_state)
            for succ_state in successors:
                if succ_state.g > curr_state.g + self.get_cost(curr_state, succ_state):
                    succ_state.g = curr_state.g + self.get_cost(curr_state, succ_state)
                    succ_state.parent_state = curr_state
                    
                    self.updateTime(curr_state, succ_state)
                    succ_state.f = succ_state.g + self.get_heuristic(succ_state.loc)
                    push_pq(open_list, (succ_state.f, self.get_heuristic(succ_state.loc), succ_state.loc, succ_state))

        return None
    
    def get_successors(self, state):
        successors = []
        valid_moves = self.move(state.loc)
        m_time = 1 
        
        for mov in valid_moves:
            cfg = self.cfg_map.get_cfg(mov)


            start_t = state.timestep + m_time
            end_t = state.interval[1]  + m_time 
        
            for interval in cfg.intervals:
                if interval[0] > end_t or interval[1] < start_t:
                    continue
        
                t = max(start_t, interval[0])
                succ = self.state_map.get_state(mov, t, interval)
                successors.append(succ)
        return successors
    
    def get_cost(self, state, succ_state):
        return abs(succ_state.timestep - state.timestep)

    def updateTime(self, state, succ_state):
        start_t = state.timestep + 1
        t = max(start_t, succ_state.interval[0])
        succ_state.timestep = t
       
    def reconstruct_path(self, state):
        path = []

        curr_state = state
        t = curr_state.timestep
        path.append(curr_state.loc)

        while(curr_state.loc != self.start_loc):
            curr_state = curr_state.parent_state

            for i in range(curr_state.timestep, t):
                path.append(curr_state.loc)
            t = curr_state.timestep
        
        return path[::-1]
 





        