#General things i'll need to accomplish
#1.) Each grid index(configuration) will need to have safe intervals assigned to it, which are time periods where a robot is allowed to move to it.
# We'll need some functionality for taking a user map and extracting pre defined paths for dynamic obstacles. In a MAPF scenario, what we'll do is compute each agents
# path and then update the safe intervals with the agents path in mind for the next one (prioritized). I'm not sure how CBS will work.
#Each grid index should probably contain a dict which has the relevent values. Two ways we can probably do this, either initialize the entire map with dicts for each index, or do it on the fly
# and keep a list to track which ones have and havent been generated. Probably the second is better because we dont need configurations for indices that will probably never be visited

#2.)we'll need to implement the stated pathfinding algorithm

#Function for initializing a configuration, we check if a dynamic obstacle list is passed

#Going to write a class for managing the configuration list and related functions for splitting configurations. This way I can share a single configuration list among a_star_sipp calls

#UNSURE IF THIS IS BEST WAY
from math import fabs
import math
import heapq


#READDD:: I SUSPECT THE ISSUE IS EITHER a.)Something wrong with a* or b.)something wrong with interval splitting. Probably former 

class State():
    def __init__(self, loc = (-1,-1), timestep = 0, interval = (0, float('inf'))):
        self.loc = loc
        self.timestep = timestep
        self.interval = interval

        self.g = float('inf')
        self.f = float('inf')
        self.parent_state = None

    
class State_Map():
    def __init__(self):
        self.state_dict = {}

    def get_state(self, loc, timestep, interval):
        if (loc, interval) not in self.state_dict:
            self.state_dict[(loc, interval)] = State(loc, timestep, interval)
        return self.state_dict[(loc, interval)]

    



class CFG():
    def __init__(self):
        #NEED TO HANDLE splitting intervals
        self.intervals = [(0, float('inf'))]
 
       
    
    #function will receive a time interval and split safe intervals accordingly
    #SEEMS TO WORK
    def split(self, collision_interval):
        #Specifying fail conditions
        #print("splitting on", collision_interval, "\n")
        print(collision_interval)
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

        #print("result", self.intervals)       

class CFG_MAP():
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
    
    


#Helper functions for pushing and popping queue NEED TO CHANGE!!
def push_pq(open_list, state_dat):
    heapq.heappush(open_list, state_dat)


def pop_pq(open_list):
    _, _, _, curr_state = heapq.heappop(open_list)
    return curr_state

def print_state(state):
    print("loc: "+ state.loc, ", time: " + state.timestep + ", interval: " + state.interval)
def print_cfg(cfg):
    print("F: " + cfg.f + ", g: " + cfg.g + ", interval_list: " + cfg.interval_list + ", parent_loc: " + cfg.parent_state.loc)

class SIPP():
    def __init__(self, my_map, start_loc, goal_loc, collision_list):
        self.my_map = my_map
        self.start_loc = start_loc
        self.goal_loc = goal_loc

        self.cfg_map = CFG_MAP(collision_list)
        self.state_map = State_Map()
    
    #Going to use the line distance as my heuristic
    def get_heuristic(self, loc):
        return math.sqrt(pow(self.goal_loc[0] - loc[0], 2) + pow(self.goal_loc[1] - loc[1], 2))
    
    #def get_heuristic(self, position):
        #return fabs(position[0] - self.goal_loc[0]) + fabs(position[1]-self.goal_loc[1])
      
    def move(self, loc):
    #Defining movement with respect to map. Going to move some functionality that in the single agent plannet would be in a_star into this function.
    #it will return all the directions we're certain we can move to minus the configurations, which will be handled in getSuccessors as specified in the paper
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        valid_moves = []
        for i in range(len(directions)):
            new_loc = (loc[0] + directions[i][0], loc[1] + directions[i][1])
            #Checking out of bounds condition and no static obstacles. Does not consider dynamic obstacles, will be handled elsewhere
            if new_loc[0] < 0 or new_loc[0] >= len(self.my_map) or new_loc[1] < 0 or new_loc[1] >= len(self.my_map[0]):
                continue
            if self.my_map[new_loc[0]][new_loc[1]]:
                continue
            valid_moves.append(new_loc)
        #will return 4 directions we can move in. I believe wait actions are handled implicitly in the way successor nodes are generated
        return valid_moves
  

    
    def get_path_sipp(self):
        #initialize the configuration dict. keys will be (x,y) tuple, value is dict with associated data. We'll update the configuration list as we expand new nodes
        edge_cost = 1
        open_list = []
        #visited_set = set()
        
        #for the root state we pick the first interval for the configuration at the nodes position
        root_cfg = self.cfg_map.get_cfg(self.start_loc)
        root_state = self.state_map.get_state(self.start_loc, 0, root_cfg.intervals[0])
        root_state.g = 0 
        root_state.f = root_state.g + self.get_heuristic(root_state.loc)

        push_pq(open_list, (root_state.f, self.get_heuristic(root_state.loc), root_state.loc, root_state))
  

        while len(open_list) > 0:
            curr_state = pop_pq(open_list)
            if curr_state.loc == self.goal_loc and curr_state.interval[1] == float('inf'):
                print("Found path!")
                return self.reconstruct_path(curr_state)
            
            successors = self.get_successors(curr_state)
            for succ_state in successors:
                if succ_state.g > curr_state.g + self.get_cost(curr_state, succ_state):
                    succ_state.g = curr_state.g + self.get_cost(curr_state, succ_state)
                    succ_state.parent_state = curr_state
                    if self.updateTime(curr_state, succ_state):
                        succ_state.f = succ_state.g + self.get_heuristic(succ_state.loc)
                        push_pq(open_list, (succ_state.f, self.get_heuristic(succ_state.loc), succ_state.loc, succ_state))
        return None
    
    def get_cost(self, state, succ_state):
        return abs(succ_state.timestep - state.timestep)
                    
    def updateTime(self, state, succ_state):
        start_t = state.timestep + 1
        end_t = state.interval[1] + 1

        if succ_state.interval[0] > end_t or succ_state.interval[1] < start_t:
            return False
        t = max(start_t, succ_state.interval[0])
        succ_state.timestep = t
        return True

           
    def get_successors(self, state):
    #Need to implement algorithm as defined in the paper + add configurations to list as needed
        successors = []
        valid_moves = self.move(state.loc)
        m_time = 1 #Time to execute a step. On grid, t = 1
        
        for mov in valid_moves:
            cfg = self.cfg_map.get_cfg(mov)


            start_t = state.timestep + m_time
            end_t = state.interval[1]  + m_time # The end time of the states interval
        
            for interval in cfg.intervals:
                if interval[0] > end_t or interval[1] < start_t:
                    continue
                #Unsure about this, check file
                t = max(start_t, interval[0])
                succ = self.state_map.get_state(mov, t, interval)
                successors.append(succ)

        return successors

    
    def reconstruct_path(self, state):
        path = []

        curr_state = state
        t = curr_state.timestep
        path.append(curr_state.loc)

        while(curr_state.loc != self.start_loc):
            curr_state = curr_state.parent_state

            #This is the time period which the agent is actually at a given loc
            for i in range(curr_state.timestep, t):
                path.append(curr_state.loc)
            t = curr_state.timestep
        

        
        return path[::-1]
 





        