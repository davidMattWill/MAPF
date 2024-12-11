import math
import heapq



class State():
    def __init__(self, loc, timestep, interval):
        self.loc = loc
        self.timestep = timestep
        self.interval = interval


class CFG():
    def __init__(self):
        self.intervals = [(0, float('inf'))]
        self.g = float('inf')
        self.f = float('inf')
        self.parent_state = None

    def split(self, collision_interval):
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
    
    
def push_pq(open_list, state_dat):
    heapq.heappush(open_list, state_dat)

def pop_pq(open_list):
    _, _, _, curr_state = heapq.heappop(open_list)
    return curr_state


class SIPP():
    def __init__(self, my_map, start_loc, goal_loc, collision_list):
        self.my_map = my_map
        self.start_loc = start_loc
        self.goal_loc = goal_loc

        self.cfg_map = CFG_MAP(collision_list)
    
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
        
        #for the root state we pick the first interval for the configuration at the nodes position
        root_cfg = self.cfg_map.get_cfg(self.start_loc)
        root_state = State(self.start_loc, 
                           0, 
                           root_cfg.intervals[0])
        root_cfg.g = 0
        root_cfg.f = self.get_heuristic(self.start_loc)
        push_pq(open_list, (root_cfg.f, self.get_heuristic(root_state.loc), root_state.loc, root_state))
  

        while len(open_list) > 0:
            curr_state = pop_pq(open_list)
            curr_cfg = self.cfg_map.get_cfg(curr_state.loc)
            
            successors = self.get_successors(curr_state)
            for succ_state in successors:
                succ_cfg = self.cfg_map.get_cfg(succ_state.loc)
                if succ_cfg.g > curr_cfg.g + edge_cost:
                    succ_cfg.g = curr_cfg.g + edge_cost
                    succ_cfg.parent_state = curr_state
                    
                    if succ_state.loc == self.goal_loc: #and succ_state.interval[1] == float('inf'):
                        print("found Path!")
                        return self.reconstruct_path(succ_state)
                       
                    succ_cfg.f = succ_cfg.g + self.get_heuristic(succ_state.loc)
                    push_pq(open_list, (succ_cfg.f, self.get_heuristic(succ_state.loc), succ_state.loc, succ_state))
        return None
                    


           
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
                succ = State(mov, t, interval)
                successors.append(succ)

        return successors

    
    def reconstruct_path(self, state):
        path = []

        curr_state = state
        curr_cfg = self.cfg_map.get_cfg(curr_state.loc)
        t = curr_state.timestep
        path.append(curr_state.loc)

        while(curr_state.loc != self.start_loc):
            curr_state = curr_cfg.parent_state
            curr_cfg = self.cfg_map.get_cfg(curr_state.loc)

            #This is the time period which the agent is actually at a given loc
            for i in range(curr_state.timestep, t):
                path.append(curr_state.loc)
            t = curr_state.timestep
        

        
        return path[::-1]
 





        