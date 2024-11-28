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
class State():
    def __init__(self, loc, cfg):
        self.loc = loc
        self.cfg = cfg

class CFG_MAP():
    def __init__(self):
        self.cfg_map = {}

    


class CFG():
    def __init__(self):
        #NEED TO HANDLE splitting intervals
        self.intervals = [(0, float('inf'))]
    
    #function will receive a time interval and split safe intervals accordingly
    #SEEMS TO WORK
    def split(self, collision_interval):
        #Specifying fail conditions
        print("splitting on", collision_interval, "\n")
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
                i-=1
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

        print("result", self.intervals)       


#Functions for pushing and popping from PQ, ordered by state F() values
def push_queue(list, state):
    pass
def pop_queue(list):
    pass


def move(loc, my_map):
    #Defining movement with respect to map. Going to move some functionality that in the single agent plannet would be in a_star into this function.
    #it will return all the directions we're certain we can move to minus the configurations, which will be handled in getSuccessors as specified in the paper
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    valid_moves = []
    for i in range(len(directions)):
        new_loc = loc[0] + directions[i][0], loc[1] + directions[i][1]
        #Checking out of bounds condition and no static obstacles. Does not consider dynamic obstacles, will be handled elsewhere
        if new_loc[0] < 0 or new_loc[0] >= len(my_map) or new_loc[1] < 0 or new_loc[1] >= len(my_map[0]):
            continue
        if my_map[new_loc[0]][new_loc[1]]:
            continue
        valid_moves.append(new_loc)
    #will return 4 directions we can move in. I believe wait actions are handled implicitly in the way successor nodes are generated
    return valid_moves
  
def get_successors(state, cfg_list):
    #Need to implement algorithm as defined in the paper + add configurations to list as needed
    successors = []
    valid_moves = move(state['loc'])

    for move in valid_moves:
        #check if configuration exists, if not we create a new one for the possible successor location
        if state.loc not in cfg_list:
            cfg_list[state.loc] = CFG()

        m_time = 1
        start_time = state['timestep'] + m_time
        end_t = state.interval[1] + m_time



    pass

def get_heuristic(loc, goal_loc):
    pass

def get_path(state):
    pass

def update_time():
    #unsure about implementation for this function atm.
    pass




def get_path_sipp(my_map, start_loc, goal_loc, agent, cfg_list):
    #initialize the configuration dict. keys will be (x,y) tuple, value is dict with associated data. We'll update the configuration list as we expand new nodes
    edge_cost = 1
    open_list = []
    closed_list = {}
    
    if start_loc not in cfg_list:
        cfg_list[start_loc] = CFG()
    root_state = State(start_loc)


    push_queue(open_list, root_state)
    while len(open_list) > 0:
        curr = pop_queue(open_list)
        if curr['loc'] == goal_loc:
            return get_path(curr)


        successors = get_successors(curr, cfg_list)
        for successor in successors:
            if successor not in open_list: #or closed list?
                successor['g_val'] = float('inf')
                successor['f_val'] = successor['g_val']
            if successor['g_val'] > curr['g_val'] + edge_cost:
                successor['g_val'] = curr['g_val'] + edge_cost
                successor['parent'] = curr
                update_time(successor) #unsure
                successor['f_val'] = successor['g_val'] + successor['h_val']
                push_queue(open_list, successor)

    return None