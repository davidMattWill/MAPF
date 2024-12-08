import sipp_planner_new
import time as timer



class SIPP_PrioritizedSolver(object):
    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        #GOING TO USE A DIFFERENNT HEURISTIC
        #self.heuristics = []
        #for goal in self.goals:
            #self.heuristics.append(compute_heuristics(my_map, goal))

   
   #The Way prioritized planning will work right now, is we'll keep a dict of collision intervals indexed by location. After each iteration we add the path of the previous agent
   #to the dict and call an update fucnction in the CFG map which will split the intervals inside each planner. This is VERY cumbersome so i'll have to think of a more efficient way for the agents to share a planner instead
    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""




        start_time = timer.time()
        result = []

  
        unsafe_interval_list = {}

        #Right now there is no prioritized planning, each agent will find a path independently. Going to keep this way for testing
        for i in range(self.num_of_agents):
            planner = sipp_planner_new.SIPP(self.my_map, self.starts[i], self.goals[i], unsafe_interval_list)
            path = planner.get_path_sipp()

            
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            
            for i, loc in enumerate(path):
                if loc not in unsafe_interval_list:
                    unsafe_interval_list[loc] = []  
                
                if i == len(path) - 1:
                    unsafe_interval_list[loc].append((i, float('inf')))
                else:
                    unsafe_interval_list[loc].append((i,i))
        



        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        #print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        
        
        return result
    
