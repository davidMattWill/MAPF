import sipp_planner_new
import time as timer
from single_agent_planner import get_sum_of_cost



class SIPP_PrioritizedSolver(object):
    def __init__(self, my_map, starts, goals):
   

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        start_time = timer.time()
        result = []

        unsafe_interval_list = {}
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
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        
        
        return result
    
