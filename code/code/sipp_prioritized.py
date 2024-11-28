import sipp_planner
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

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""




        start_time = timer.time()
        result = []

        #configuration list is initially empty. The way prioritized planning will work is after each iteration we'll take the agents path and add configuration data for it
        cfg_list = {}

        for i in range(self.num_of_agents):  # Find path for each agent
            #generate path
            path = sipp_planner.a_star_sipp(self.my_map, self.starts[i], self.goals[i], i)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            #NEED TO APPEND PATH AS COLLISIONS TO CONFIGURATION LIST FOR THE NEXT AGENT


        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        #print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        
        
        return None

