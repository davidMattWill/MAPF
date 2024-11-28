import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.max_timestep = 20

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        #1.2/1.3 Test constraint
        constraint_1_2 = {'agent':0, 'loc':[(1,5)], 'timestep':4}
        constraint_1_3 = {'agent':1, 'loc':[(1,2),(1,3)], 'timestep':1}
        constraint_1_4 = {'agent':0, 'loc':[(1,5)], 'timestep':10}

        #1.5 Collision Free Path
        constraint_1 = {'agent':1, 'loc':[(1,4)], 'timestep':2}
        constraint_2 = {'agent':1, 'loc':[(1,2)], 'timestep':2}
        constraint_3 = {'agent':1, 'loc':[(1,3)], 'timestep':2}
      




        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            if path:
                #for each successive agent that hasnt generated a path yet we add restrictions
                goal_loc = path[-1]
                goal_timestep = len(path) - 1


                for jth_agent in range(i+1, self.num_of_agents):
                    for timestep in range(1, len(path)):
                        vertex_constraint = {'agent':jth_agent, 'loc':[path[timestep]], 'timestep':timestep, 'positive':False}
                        edge_constraint = {'agent':jth_agent, 'loc':[path[timestep], path[timestep - 1]], 'timestep':timestep, 'positive':False}
                        constraints.append(vertex_constraint)
                        constraints.append(edge_constraint)

                    #NEED TO THINK OF SOMETHING BETTER
                    for timestep in range(goal_timestep, self.max_timestep):
                        future_constraint = {'agent':jth_agent, 'loc':[goal_loc], 'timestep':timestep, 'positive':False}
                        constraints.append(future_constraint)
                



            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
