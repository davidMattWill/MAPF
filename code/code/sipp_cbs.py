import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

import sipp_planner_new



def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    #Dont need to check path[0] for either path, as those are the start positions
    for i in range(1, max(len(path1), len(path2))):
        if get_location(path1, i) == get_location(path2, i-1) and get_location(path2, i) == get_location(path1, i-1):
           return {'a1':None, 'a2':None, 'loc':[get_location(path1, i - 1), get_location(path1, i)], 'timestep':i}
        if get_location(path1, i) == get_location(path2, i):
            return {'a1':None, 'a2':None, 'loc':[get_location(path1, i)], 'timestep':i}
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    num_agents = len(paths)

    for ith_agent in range(num_agents):
        for jth_agent in range(ith_agent + 1, num_agents):
            collision = detect_collision(paths[ith_agent], paths[jth_agent])
            if collision:
                collision['a1'] = ith_agent
                collision['a2'] = jth_agent
                collisions.append(collision)
    return collisions
        
    


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    #We need to say both agents cant be at position (x,y) at t
    if len(collision['loc']) == 1:
        constraint_1 = {'agent':collision['a1'], 'loc':collision['loc'][0], 'timestep':collision['timestep']}
        constraint_2 = {'agent':collision['a2'], 'loc':collision['loc'][0], 'timestep':collision['timestep']}
        return [constraint_1, constraint_2]
    if len(collision['loc']) == 2:
        constraint_1 = {'agent':collision['a1'], 'loc': collision['loc'][1], 'timestep':collision['timestep']}
        constraint_2 = {'agent':collision['a2'], 'loc': collision['loc'][0], 'timestep':collision['timestep']}
        print(constraint_1, constraint_2)
        return [constraint_1, constraint_2]

    return None






class SIPP_CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            planner = sipp_planner_new.SIPP(self.my_map, self.starts[i], self.goals[i], {})
            path = planner.get_path_sipp()
            if path is None:
                raise BaseException('No solutions')
            
        
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

     
        while(len(self.open_list) > 0):
            P = self.pop_node()
            if len(P['collisions']) == 0:
                self.print_results(P)
                return P['paths']

            #Alternatively, could pick the collision at random. Seems to work better in some cases
            #collision_index = random.randrange(0, len(P['collisions']))
            first_collision = P['collisions'][0]
            new_constraints = standard_splitting(first_collision)


            
            for constraint in new_constraints:
                agent = constraint['agent']

                Q = {'cost': 0,
                 'constraints': [constraint],  
                 'paths': [],  
                 'collisions': []}
                #Copy the constraints and paths from P

                unsafe_list = {}
                for const in P['constraints']:
                    if const not in Q['constraints']:
                        Q['constraints'].append(const)

                
                for const in Q['constraints']:
                    if(const['agent'] == agent):
                        if const['loc'] not in unsafe_list:
                            unsafe_list[const['loc']] = []
                        unsafe_list[const['loc']].append((const['timestep'], const['timestep']))

                
                for pat in P['paths']:
                    Q['paths'].append(pat)
                #generate a new path on the agent the constraint applies to



                print(unsafe_list)
                planner = sipp_planner_new.SIPP(self.my_map, self.starts[agent], self.goals[agent],unsafe_list)
                path = planner.get_path_sipp()
              
                if path:
                    #update path in Q
                    Q['paths'][agent] = path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.push_node(Q)

        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
