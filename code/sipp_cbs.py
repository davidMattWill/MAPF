import time as timer
import heapq
from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost

import sipp_planner_alt as sipp_planner


def detect_collision(path1, path2):
    ##############################
    #           Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           Use "get_location(path, t)" to get the location of a robot at time t.

    for i in range(1, max(len(path1), len(path2))):
        if get_location(path1, i) == get_location(path2, i-1) and get_location(path2, i) == get_location(path1, i-1):
           return {'a1':None, 'a2':None, 'loc':[get_location(path1, i - 1), get_location(path1, i)], 'timestep':i}
        if get_location(path1, i) == get_location(path2, i):
            return {'a1':None, 'a2':None, 'loc':[get_location(path1, i)], 'timestep':i}
    return None


def detect_collisions(paths):
    ##############################
    #           Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.

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
    #           Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                             specified timestep, and the second constraint prevents the second agent to be at the
    #                             specified location at the specified timestep.
    #           Edge collision:   The first constraint prevents the first agent from travelling to the second agents tile,
    #                             And the second constraint prevents the second agent from travelling to the first agents tile
    #                             The edge constraint already presumes that the agents are intersecting along an edge between t-1 and t
    #                         

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
    ##############################
    # High level sipp_CBS Search:
    #           my_map  - List of lists specifying obstacle positions
    #           starts  - List of agent start positions 
    #           Goals   - List of agent goal positions
    #           Functions:
    #               push/pop_node:  Manages a priority queue for CBS nodes, sorted by node['cost'] property
    #               
    #               find_solution:  Generates the CB search tree. Each node contains properties:
    #                                  
    #                                Node = {'cost': c
    #                                       'constraints': [],
    #                                       'paths': [],
    #                                       'collisions': []}
    #                               
    #                                Cost is self explainatory, the value we're trying to minimize with the search tree. It is calculated from the overall sum of path lengths in the nodes path list
    #                                Constraints specifies a list of constrained agent positions, i.e. Positions agents are 'banned' from being at on time t. 
    #                                Paths defines the list of agent paths calculated for the Node using the SIPP A* algorithm, and constrained by the constraint list
    #                                Collisions maintains a list of first time collisions between all agent pairs for the particular path list
    #                               
    #                                Very brief overview is that the function defines a binary search tree explored in a usual best first Search manner. 
    #                                Two successor nodes are generated by picking a collision, calculated from the path configration assigned to the node, extracting a constraint pair,
    #                                and respectively applying the constraints to each successor. This leaves one successor with a constraint on the first agent, and the other with a constraint on the second.
    #                                Furthermore, each successor also keeps all the constraints of its parent node. Each new constraint will be used to recalculate the path of the agent the constraint applies to.
    #               
    #               other functions: Defined above
                  
    def __init__(self, my_map, starts, goals):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

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

    def find_solution(self):
        self.start_time = timer.time()
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        # Find initial path for each agent
        for i in range(self.num_of_agents):  
            path = sipp_planner.get_path_sipp(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], {})
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
                return P['paths'], timer.time() - self.start_time, self.num_of_generated, self.num_of_expanded

            #Picks a collision. There are other ways to pick collisions, but picking the first one generally works. 
            first_collision = P['collisions'][0]
            new_constraints = standard_splitting(first_collision)

            for constraint in new_constraints:
                agent = constraint['agent']
                #Copies Data from the parent node + the new constraint          
                Q = {'cost': 0,
                 'constraints': [constraint],  
                 'paths': [],  
                 'collisions': []}
                for const in P['constraints']:
                    if const not in Q['constraints']:
                        Q['constraints'].append(const)
                for pat in P['paths']:
                    Q['paths'].append(pat)

                #Converts the constraint list into a form the sipp planner can use. Only uses constraints that apply to the agent
                unsafe_list = {}
                for const in Q['constraints']:
                    if(const['agent'] == agent):
                        if const['loc'] not in unsafe_list:
                            unsafe_list[const['loc']] = []
                        unsafe_list[const['loc']].append((const['timestep'], const['timestep']))

                #Recalculates the path of 'agent' with the constraints
                path = sipp_planner.get_path_sipp(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], unsafe_list)   
                if path:
                    Q['paths'][agent] = path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.push_node(Q)

        self.print_results(root)
        return root['paths'], 0, 0, 0


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

'''
unsafe_list = {}
        for const in Q['constraints']:
            if const['agent'] == agent:
                if len(const['loc']) == 1:
                    if const['loc'][0] not in unsafe_list:
                        unsafe_list[const['loc'][0]] = []
                    unsafe_list[const['loc'][0]].append((const['timestep'], const['timestep']))
                else:
                    if const['loc'][0] == Q['paths'][agent][const['timestep'] -1]:
                        if const['loc'][1] not in unsafe_list:
                            unsafe_list[const['loc'][1]] = []
                        unsafe_list[const['loc'][1]].append((const['timestep'], const['timestep']))
                    if const['loc'][1] == Q['paths'][agent][const['timestep'] -1]:
                        if const['loc'][0] not in unsafe_list:
                            unsafe_list[const['loc'][0]] = []
                        unsafe_list[const['loc'][0]].append((const['timestep'], const['timestep']))
'''