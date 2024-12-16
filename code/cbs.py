import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost




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

   
    if len(collision['loc']) == 1:
        constraint_1 = {'agent':collision['a1'], 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':False}
        constraint_2 = {'agent':collision['a2'], 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':False}
        return [constraint_1, constraint_2]
    if len(collision['loc']) == 2:
        constraint_1 = {'agent':collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]], 'timestep':collision['timestep'], 'positive':False}
        constraint_2 = {'agent':collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep':collision['timestep'], 'positive':False}
        print(constraint_1, constraint_2)
        return [constraint_1, constraint_2]

    return None


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    agent = random.choice([collision['a1'], collision['a2']])
   # other_agent = collision['a2'] if agent == collision['a1'] else collision['a1']

    if len(collision['loc']) == 1:
        constraint_1 = {'agent':agent, 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':True}
        constraint_2 = {'agent':agent, 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':False}
        return [constraint_1, constraint_2]
    if len(collision['loc']) == 2:
        if agent == collision['a1']:
            constraint_1 = {'agent':agent, 'loc': [collision['loc'][0], collision['loc'][1]], 'timestep':collision['timestep'], 'positive':True}
            constraint_2 = {'agent':agent, 'loc': [collision['loc'][0], collision['loc'][1]], 'timestep':collision['timestep'], 'positive':False}
            return [constraint_1, constraint_2]
        if agent == collision['a2']:
            constraint_1 = {'agent':agent, 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep':collision['timestep'], 'positive':True}
            constraint_2 = {'agent':agent, 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep':collision['timestep'], 'positive':False}
            return [constraint_1, constraint_2]
    return None


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst




class CBSSolver(object):
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
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while(len(self.open_list) > 0):
            time = timer.time() - self.start_time
            if time > 100:
                print("went over max-time")
                break

          

            P = self.pop_node()
            if len(P['collisions']) == 0:
                self.print_results(P)
                return P['paths'], timer.time() - self.start_time, self.num_of_generated, self.num_of_expanded

            #Alternatively, could pick the collision at random. Seems to work better in some cases
            #collision_index = random.randrange(0, len(P['collisions']))
            first_collision = P['collisions'][0]
            if(disjoint):
                #first_collision = P['collisions'][collision_index]
                new_constraints = disjoint_splitting(first_collision)
            else:
                #first_collision = P['collisions'][0]
                new_constraints = standard_splitting(first_collision)
            for constraint in new_constraints:
                Q = {'cost': 0,
                 'constraints': [constraint],  
                 'paths': [],  
                 'collisions': []}
                #Copy the constraints and paths from P
                for const in P['constraints']:
                    if const not in Q['constraints']:
                        Q['constraints'].append(const)
                for pat in P['paths']:
                    Q['paths'].append(pat)
                #generate a new path on the agent the constraint applies to
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                          agent, Q['constraints'])
                if path:
                    #update path in Q
                    Q['paths'][agent] = path
                    insert_node = True
                    #if disjoint we check for agents that violate the positive constraint and recalcuate their paths
                    if disjoint and constraint['positive'] is True:
                        violating_agents = paths_violate_constraint(constraint, Q['paths'])
                        for viol_agent in violating_agents:
                            viol_agent_newpath = a_star(self.my_map, self.starts[viol_agent], self.goals[viol_agent], self.heuristics[viol_agent],
                                      viol_agent, Q['constraints'])
                            if viol_agent_newpath is None:
                                insert_node = False
                                break
                            Q['paths'][viol_agent] = viol_agent_newpath
                        if insert_node is False:
                            continue
                    
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.push_node(Q)

        self.print_results(root)
        return root['paths'], -1,-1,-1


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
