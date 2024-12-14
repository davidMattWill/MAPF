import heapq

def move(loc, dir):
    #Added fifth direction (0,0) for no movement
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

def build_constraint_table(constraints, agent):
    #index the negative constraints that apply to the agent according to timestep
    constraint_table = {}
    if constraints:
        for constraint in constraints:
            timestep = constraint['timestep']
            if constraint['agent'] == agent:
                if timestep not in constraint_table:
                    constraint_table[timestep] = [constraint]
                else:
                    constraint_table[timestep].append(constraint)
            #If we encounter a positive constraint on another agent we build a new negative constraint for this one
            if constraint['agent'] != agent and constraint['positive'] is True:
                if len(constraint['loc']) == 1:
                    rev_constraint = {'agent':agent, 'loc':constraint['loc'], 'timestep':constraint['timestep'], 'positive':False}
                else:
                    rev_constraint = {'agent':agent, 'loc': [constraint['loc'][1], constraint['loc'][0]], 'timestep':constraint['timestep'], 'positive':False}
                if timestep not in constraint_table:
                    constraint_table[timestep] = [rev_constraint]
                else:
                    constraint_table[timestep].append(rev_constraint)
    return constraint_table




def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    #Two possible constraints. Edge and Vertex. Iterate through timesteps and check each index to see if its a single loc or loc pair. If its a pair, its an edge constraint, if a single, a vert
    #Check if the constraints apply to the locations passed and time passed
    if constraint_table:
        if next_time in constraint_table:
            for constraint in constraint_table[next_time]:
                if len(constraint['loc']) == 1 and next_loc == constraint['loc'][0]:
                    if constraint['positive'] is True:
                        return 'P'
                    else:
                        return 'N'
                if len(constraint['loc']) == 2 and curr_loc == constraint['loc'][0]and next_loc == constraint['loc'][1]:
                    if constraint['positive'] is True:
                        return 'P'
                    else:
                        return 'N'
    return 'DNE'

#function checks if a given position is constrained at a point in future time for a given agent
#Will be used to modify the goal test condition 
#only considers negative vertex constraints as it is only to be used for an agent already on its goal position
def is_future_constrained(curr_loc, curr_time, constraint_table):
    #Maybe smarter is way to check if loc exists and see if its key is > curr time?
    future_timesteps = [key for key in constraint_table.keys() if key > curr_time]
    for key in future_timesteps:
        for constraint in constraint_table[key]:

            if len(constraint['loc']) == 1 and curr_loc == constraint['loc'][0]:
                return True
    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    #defining max timestep here, but could be passed as an arg to a_star from CBS and prioritized
    max_timestep = 1000

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    #print(constraint_table)
    constraint_table = build_constraint_table(constraints, agent)

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': earliest_goal_timestep}
    push_node(open_list, root)
    closed_list[(root['loc'])] = root

    
    while len(open_list) > 0:
        #maybe
        curr = pop_node(open_list)
        if curr['timestep'] > max_timestep:
            return None
 
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and not is_future_constrained(curr['loc'], curr['timestep'], constraint_table):
            return get_path(curr)
      

        #positive constraint handling
        #Check if there is a positive constraint at time t, and if exists and is enforceable we generate a single child node, then push the node.
        #If its not enforceable then we generate no children
        found_positive_constraint = False
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > len(my_map) - 1 or child_loc[1] > len(my_map[0]) - 1:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table) == 'P':
                #Generate just the one child and skip   
                child = {'loc': child_loc,
                        'g_val': curr['g_val'] + 1,
                        'h_val': h_values[child_loc],
                        'parent': curr,
                        'timestep': curr['timestep'] + 1}
                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

                found_positive_constraint = True
                break

        #If we found a positive constraint for the agent at time t, theres no need to generate other children
        if found_positive_constraint is True:
            continue
        #If we never found a positive constraint, we then consider all the possible negative constraints out of the potential movement directions
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            #conditions for dropping a potential child node
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > len(my_map) - 1 or child_loc[1] > len(my_map[0]) - 1:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table) == 'N':
                continue
            #If conditions not met, generate child node
           
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
        
    print("NOPATH")
    return None  # Failed to find solutions
