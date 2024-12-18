#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

from sipp_prioritized import SIPP_PrioritizedSolver
from sipp_cbs import SIPP_CBSSolver

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)

#Parses the .map file
def parse_map_file(filename):
    my_map = []

    with open(filename, 'r') as file:
        lines = file.readlines()
    
    height = int(lines[1].split()[1])
    width = int(lines[1].split()[1])

    
    my_map = [[False for _ in range(width)] for _ in range(height)]
    for i in range(4, 4+height):
        for j in range(width):
            if lines[i][j] == '@':
                my_map[i-4][j] = True
    return my_map
#parses the .scen file
def parse_scenario_file(file_path):
    starts = []
    goals = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

    for index, line in enumerate(lines):
        parts = line.split()
        if(index == 0):
            continue
        start_x, start_y = int(parts[5]), int(parts[4])
        goal_x, goal_y = int(parts[7]), int(parts[6])
        starts.append((start_x, start_y))
        goals.append((goal_x, goal_y))

    return starts, goals



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')

    parser.add_argument('--instance', type=str, nargs=2, required=True,
                        help='The names of the map and scenario files')                     
    
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()
    map_file, scenario_file = args.instance
    
    
    print("***Import an instance***")
    #specifies the number of agents starting we're going to account for in the list
    my_map = parse_map_file(map_file)
    starts, goals = parse_scenario_file(scenario_file)
    print(starts)
    print(goals)

    #IF CBS OR sipp_cbs is called we run a loop for incrementing number of agents until we retrieve a solution that has crossed the time threshold. Data is written to file

    if args.solver == "CBS":
        print("***Run CBS***")
        result_file = open("cbs_results.csv", "w", buffering=1)
        for i in range(len(starts)):
            cbs = CBSSolver(my_map, starts[0:i], goals[0:i])
            paths, time, num_expanded, num_generated = cbs.find_solution(args.disjoint)
            cost = get_sum_of_cost(paths)
            if time == -1:
                break
            
            result_file.write("{} {} {} {} {}\n".format(i+1, cost, time, num_expanded, num_generated))
            
        result_file.close()
    elif args.solver == "Independent":
        print("***Run Independent***")
        solver = IndependentSolver(my_map, starts, goals)
        paths = solver.find_solution()
    elif args.solver == "Prioritized":
        print("***Run Prioritized***")
        solver = PrioritizedPlanningSolver(my_map, starts, goals)
        paths = solver.find_solution() 
    elif args.solver == "sipp_prioritized":
        print("***Run SIPP Prioritized***")
        solver = SIPP_PrioritizedSolver(my_map, starts, goals)
        paths = solver.find_solution()
    elif args.solver == "sipp_cbs":
        print("***Ruin SIPP CBS***")
        result_file = open("sipp_results.csv", "w", buffering=1)
        for i in range(len(starts)):

            solver = SIPP_CBSSolver(my_map, starts[0:i], goals[0:i])
            paths, time, num_expanded, num_generated = solver.find_solution()
            cost = get_sum_of_cost(paths)
            if time  == -1:
                break
            result_file.write("{} {} {} {} {}\n".format(i+1, cost, time, num_expanded, num_generated))

        result_file.close()
            
    else:
        raise RuntimeError("Unknown solver!")
    
    #print(paths)

    if not args.batch and not args.solver == "sipp_cbs" and not args.solver == "CBS":
        print("***Test paths on a simulation***")
        animation = Animation(my_map, starts, goals, paths)
        animation.show()
    result_file.close()
    

    
    
    