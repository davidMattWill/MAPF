To run: python run_experiments.py --instance instances/(instanceName).txt --solver (solverName)

Depdendencies: Python3 with numpy and matplotlib

Solvers include:
  - Independent
  - Prioritized
  - CBS
  - sipp_prioritized
  - sipp_cbs

  - Independent, Prioritized and CBS make use of conventional A*, and sipp_prioritized and sipp_cbs make use of safe interval A*

to create instances:
  - First row of the text file should specify dimensions of the map (4x4 in this case).
  - @ specifies an obstacle and . specifies a free space.
  - Below the map, the next row specifies the number of agents (2 in this case)
  - The final rows specifies the agent's start and goal locations (0,2) to (2,3) for agent 1 and (1,2) to (2,2) for agent 2
  - Save as a .txt
 
  4 4
  @ @ . @
  @ @ . @
  @ . . .
  @ @ @ @
  2
  0 2 2 3
  1 2 2 2


  
  


