To run instances: python run_instances.py --instance instances/(instanceName).txt --solver (solverName)

Depdendencies: Python3 with numpy and matplotlib

Solvers include:
  - Independent
  - Prioritized
  - CBS
  - sipp_prioritized
  - sipp_cbs

  - Independent, Prioritized and CBS make use of conventional A*, and sipp_prioritized and sipp_cbs make use of safe interval A*


To run benchmarks: python run_benchmarks.py --instance bench/(benchfolder)/(map-name).map bench/(benchfolder)/(scen).scen --solver (solverName)
  - run_benchmarks does not visualize as larger maps might cause program to freeze with the visualizer.
  - benchmark requires the scen file and map to match, they are together in the same folders. 
  - you can call: python plot_results.py afterword and it should work fine 
  
  -DATA FORMAT FOR CSV FILES per row: (number of agents) (sum of cost) (Execution time) (Node Generations) (Node expansions). run_benchmarks will stop when it goes over time

  