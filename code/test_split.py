from sipp_planner_alt import CFG_MAP

cfg_map = CFG_MAP()

z= cfg_map.get_cfg((1,1))
cfg_map.split(z, (3,3))
cfg_map.split(z, (3,3))
cfg_map.split(z, (3,3))
cfg_map.split(z, (4,4))
cfg_map.split(z, (3,5))
cfg_map.split(z, (7,10))
cfg_map.split(z, (2,12))


print(z['intervals'])
