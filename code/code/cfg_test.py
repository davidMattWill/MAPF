from sipp_planner import CFG, CFG_MAP

unsafe = {(1,2): [(0, 3), (5,5), (6,6), (7,8), (10,10), (15,20)]}

cfg = CFG_MAP(unsafe)
z = cfg.get_cfg((1,2))
print(z.intervals)
q = cfg.get_cfg((1,3))
print(q.intervals)

q = cfg.get_cfg((1,2))
print(q.intervals)