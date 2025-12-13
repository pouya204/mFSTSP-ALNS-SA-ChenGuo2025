from src.stage1_alns import ALNS_SA_TSP
from src.stage2_segmentation import segment_route
from src.stage2_allocation_sa import improved_SA_allocation

def solve_mFSTSP(coords, LE):
    nodes = list(coords.keys())
    init_route = [0] + nodes[1:] + [0]

    tsp_route = ALNS_SA_TSP(init_route, coords)
    segments = segment_route(tsp_route, coords, LE)

    total_time = 0.0
    for seg in segments:
        X0 = [0] + [1]*(len(seg)-2) + [0]
        _, cost = improved_SA_allocation(X0, seg, coords)
        total_time += cost

    return total_time
