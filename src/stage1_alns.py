import random
import math
from src.utils import euclidean, roulette_wheel

def tour_length(route, coords):
    return sum(
        euclidean(coords[route[i]], coords[route[i + 1]])
        for i in range(len(route) - 1)
    )

def random_destroy(route, ratio):
    route = route[:]
    n_remove = max(1, int((len(route) - 2) * ratio))
    removed = random.sample(route[1:-1], n_remove)
    for n in removed:
        route.remove(n)
    return route, removed

def greedy_repair(partial, removed, coords):
    route = partial[:]
    for node in removed:
        best_pos, best_inc = None, float("inf")
        for i in range(len(route) - 1):
            inc = (
                euclidean(coords[route[i]], coords[node]) +
                euclidean(coords[node], coords[route[i + 1]]) -
                euclidean(coords[route[i]], coords[route[i + 1]])
            )
            if inc < best_inc:
                best_inc = inc
                best_pos = i + 1
        route.insert(best_pos, node)
    return route

def ALNS_SA_TSP(
    init_route,
    coords,
    T0=200,
    Tend=1,
    alpha=0.995,
    fini=0.75,
    fend=0.25,
    BZ=0.1,
    lambda_w=0.9
):
    S = init_route[:]
    S_star = S[:]
    f_star = tour_length(S, coords)

    T = T0
    k = 1
    destroy_w = [1.0]
    repair_w = [1.0]

    while T > Tend:
        ratio = fini * (fend / fini) ** k

        d_id = roulette_wheel(destroy_w)
        r_id = roulette_wheel(repair_w)

        partial, removed = random_destroy(S, ratio)
        S_new = greedy_repair(partial, removed, coords)

        f_old = tour_length(S, coords)
        f_new = tour_length(S_new, coords)

        if f_new < f_old or random.random() < math.exp(-(f_new - f_old)/(BZ*T)):
            S = S_new
            if f_new < f_star:
                S_star, f_star = S_new, f_new
                destroy_w[d_id] += 1
                repair_w[r_id] += 1

        destroy_w = [lambda_w*w + (1-lambda_w) for w in destroy_w]
        repair_w = [lambda_w*w + (1-lambda_w) for w in repair_w]

        T *= alpha
        k += 1

    return S_star
