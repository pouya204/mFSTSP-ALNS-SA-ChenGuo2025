import random
import math

# =========================================================
# Basic utilities
# =========================================================

def euclidean(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def roulette_wheel(weights):
    s = sum(weights)
    r = random.uniform(0, s)
    c = 0.0
    for i, w in enumerate(weights):
        c += w
        if c >= r:
            return i
    return len(weights) - 1


# =========================================================
# Algorithm 1: ALNS-SA for TSP
# =========================================================

def tsp_length(route, coords):
    return sum(
        euclidean(coords[route[i]], coords[route[i + 1]])
        for i in range(len(route) - 1)
    )


def random_destroy(route, ratio):
    r = route[:]
    k = max(1, int((len(r) - 2) * ratio))
    removed = random.sample(r[1:-1], k)
    for n in removed:
        r.remove(n)
    return r, removed


def greedy_repair(partial, removed, coords):
    r = partial[:]
    for node in removed:
        best_pos = None
        best_inc = float("inf")
        for i in range(len(r) - 1):
            inc = (
                euclidean(coords[r[i]], coords[node]) +
                euclidean(coords[node], coords[r[i + 1]]) -
                euclidean(coords[r[i]], coords[r[i + 1]])
            )
            if inc < best_inc:
                best_inc = inc
                best_pos = i + 1
        r.insert(best_pos, node)
    return r


def ALNS_SA_TSP(
    S0,
    coords,
    L=50,
    alpha=0.995,
    T0=200,
    K=500,
    BZ=0.1,
    lambda_w=0.9
):
    S = S0[:]
    S_star = S[:]
    T = T0
    k = 1
    l = 1

    destroy_w = [1.0]
    repair_w = [1.0]

    while k <= K:
        d = roulette_wheel(destroy_w)
        r = roulette_wheel(repair_w)

        partial, removed = random_destroy(S, ratio=0.3)
        S_new = greedy_repair(partial, removed, coords)

        f_old = tsp_length(S, coords)
        f_new = tsp_length(S_new, coords)

        if f_new < f_old:
            S = S_new
            if f_new < tsp_length(S_star, coords):
                S_star = S_new[:]
        else:
            if random.random() < math.exp(-(f_new - f_old) / (BZ * T)):
                S = S_new

        if l == L:
            destroy_w = [lambda_w * w + (1 - lambda_w) for w in destroy_w]
            repair_w = [lambda_w * w + (1 - lambda_w) for w in repair_w]
            l = 0

        T *= alpha
        k += 1
        l += 1

    return S_star


# =========================================================
# Algorithm 2: Route Segmentation
# =========================================================

def route_segmentation(route, coords, LE):
    segments = []
    start = 0

    while True:
        dist = 0.0
        end = start

        while end < len(route) - 1:
            d = euclidean(coords[route[end]], coords[route[end + 1]])
            if dist + d > LE:
                break
            dist += d
            end += 1

        if end == start:
            end += 1

        segments.append(route[start:end + 1])
        start = end

        if route[end] == route[0]:
            break

    return segments


# =========================================================
# Algorithm 3: Improved SA for Node Allocation
# =========================================================

def combined_transport_time(X, segment, coords, v_truck=1.0, v_drone=1.5):
    t_truck = 0.0
    t_drone = 0.0

    for i in range(len(segment) - 1):
        d = euclidean(coords[segment[i]], coords[segment[i + 1]])
        if X[i] == 1:
            t_truck += d / v_truck
        elif X[i] == 2:
            t_drone += d / v_drone

    return max(t_truck, t_drone)


def disturb_solution(X):
    X2 = X[:]
    i, j = sorted(random.sample(range(1, len(X2) - 1), 2))
    X2[i:j] = reversed(X2[i:j])
    return X2


def improved_SA_allocation(
    X0,
    segment,
    coords,
    L2=30,
    beta=0.95,
    T0=100,
    BZ=0.1
):
    X = X0[:]
    X_star = X0[:]
    f_star = combined_transport_time(X, segment, coords)

    T = T0
    tau = 0
    N = X0.count(1)

    while tau < N:
        l = 1
        while l <= L2:
            Xp = X[:]
            truck_nodes = [i for i in range(1, len(Xp) - 1) if Xp[i] == 1]
            if not truck_nodes:
                break

            idx = random.choice(truck_nodes)
            Xp[idx] = 2

            Xpp = disturb_solution(Xp)

            f_p = combined_transport_time(Xp, segment, coords)
            f_pp = combined_transport_time(Xpp, segment, coords)

            if f_pp < f_p or random.random() < math.exp(-(f_pp - f_p) / (BZ * T)):
                X = Xpp
                if f_pp < f_star:
                    X_star = Xpp[:]
                    f_star = f_pp

            l += 1

        T *= beta
        tau += 1

    return X_star, f_star
