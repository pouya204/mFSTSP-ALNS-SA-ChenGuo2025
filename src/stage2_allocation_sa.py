import random
import math
from src.objective import combined_time

def disturb(X):
    X2 = X[:]
    i, j = sorted(random.sample(range(1, len(X) - 1), 2))
    X2[i:j] = reversed(X2[i:j])
    return X2

def improved_SA_allocation(X0, segment, coords, L2=30, beta=0.95, T0=100, BZ=0.1):
    N = X0.count(1)
    X_star = X0[:]
    f_star = combined_time(X_star, segment, coords)

    T = T0
    tau = 0

    while tau < N:
        l = 1
        while l < L2:
            Xp = X0[:]
            idx = random.choice([i for i in range(1, len(Xp)-1) if Xp[i] == 1])
            Xp[idx] = 2

            Xpp = disturb(Xp)

            f_p = combined_time(Xp, segment, coords)
            f_pp = combined_time(Xpp, segment, coords)

            if f_pp < f_p or random.random() < math.exp(-(f_pp - f_p)/(BZ*T)):
                if f_pp < f_star:
                    X_star, f_star = Xpp, f_pp

            l += 1

        T *= beta
        tau += 1

    return X_star, f_star
