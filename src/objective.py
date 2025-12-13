from src.utils import euclidean

def combined_time(X, segment, coords, v_truck=1.0, v_drone=1.5):
    t_truck = 0.0
    t_drone = 0.0

    for i in range(len(segment) - 1):
        d = euclidean(coords[segment[i]], coords[segment[i + 1]])
        if X[i] == 1:
            t_truck += d / v_truck
        elif X[i] == 2:
            t_drone += d / v_drone

    return max(t_truck, t_drone)
