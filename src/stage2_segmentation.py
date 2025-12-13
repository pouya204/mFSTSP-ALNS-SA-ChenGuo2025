from src.utils import euclidean

def segment_route(route, coords, LE):
    segments = []
    start = 0

    while start < len(route) - 1:
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
    return segments
