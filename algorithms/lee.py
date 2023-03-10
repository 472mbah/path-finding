from collections import deque

def lee_algorithm(grid, start, end, maxIterations=1000):
    queue = deque()
    queue.append(start)
    visited = set()
    visited.add(start)
    distance = {start: 0}
    
    while queue and maxIterations>=0:
        current = queue.popleft()
        if current == end:
            break
        for neighbor in get_neighbors(grid, current):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
                distance[neighbor] = distance[current] + 1
        maxIterations -= 1
                
    if end not in distance:
        return ([], visited)
    
    path = [end]
    while path[-1] != start:
        current = path[-1]
        neighbors = get_neighbors(grid, current)
        neighbor_distances = [distance.get(n, float('inf')) for n in neighbors]
        min_distance = min(neighbor_distances)
        if min_distance == float('inf'):
            return ([], visited)
        min_neighbors = [n for i, n in enumerate(neighbors) if neighbor_distances[i] == min_distance]
        path.append(min_neighbors[0])
        
    return (path[::-1], visited)
        
def get_neighbors(grid, current):
    row, col = current
    neighbors = []
    for r, c in ((row-1, col), (row, col-1), (row+1, col), (row, col+1)):
        if (r, c) in grid:
            continue
        neighbors.append((r, c))
    return neighbors
