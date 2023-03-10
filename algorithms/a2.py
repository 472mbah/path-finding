import heapq

def astar_algorithm(grid, start, end, maxIterations=1000):
    heap = []
    heapq.heappush(heap, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while heap and maxIterations >= 1000:
        current = heapq.heappop(heap)[1]
        if current == end:
            break
        for neighbor in get_neighbors(grid, current):
            new_cost = cost_so_far[current] + distance(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + distance(neighbor, end)
                heapq.heappush(heap, (priority, neighbor))
                came_from[neighbor] = current
        maxIterations -= 1
        
    if end not in came_from:
        return ([], came_from)
    
    path = []
    current = end
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    return (path[::-1], came_from)

def get_neighbors(grid, current):
    row, col = current
    neighbors = []
    for r, c in ((row-1, col), (row, col-1), (row+1, col), (row, col+1)):
        if (r, c) in grid:
            continue
        neighbors.append((r, c))
    return neighbors

def distance(a, b):
    x1, y1 = a
    x2, y2 = b
    return abs(x1 - x2) + abs(y1 - y2)
