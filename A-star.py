import heapq

# ---------- Heuristic (Manhattan distance) ----------
def heuristic(a, b):
    """Manhattan distance between points a and b."""
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


# ---------- Get neighbors in 4 directions ----------
def get_neighbors(node, grid):
    """Return valid 4-directional neighbors of node on the grid."""
    (x, y) = node
    rows = len(grid)
    cols = len(grid[0])

    # 4 possible moves: up, down, left, right
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        # Check bounds and that it's not a wall (1 means wall)
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
            neighbors.append((nx, ny))

    return neighbors


# ---------- Reconstruct path from came_from dict ----------
def reconstruct_path(came_from, current):
    """Reconstruct path from start to current using came_from map."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------- A* Search ----------
def astar(grid, start, goal):
    """
    A* pathfinding on a grid.
    grid: 2D list, 0 = free, 1 = obstacle
    start, goal: (row, col)
    Returns: (path, total_cost) or (None, inf) if no path.
    """

    # Priority queue of (f_score, node)
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}          # node -> previous node on best path
    g_score = {start: 0}    # cost from start to this node
    f_score = {start: heuristic(start, goal)}  # estimated total cost

    while open_set:
        # Get node with smallest f_score
        current_f, current = heapq.heappop(open_set)

        # If we reached the goal, reconstruct the path
        if current == goal:
            return reconstruct_path(came_from, current), g_score[current]

        # If this f_score is outdated, skip (classic A* optimization)
        if current_f > f_score.get(current, float('inf')):
            continue

        # Explore neighbors
        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + 1  # cost between neighbors = 1

            # If this path is better, record it
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)

                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # No path found
    return None, float('inf')


# ---------- Helper: pretty print grid with path ----------
def print_grid_with_path(grid, path, start, goal):
    """Print the grid and mark the path, start, goal, and walls."""
    path_set = set(path) if path else set()
    for i, row in enumerate(grid):
        line = ""
        for j, cell in enumerate(row):
            if (i, j) == start:
                line += "S "
            elif (i, j) == goal:
                line += "G "
            elif cell == 1:
                line += "# "
            elif (i, j) in path_set:
                line += "* "
            else:
                line += ". "
        print(line)
    print()


# ---------- Example usage ----------
if __name__ == "__main__":
    # 0 = free cell, 1 = obstacle
    grid = [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [1, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 0],
    ]

    start = (0, 0)  # top-left
    goal = (5, 5)   # bottom-right

    path, cost = astar(grid, start, goal)

    if path:
        print(f"Path found with cost {cost}:")
        print(path)
        print()
        print_grid_with_path(grid, path, start, goal)
    else:
        print("No path found.")
