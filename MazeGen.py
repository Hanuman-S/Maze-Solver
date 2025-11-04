import random

def generate_maze(width, height, seed=None):
    if seed is not None:
        random.seed(seed)
    if width % 2 == 0: width += 1
    if height % 2 == 0: height += 1

    grid = [[1]*width for _ in range(height)]

    def in_bounds(r, c): return 0 <= r < height and 0 <= c < width
    dirs = [(-2,0),(2,0),(0,-2),(0,2)]

    # Start carving from a random odd cell
    start_r = random.randrange(1, height, 2)
    start_c = random.randrange(1, width, 2)
    grid[start_r][start_c] = 0
    stack = [(start_r, start_c)]

    while stack:
        r, c = stack[-1]
        neighbors = []
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if in_bounds(nr, nc) and grid[nr][nc] == 1:
                neighbors.append((nr, nc, dr, dc))
        if neighbors:
            nr, nc, dr, dc = random.choice(neighbors)
            grid[r + dr//2][c + dc//2] = 0
            grid[nr][nc] = 0
            stack.append((nr, nc))
        else:
            stack.pop()

    # Mark start and end
    start, end = (1,1), (height-2, width-2)
    grid[start[0]][start[1]] = 2
    grid[end[0]][end[1]] = 3

    return grid

def print_maze_formatted(grid):
    for row in grid:
        print("{" + ",".join(map(str, row)) + "},")

# Example usage
maze = generate_maze(20, 20, seed=21)

print("Maze (1=wall, 0=path, 2=start, 3=end):\n")
print_maze_formatted(maze)
