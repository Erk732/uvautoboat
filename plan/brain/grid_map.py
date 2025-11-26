import math

class GridMap:
    def __init__(self, width_m=300, height_m=300, resolution=1.0):
        self.resolution = resolution
        self.width_m = width_m
        self.height_m = height_m
        self.cols = int(width_m / resolution)
        self.rows = int(height_m / resolution)

        # Offset puts (0,0) in the middle of the grid
        self.offset_x = width_m / 2.0
        self.offset_y = height_m / 2.0

        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def reset(self):
        # Clears the map to remove "ghost" obstacles
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def world_to_grid(self, wx, wy):
        gx = int((wx + self.offset_x) / self.resolution)
        gy = int((wy + self.offset_y) / self.resolution)
        if 0 <= gx < self.cols and 0 <= gy < self.rows:
            return (gx, gy)
        return None 

    def grid_to_world(self, gx, gy):
        wx = (gx * self.resolution) - self.offset_x
        wy = (gy * self.resolution) - self.offset_y
        return (wx, wy)

    def set_obstacle(self, wx, wy, radius):
        center = self.world_to_grid(wx, wy)
        if not center: return
        cx, cy = center
        r_cells = int(math.ceil(radius / self.resolution))

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.cols and 0 <= ny < self.rows:
                    self.grid[ny][nx] = 1 

    def is_blocked(self, gx, gy):
        if 0 <= gx < self.cols and 0 <= gy < self.rows:
            return self.grid[gy][gx] == 1
        return True