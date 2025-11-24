import math

class GridMap:
    def __init__(self, width_m=200, height_m=200, resolution=1.0):
        self.resolution = resolution
        self.cols = int(width_m / resolution)
        self.rows = int(height_m / resolution)
        self.origin_x_idx = self.cols // 2
        self.origin_y_idx = self.rows // 2
        self.data = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def world_to_grid(self, x, y):
        c = int(x / self.resolution) + self.origin_x_idx
        r = int(y / self.resolution) + self.origin_y_idx
        if 0 <= c < self.cols and 0 <= r < self.rows:
            return (r, c)
        return None

    def grid_to_world(self, r, c):
        x = (c - self.origin_x_idx) * self.resolution
        y = (r - self.origin_y_idx) * self.resolution
        return (x, y)

    def set_obstacle(self, x, y, radius=3.0):
        center = self.world_to_grid(x, y)
        if center is None: return
        r_center, c_center = center
        inflation = int(radius / self.resolution)

        for r in range(r_center - inflation, r_center + inflation + 1):
            for c in range(c_center - inflation, c_center + inflation + 1):
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    self.data[r][c] = 1 

    def is_blocked(self, r, c):
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return True 
        return self.data[r][c] == 1
