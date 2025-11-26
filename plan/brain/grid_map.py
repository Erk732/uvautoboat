import math

class GridMap:
    def __init__(self, width_m=300, height_m=300, resolution=1.0):
        self.resolution = resolution
        self.width_m = width_m
        self.height_m = height_m
        
        # Calculate grid dimensions
        self.cols = int(width_m / resolution)
        self.rows = int(height_m / resolution)
        
        # The OFFSET is crucial. It puts (0,0) in the middle of the grid
        # instead of the bottom-left corner.
        self.offset_x = width_m / 2.0
        self.offset_y = height_m / 2.0
        
        # Initialize grid with 0 (0 = Free, 1 = Obstacle)
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def reset(self):
        # Clears the map for the new frame (removes ghost obstacles)
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def world_to_grid(self, wx, wy):
        # Apply offset so negative world coords become positive grid indices
        gx = int((wx + self.offset_x) / self.resolution)
        gy = int((wy + self.offset_y) / self.resolution)
        
        # Bounds check
        if 0 <= gx < self.cols and 0 <= gy < self.rows:
            return (gx, gy)
        return None  # Out of bounds

    def grid_to_world(self, gx, gy):
        # Convert back to world coords, removing the offset
        wx = (gx * self.resolution) - self.offset_x
        wy = (gy * self.resolution) - self.offset_y
        return (wx, wy)

    def set_obstacle(self, wx, wy, radius):
        # Simple inflation: Draw a square or circle around the point
        center = self.world_to_grid(wx, wy)
        if not center: return
        
        cx, cy = center
        # Convert radius from meters to grid cells
        r_cells = int(math.ceil(radius / self.resolution))
        
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                nx, ny = cx + dx, cy + dy
                # Check bounds
                if 0 <= nx < self.cols and 0 <= ny < self.rows:
                    self.grid[ny][nx] = 1 # Mark as blocked

    def is_blocked(self, gx, gy):
        if 0 <= gx < self.cols and 0 <= gy < self.rows:
            return self.grid[gy][gx] == 1
        return True # Treat out of bounds as blocked