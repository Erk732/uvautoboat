import math

class GridMap:
    def __init__(self, width_m=300, height_m=300, resolution=1.0):
        """
        width_m: Total width of the area in meters
        height_m: Total height of the area in meters
        resolution: How big each cell is (1.0 = 1 meter per cell)
        """
        self.resolution = resolution
        self.cols = int(width_m / resolution)
        self.rows = int(height_m / resolution)
        
        # Center of the grid in indices
        self.origin_x_idx = self.cols // 2
        self.origin_y_idx = self.rows // 2
        
        # The Grid itself (2D List). 0 = Water, 1 = Obstacle
        self.data = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def world_to_grid(self, x, y):
        """ Converts real world (x,y) to grid indices (row, col) """
        c = int(x / self.resolution) + self.origin_x_idx
        r = int(y / self.resolution) + self.origin_y_idx
        
        # Check if point is inside our map
        if 0 <= c < self.cols and 0 <= r < self.rows:
            return (r, c)
        return None # Out of bounds

    def grid_to_world(self, r, c):
        """ Converts grid indices (row, col) back to real world (x,y) """
        x = (c - self.origin_x_idx) * self.resolution
        y = (r - self.origin_y_idx) * self.resolution
        return (x, y)

    def set_obstacle(self, x, y, radius=2.0):
        """ Marks a point (and surroundings) as an obstacle """
        center = self.world_to_grid(x, y)
        if center is None: return

        r_center, c_center = center
        
        # Simple square inflation for safety
        inflation = int(radius / self.resolution)
        
        for r in range(r_center - inflation, r_center + inflation + 1):
            for c in range(c_center - inflation, c_center + inflation + 1):
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    self.data[r][c] = 1 # Mark as Occupied

    def is_blocked(self, r, c):
        """ Returns True if the cell is an obstacle or out of bounds """
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return True # Out of bounds is technically blocked
        return self.data[r][c] == 1

# --- Simple Test Code ---
if __name__ == '__main__':
    # Create a 10x10 meter map
    my_map = GridMap(width_m=10, height_m=10, resolution=1.0)
    
    # Add an obstacle at (0,0) - The center
    print("Adding obstacle at (0,0)...")
    my_map.set_obstacle(0.0, 0.0, radius=0)
    
    # Check points
    test_points = [(0,0), (2,2), (-50, -50)]
    
    for x, y in test_points:
        idx = my_map.world_to_grid(x, y)
        if idx:
            r, c = idx
            status = "BLOCKED" if my_map.is_blocked(r, c) else "FREE"
            print(f"Point ({x}, {y}) -> Grid [{r}][{c}] -> {status}")
        else:
            print(f"Point ({x}, {y}) -> OUT OF BOUNDS")
