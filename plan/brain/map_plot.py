import pandas as pd
import matplotlib.pyplot as plt

# --- 1. Load CSV ---
csv_file = '/home/bot/ht_ws/src/uvautoboat/plan/brain/map_coordinates.csv'
df = pd.read_csv(csv_file)

# --- 2. Classify objects ---
# Buoys
buoys = df[df['name'].str.contains('buoy', case=False)]
# Posts
posts = df[df['name'].str.contains('post', case=False)]
# Ground stations / antenna / projectiles
others = df[~df['name'].str.contains('buoy|post', case=False)]

# --- 3. Plot map ---
plt.figure(figsize=(14,10))
plt.title("Sydney Regatta Centre VRX Map", fontsize=16)

# Plot buoys
plt.scatter(buoys['x'], buoys['y'], c='red', s=80, label='Buoys', marker='o', edgecolors='black')

# Plot posts
plt.scatter(posts['x'], posts['y'], c='green', s=100, label='Posts', marker='s', edgecolors='black')

# Plot other objects
plt.scatter(others['x'], others['y'], c='blue', s=50, label='Other objects', marker='^', edgecolors='black')

# Add labels for all points (optional)
for i, row in df.iterrows():
    plt.text(row['x'] + 0.5, row['y'] + 0.5, row['name'], fontsize=7, alpha=0.8)

# Styling
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.grid(True, linestyle='--', alpha=0.5)
plt.axis('equal')
plt.legend()
plt.tight_layout()

# --- 4. Save the map as an image ---
plt.savefig('sydney_regatta_map.png', dpi=300)
plt.show()

# --- 5. Prepare data for path planning ---
# Example: buoy coordinates (as obstacles or waypoints)
buoy_coords = list(zip(buoys['x'], buoys['y']))
post_coords = list(zip(posts['x'], posts['y']))
other_coords = list(zip(others['x'], others['y']))

print("Buoy coordinates:", buoy_coords)
print("Post coordinates:", post_coords)
print("Other object coordinates:", other_coords)
