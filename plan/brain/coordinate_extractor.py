import xml.etree.ElementTree as ET
import csv

# Path to your SDF world file
world_file = "/home/bot/ht_ws/src/vrx/vrx_gz/worlds/sydney_regatta.sdf"

# Output CSV file
csv_file = "map_coordinates.csv"

# Parse the SDF file
tree = ET.parse(world_file)
root = tree.getroot()

coordinates = []

# Find all <include> tags anywhere inside the world
for include in root.findall(".//include"):
    # Get the name
    name_elem = include.find('name')
    name = name_elem.text.strip() if name_elem is not None else "unnamed"
    
    # Get the pose
    pose_elem = include.find('pose')
    if pose_elem is not None:
        pose_text = pose_elem.text.strip()
        parts = pose_text.split()
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        coordinates.append([name, x, y, z])

# Save to CSV
with open(csv_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['name', 'x', 'y', 'z'])
    writer.writerows(coordinates)

print(f"{len(coordinates)} coordinates extracted and saved to {csv_file}")
