import numpy as np
import math

def generate_pipe_point_cloud(pipe_density=1.0, plate_density=1.0):
    """
    Generate a dense point cloud for a 3D pipe with support plates
    
    Specifications:
    - Pipe: length=2m, diameter=0.7m, oriented along x-axis
    - Support plates: length=0.5m, thickness=0.05m at both ends
    
    Parameters:
    - pipe_density: float, multiplier for pipe point density (1.0 = default, 2.0 = double density, 0.5 = half density)
    - plate_density: float, multiplier for plate point density (1.0 = default, 2.0 = double density, 0.5 = half density)
    """
    
    # Pipe parameters
    pipe_length = 3.0 * 10  # meters
    pipe_radius = 0.3 * 10 # meters (diameter = 0.7m)
    
    # Support plate parameters
    plate_length = 0.5  # meters (along Y-axis)
    plate_thickness = 0.09  # meters (along X-axis)
    plate_height = pipe_radius + 0.01  # meters (Z-axis) - extends from ground to pipe bottom + margin
    
    # Point cloud density parameters (base values)
    base_pipe_circumferential = 150
    base_pipe_longitudinal = 600
    base_pipe_radial = 25
    
    # Apply density multipliers
    pipe_circumferential_points = max(10, int(base_pipe_circumferential * pipe_density))
    pipe_longitudinal_points = max(10, int(base_pipe_longitudinal * pipe_density))
    pipe_radial_points = max(5, int(base_pipe_radial * pipe_density))
    
    # Dense plate parameters (base values)
    base_grid_x = 15
    base_grid_y = 50
    base_grid_z = 60
    
    # Apply density multipliers
    grid_size_x = max(3, int(base_grid_x * plate_density))
    grid_size_y = max(5, int(base_grid_y * plate_density))
    grid_size_z = max(5, int(base_grid_z * plate_density))
    
    points = []
    
    print(f"Pipe density settings: {pipe_circumferential_points} × {pipe_longitudinal_points} × {pipe_radial_points}")
    print("Generating pipe point cloud...")
    
    # Generate pipe points (solid cylinder - filled, not hollow)
    for i in range(pipe_longitudinal_points):
        x = (i / (pipe_longitudinal_points - 1)) * pipe_length
        
        for j in range(pipe_circumferential_points):
            theta = (j / pipe_circumferential_points) * 2 * math.pi
            
            # Generate points throughout the entire radius (solid pipe)
            for k in range(pipe_radial_points):
                r = (k / (pipe_radial_points - 1)) * pipe_radius  # From center to outer radius
                y = r * math.cos(theta)
                z = r * math.sin(theta)
                points.append([x, y, z])
    
    print(f"Generated {len(points)} pipe points")
    
    # Generate left support plate (underneath and touching the pipe at x = 0)
    print(f"Generating left support plate... (density: {grid_size_x} × {grid_size_y} × {grid_size_z})")
    plate_x_start = 0  # At the very beginning of the pipe
    
    # Create grid of points for the plate volume
    grid_size_x = 8   # Points through thickness (X direction)
    grid_size_y = 25  # Points along length (Y direction) 
    grid_size_z = 20  # Points through height (Z direction)
    
    for i in range(grid_size_x):  # X direction (thickness)
        for j in range(grid_size_y):  # Y direction (length)
            for k in range(grid_size_z):  # Z direction (height)
                x = plate_x_start + (i / max(1, grid_size_x - 1)) * plate_thickness
                y = -plate_length/2 + (j / max(1, grid_size_y - 1)) * plate_length
                z = -plate_height + (k / max(1, grid_size_z - 1)) * plate_height
                
                # Only include points within the plate bounds and ensure contact with pipe
                if abs(y) <= plate_length/2 and z <= -pipe_radius + 0.001:
                    points.append([x, y, z])
    
    # Generate right support plate (underneath and touching the pipe at x = pipe_length)
    print(f"Generating right support plate... (density: {grid_size_x} × {grid_size_y} × {grid_size_z})")
    plate_x_start = pipe_length - plate_thickness
    
    for i in range(grid_size_x):  # X direction (thickness)
        for j in range(grid_size_y):  # Y direction (length)
            for k in range(grid_size_z):  # Z direction (height)
                x = plate_x_start + (i / max(1, grid_size_x - 1)) * plate_thickness
                y = -plate_length/2 + (j / max(1, grid_size_y - 1)) * plate_length
                z = -plate_height + (k / max(1, grid_size_z - 1)) * plate_height
                
                # Only include points within the plate bounds and ensure contact with pipe
                if abs(y) <= plate_length/2 and z <= -pipe_radius + 0.001:
                    points.append([x, y, z])
    
    print(f"Total points generated: {len(points)}")
    
    return np.array(points)

def save_pcd_file(points, filename="pipe_model.pcd"):
    """
    Save point cloud data to a PCD file format
    """
    print(f"Saving point cloud to {filename}...")
    
    num_points = len(points)
    
    with open(filename, 'w') as f:
        # Write PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # Write point data
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    
    print(f"Point cloud saved successfully!")
    print(f"File: {filename}")
    print(f"Total points: {num_points}")

def print_model_statistics(points):
    """
    Print statistics about the generated point cloud
    """
    # Calculate plate height (same as in generate function)
    pipe_radius = 0.35
    plate_height = pipe_radius + 0.1
    
    print("\n" + "="*50)
    print("POINT CLOUD MODEL STATISTICS")
    print("="*50)
    
    print(f"Total points: {len(points):,}")
    print(f"Point cloud bounds:")
    print(f"  X: {np.min(points[:, 0]):.3f} to {np.max(points[:, 0]):.3f} meters")
    print(f"  Y: {np.min(points[:, 1]):.3f} to {np.max(points[:, 1]):.3f} meters")
    print(f"  Z: {np.min(points[:, 2]):.3f} to {np.max(points[:, 2]):.3f} meters")
    
    print(f"\nModel specifications:")
    print(f"  Pipe: SOLID (filled) cylinder - 2.0m length × 0.7m diameter")
    print(f"  Support plates: SOLID - 0.5m (length) × 0.08m (thickness) × {plate_height:.2f}m (height)")
    print(f"  Plate positions: At x=0 and x=2.0 (pipe ends)")
    print(f"  Plates extend from ground to pipe bottom (no gaps)")
    print(f"  Point density: Very high - no gaps in geometry")
    print(f"  Orientation: Pipe along X-axis, plates underneath providing full support")

def main():
    """
    Main function to generate and save the pipe point cloud
    """
    print("3D Pipe Point Cloud Generator")
    print("="*40)
    
    # Density control parameters
    # Adjust these values to control point cloud density:
    # 1.0 = default density, 2.0 = double density, 0.5 = half density
    PIPE_DENSITY = 0.4     # Controls pipe point density
    PLATE_DENSITY = 1.0     # Controls support plate point density
    
    print(f"Density settings:")
    print(f"  Pipe density multiplier: {PIPE_DENSITY}x")
    print(f"  Plate density multiplier: {PLATE_DENSITY}x")
    print()
    
    # Generate the point cloud
    points = generate_pipe_point_cloud(pipe_density=PIPE_DENSITY, plate_density=PLATE_DENSITY)
    
    # Print statistics
    print_model_statistics(points)
    
    # Save to PCD file
    filename = f"pipe_with_supports_p{PIPE_DENSITY}_s{PLATE_DENSITY}.pcd"
    save_pcd_file(points, filename)
    
    # Also save as numpy array for further processing if needed
    np_filename = f"pipe_points_p{PIPE_DENSITY}_s{PLATE_DENSITY}.npy"
    np.save(np_filename, points)
    print(f"Point cloud also saved as NumPy array: {np_filename}")
    
    print("\nGeneration complete!")
    print("You can now load the PCD file in any point cloud viewer")
    print("such as CloudCompare, PCL viewer, or Open3D")
    print("\nTo adjust density, modify PIPE_DENSITY and PLATE_DENSITY in main() function")

if __name__ == "__main__":
    main()