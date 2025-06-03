import numpy as np
import os

def generate_and_save_helical_path(
    cylinder_center,
    cylinder_yaw,
    output_filename="dfki_pipe_helix.txt",
    cylinder_diameter=0.7,
    cylinder_length=2.0,
    safe_distance=0.5,
    pitch=1.0,
    num_points_per_revolution=36,
    time_increment=0.1,
):
    """
    Generates a helical path around a cylinder and saves it to a file in a
    specific format.

    Args:
        cylinder_center (np.ndarray): A 3-element numpy array for the (x, y, z)
                                     center of the cylinder.
        cylinder_yaw (float): The yaw of the cylinder in radians.
        output_filename (str): The name of the file to save the path to.
        cylinder_diameter (float, optional): The cylinder's diameter in meters.
                                             Defaults to 0.7.
        cylinder_length (float, optional): The cylinder's length in meters.
                                           Defaults to 2.0.
        safe_distance (float, optional): The safe distance from the cylinder's
                                        surface in meters. Defaults to 0.5.
        pitch (float, optional): The distance the helix travels along the
                                 cylinder's axis in one revolution. Defaults to 1.0.
        num_points_per_revolution (int, optional): The number of points to generate
                                                   per revolution. Defaults to 36.
        time_increment (float, optional): The time step between consecutive points.
                                          Defaults to 0.1.
    """

    # --- 1. Define Cylinder and Helix Parameters ---
    cylinder_radius = cylinder_diameter / 2
    path_radius = cylinder_radius + safe_distance
    num_revolutions = cylinder_length / pitch
    total_num_points = int(num_revolutions * num_points_per_revolution)

    # --- 2. Create the Helical Path in a Local Frame ---
    t = np.linspace(0, cylinder_length, total_num_points)
    theta = (t / pitch) * 2 * np.pi  # Angle of rotation

    local_path = np.zeros((total_num_points, 3))
    local_path[:, 0] = t - (cylinder_length / 2)
    local_path[:, 1] = path_radius * np.sin(theta)
    local_path[:, 2] = path_radius * np.cos(theta)

    local_yaw = np.arctan2(-local_path[:, 1], -local_path[:, 2])

    # --- 3. Transform the Path to the Global Frame ---
    # rotation_matrix = np.array([
    #     [np.cos(cylinder_yaw), -np.sin(cylinder_yaw), 0],
    #     [np.sin(cylinder_yaw), np.cos(cylinder_yaw), 0],
    #     [0, 0, 1]
    # ])

    rotation_matrix = np.array([
        [1, 0, 0],
        [0.0, np.cos(cylinder_yaw), -np.sin(cylinder_yaw)],
        [0, np.sin(cylinder_yaw), np.cos(cylinder_yaw)]
    ])

    global_path_xyz = np.dot(local_path, rotation_matrix.T) + cylinder_center
    global_yaw = local_yaw + cylinder_yaw

    # --- 4. Generate Timestamps and Format Output ---
    timestamps = np.arange(0, total_num_points * time_increment, time_increment)
    
    # As per the request, the PITCH value in the output is always 0.0
    output_pitch_value = 0.0

    # --- 5. Write to File ---
    try:
        with open(output_filename, 'w') as f:
            for i in range(total_num_points):
                timestamp = timestamps[i]
                x = global_path_xyz[i, 0]
                y = global_path_xyz[i, 1]
                z = global_path_xyz[i, 2]
                yaw = global_yaw[i]

                # Format the line as per the requested format
                line = (f"TIMESTAMP: {timestamp:.6f}, "
                        f"X: {x:.6f}, Y: {y:.6f}, Z: {z:.6f}, "
                        f"PITCH: {output_pitch_value:.6f}, YAW: {yaw:.6f}\n")
                f.write(line)
        
        print(f"Successfully generated and saved the helical path to '{os.path.abspath(output_filename)}'")

    except IOError as e:
        print(f"Error writing to file {output_filename}: {e}")


if __name__ == '__main__':
    # --- Example Usage ---
    # Define the cylinder's properties based on the previous example
    center_of_cylinder = np.array([0.4, 0.0, 0.0])  # (x, y, z) position
    yaw_of_cylinder = np.deg2rad(0.0)  # 45 degrees yaw

    # Define the desired path parameters
    safety_margin = 0.7  # meters
    helix_pitch = 1.2    # meters for calculation, but will be 0.0 in output
    
    # Define the output file name
    output_file = "dfki_pipe_helix.txt"

    # Generate the path and save it to the specified file
    generate_and_save_helical_path(
        cylinder_center=center_of_cylinder,
        cylinder_yaw=yaw_of_cylinder,
        output_filename=output_file,
        safe_distance=safety_margin,
        pitch=helix_pitch
    )

    # Optional: Print the first few lines of the generated file for verification
    try:
        with open(output_file, 'r') as f:
            print("\n--- Start of dfki_pipe_helix.txt ---")
            for i, line in enumerate(f):
                if i < 5:
                    print(line.strip())
            print("...")
            print("--- End of file preview ---")
    except FileNotFoundError:
        pass