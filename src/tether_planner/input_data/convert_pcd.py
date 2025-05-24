import open3d as o3d
import argparse

def convert_pcd_to_binary(input_path, output_path):
    """
    Converts a PCD file from ASCII to binary format using Open3D.
    """
    try:
        # Read the point cloud
        pcd = o3d.io.read_point_cloud(input_path)

        # Write the point cloud in binary format
        o3d.io.write_point_cloud(output_path, pcd, write_ascii=False)

        print(f"Successfully converted {input_path} to {output_path} (binary)")

    except Exception as e:
        print(f"Error: Could not read or write PCD file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert PCD file from ASCII to binary format.")
    parser.add_argument("input_path", help="Path to the input PCD file (ASCII).")
    parser.add_argument("output_path", help="Path to the output PCD file (binary).")

    args = parser.parse_args()

    convert_pcd_to_binary(args.input_path, args.output_path)
