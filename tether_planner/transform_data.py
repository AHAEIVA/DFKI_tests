import csv

def transform_data(input_file, output_file):
    """
    Transform CSV data to formatted TXT file with TIMESTAMP, X, Y, Z, PITCH, YAW values.
    Each row in the CSV is expected to have at least 3 values (X, Y, Z) and optionally ROLL, PITCH, YAW.
    """
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        reader = csv.reader(infile)
        timestamp = 0.0
        for row in reader:
            try:
                # Handle different possible CSV formats
                if len(row) >= 3:  # At minimum, we need X, Y, Z
                    x, y, z = map(float, row[:3])
                    
                    # Handle optional pitch and yaw values
                    pitch = float(row[4]) if len(row) > 4 else 0.0
                    yaw = float(row[5]) if len(row) > 5 else 0.0
                    
                    # Format the output string with proper newline character
                    output_string = f"TIMESTAMP: {timestamp:.6f}, X: {x:.6f}, Y: {y:.6f}, Z: {z:.6f}, PITCH: {pitch:.6f}, YAW: {yaw:.6f}\n"
                    outfile.write(output_string)
                    timestamp += 0.1
                else:
                    print(f"Skipping row with insufficient data: {row}")
            except ValueError as e:
                print(f"Skipping row due to error: {e}")
                continue

if __name__ == "__main__":
    input_csv_file = "input_data/sip_plan.csv"
    output_txt_file = "input_data/latest_path.txt"
    
    transform_data(input_csv_file, output_txt_file)
    print(f"Successfully converted {input_csv_file} to {output_txt_file}")