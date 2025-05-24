
# Tether Planner ROS

This repository provides a ROS package for a tether model and a tether-aware path planner.

## Installation

Follow these steps to set up your environment and install the necessary dependencies.

---

### 1. Install WSL2 on Windows

1. Open **PowerShell** as Administrator and run the following command to enable WSL:
   ```bash
   wsl --install
   ```

2. Restart your system after installation.

3. If you already have WSL 1, you can upgrade to WSL 2 by running:
   ```bash
   wsl --set-default-version 2
   ```

---

### 2. Install Ubuntu 20.04 from Microsoft Store

1. Go to the **Microsoft Store** and search for **Ubuntu 20.04**.

2. Click **Install** to install Ubuntu 20.04.

3. Once installed, launch the **Ubuntu 20.04** application, set up a user, and complete the initial setup.

---

### 3. Install Basic Tools and Dependencies

Run the following command to install basic tools and dependencies:

```bash
sudo apt update
sudo apt install gedit
sudo apt install terminator
sudo apt install nlohmann-json3-dev
sudo apt install libgoogle-glog-dev
sudo apt install libunwind-dev
```

---

### 4. Install ROS Noetic

1. Follow the instructions on the [ROS Noetic installation page](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. Add the ROS repository:
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

3. Set up your keys:
   ```bash
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

4. Update your package list and install ROS Noetic:
   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

5. Set up your ROS environment:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```

6. Install `rosdep` to handle dependencies:
   ```bash
   sudo apt install python-rosdep
   sudo rosdep init
   rosdep update
   ```

---

### 5. Install CUDA

To install CUDA, follow the instructions on the [NVIDIA website](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local).

1. After installation, check if CUDA is correctly installed with:
   ```bash
   nvcc --version
   ```

2. If you need to install CUDA manually, run:
   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
   sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
   wget https://developer.download.nvidia.com/compute/cuda/12.8.0/local_installers/cuda-repo-wsl-ubuntu-12-8-local_12.8.0-1_amd64.deb
   sudo dpkg -i cuda-repo-wsl-ubuntu-12-8-local_12.8.0-1_amd64.deb
   sudo cp /var/cuda-repo-wsl-ubuntu-12-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
   sudo apt-get update
   sudo apt-get -y install cuda-toolkit-12-8
   ```

3. Add CUDA to your environment:
   ```bash
   echo 'export PATH=/usr/local/cuda-12.4/bin:$PATH' >> ~/.bashrc
   echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
   source ~/.bashrc
   ```

---

### 6. Install Dependencies for OMPL and NVBLOX

Before installing OMPL and NVBLOX, make sure you have the required dependencies installed.

Run the following to install the necessary packages:

```bash
sudo apt-get update && sudo apt-get install git jq gnupg apt-utils software-properties-common build-essential sudo python3-pip wget sudo git python3-dev git-lfs
```

---

### 7. Install OMPL Library

1. Clone the OMPL repository:

   ```bash
   cd ~
   git clone https://github.com/ompl/ompl.git
   ```

2. Build and install OMPL:

   ```bash
   cd ompl
   mkdir build
   cmake ..
   make
   sudo make install
   ```

---

### 8. Install NVBLOX Library

1. Clone the NVBLOX repository:

   ```bash
   cd ~
   git clone https://github.com/NVlabs/nvblox.git
   ```

2. Build and install NVBLOX:

   ```bash
   cd nvblox
   mkdir build
   cmake ..
   make
   sudo make install
   ```

---

Once these steps are completed, you will have the environment set up for the Tether Planner ROS package.

---

## Generating Results (Data Processing & Analysis Workflow)

This section outlines the steps to process data recorded in ROS bag files and generate analysis results or visualizations.

**Standard Topics:** The typical topics used for analysis are:
*   `/rope_path` (`visualization_msgs/Marker`)
*   `/mobula/rov/odometry` (`nav_msgs/Odometry`)
*   `/mobula/rov/orientation` (`geometry_msgs/Vector3Stamped`)

**Workflow:**

1.  **Extract Raw Data from ROS Bag:**
    *   Use `rostopic echo` to extract the relevant topics from your `.bag` file into text files (YAML stream format). Place these files in a dedicated directory, e.g., `results/my_run_data/`.
    *   **Important:** Due to potential subtle bag file corruption (see `.clinerules` - Known Challenges), it's recommended to use `rosbag play` in one terminal and `rostopic echo` in another, rather than directly reading from the bag file with scripts.
    *   Example commands (run `rosbag play` first):
        ```bash
        # Terminal 1: Play the bag file
        rosbag play <path_to_your_bag_file.bag>

        # Terminal 2: Echo topics to text files
        rostopic echo /rope_path > results/my_run_data/tether_raw.txt
        rostopic echo /mobula/rov/odometry > results/my_run_data/odom_raw.txt
        rostopic echo /mobula/rov/orientation > results/my_run_data/orient_raw.txt
        ```
    *   *Wait for `rosbag play` to finish before proceeding.*

2.  **Process Text Data to CSV:**
    *   Use the `process_ros_txt_to_csv.py` script to convert the raw text files into structured CSV files. This script handles Odometry, Orientation (converted to degrees), and Tether Path messages.
    *   Organize the output CSV files into a run-specific subdirectory, often named after the planner or experiment variation (e.g., `results/runX/planner_name/`).
    *   Example command:
        ```bash
        python3 process_ros_txt_to_csv.py \
          --odom-txt results/my_run_data/odom_raw.txt \
          --orient-txt results/my_run_data/orient_raw.txt \
          --tether-txt results/my_run_data/tether_raw.txt \
          --odom-csv results/runX/planner_name/odom.csv \
          --orient-csv results/runX/planner_name/orient.csv \
          --tether-csv results/runX/planner_name/tether.csv
        ```
    *   Replace `runX/planner_name` with your desired output structure.

3.  **Visualize a Single Run:**
    *   Use `simulate_trajectory.py` to visualize the processed trajectory from a single run and generate coverage data.
    *   Point the `--data-dir` argument to the directory containing the processed CSV files for that run.
    *   This script will also save `coverage_data.csv` within the specified `--data-dir`.
    *   Example command:
        ```bash
        python3 simulate_trajectory.py --data-dir results/runX/planner_name/
        ```

4.  **Compare Two Runs:**
    *   Use `compare_planner_runs.py` to compare metrics and generate plots for two different processed runs.
    *   Ensure you have already run `simulate_trajectory.py` (Step 3) for both runs you want to compare, as this script relies on the `coverage_data.csv` files generated by it.
    *   Provide the paths to the data directories for both runs, names for labeling, the maximum tether length (`L_max`), and the path to the environment's STL file.
    *   Example command:
        ```bash
        python3 compare_planner_runs.py \
          --data-dir1 results/runX/planner1_name/ --name1 "Planner 1 Display Name" \
          --data-dir2 results/runY/planner2_name/ --name2 "Planner 2 Display Name" \
          --l-max <max_tether_length_value> \
          --stl-path <path_to_environment.stl> \
          --output-dir results/comparison_output # Optional: Specify output directory for plots/metrics
        ```
    *   Replace placeholders (`<...>`) with your specific values. The comparison results (plots, metrics table) will be saved in the specified output directory (or `results/comparison/` by default).
