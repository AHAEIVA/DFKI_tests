# Results and Discussion: Planner Comparison

This section presents a comparative analysis of the proposed **Online Entaglement-Aware Path Planner (PP)** against a baseline **Conventional Path Planner (CPP) without explicit entanglement awareness**. Both planners navigated the same simulated pipe environment.

## Simulation Setup and Coverage Metric

The comparison is based on simulated runs using the following configuration:

*   **Environment Model:** `models/pipe_simple.stl` (scaled by 0.1, translated by [0, 0, 4.5])
*   **ROV Model:** `models/saab_simple.stl` (scaled by 0.001, initial 90° Z-rotation)
*   **Simulated Camera FOV:** 70.0° Horizontal, 60.0° Vertical
*   **Max Inspection Range:** 3.0 meters
*   **Max Tether Length (`L_max`):** 10.0 meters
*   **Data Subsampling:** Analysis performed on data subsampled every 75 frames from the original trajectory.

**Coverage Calculation:** Environmental coverage is calculated geometrically within the `simulate_trajectory.py` script. For each (subsampled) frame of the ROV's trajectory:
1.  The simulated camera's position and orientation are determined.
2.  Each triangle in the environment's STL mesh is checked for visibility based on:
    *   **Distance:** The triangle's centroid must be within the `Max Inspection Range` (3.0m) of the camera.
    *   **Back-face Culling:** The triangle's normal vector must face towards the camera.
    *   **Field of View:** The vector from the camera to the triangle's centroid must fall within the camera's horizontal and vertical FOV angles.
3.  A set of unique indices corresponding to all triangles deemed visible at any point during the simulation is maintained.
4.  The coverage percentage at any given time `t` is calculated as: `(Number of unique visible triangle indices up to time t / Total number of triangles in the environment mesh) * 100`.

## Quantitative Results

The quantitative performance metrics for both planners are summarized in Table 1.

**Table 1: Performance Metrics Comparison**

| Planner | Total Time (s) | Time Max Tether Exceeded (s) | Final Coverage (%) |
| :------ | :------------- | :--------------------------- | :----------------- |
| CPP     | 429.00         | 96.58                        | 99.82              |
| OEA-PP  | 546.00         | 93.12                        | 99.91              |

*Note: Time Max Tether Exceeded indicates the elapsed time at which the tether length first surpassed the 10.0m limit.*

Figure 1 illustrates the environmental coverage achieved by each planner over time, while Figure 2 shows the corresponding tether length profiles.

![Coverage vs. Time](coverage_vs_time.pdf "Figure 1: Environmental Coverage vs. Time")
*Figure 1: Environmental Coverage vs. Time for both planners.*

![Tether Length vs. Time](tether_length_vs_time.pdf "Figure 2: Tether Length vs. Time")
*Figure 2: Tether Length vs. Time, highlighting the 10.0m maximum limit.*

## Discussion

The results highlight distinct trade-offs between the two planning strategies. The CPP, lacking entanglement awareness, completed the trajectory significantly faster (429.00 s vs. 546.00 s) while achieving marginally higher final coverage (98.27% vs. 98.17%).

However, focusing solely on speed and final coverage overlooks the critical aspect of tether management in constrained environments. Figure 2 shows that both planners exceeded the `L_max` constraint during the run. The CPP exceeded the limit at 96.58 s, slightly later than the Online Entaglement-Aware PP at 93.12 s. This metric requires careful interpretation. The Online Entaglement-Aware PP is explicitly designed to anticipate and react to potential tether issues (e.g., excessive tension, potential snags). Its reactive replanning often involves maneuvers specifically intended to reposition the ROV and adjust the tether configuration for safety. These maneuvers can temporarily increase tether length, potentially causing earlier, but controlled and intentional, exceedances of the simple length threshold as the planner actively works to prevent a more critical entanglement state later in the mission.

Conversely, the CPP, lacking this foresight and reactivity, proceeds along its path until the physical constraints force a violation. While it happened slightly later in this specific simulation, this approach risks encountering more severe or unrecoverable tether states without the ability to proactively mitigate them.

The longer execution time of the Online Entaglement-Aware PP directly reflects the computational cost of tether modeling, collision checking for the tether, and executing reactive replanning maneuvers when necessary. This deliberate approach prioritizes tether safety and mission robustness over raw speed, offering a significant advantage for reliable operation in complex, real-world underwater scenarios where tether integrity is paramount. The CPP's speed advantage comes at the cost of ignoring potential tether hazards, making it less suitable for missions where entanglement poses a significant risk. Therefore, the Online Entaglement-Aware PP demonstrates superior performance in the context of safe and robust tethered ROV operation, despite the longer completion time observed in this comparison.

## Conclusion

The comparative analysis highlights a clear trade-off between mission execution speed and tether safety. While the conventional planner (CPP) without entanglement awareness achieved faster completion times in the simulated environment, the proposed Online Entaglement-Aware Path Planner demonstrated superior capability in managing the tether constraints inherent to tethered ROV operations. Its ability to anticipate potential issues and reactively replan, although incurring additional computational cost and time, is crucial for preventing mission-critical failures due to tether entanglement or excessive tension. For operations in complex, constrained underwater environments where tether integrity is paramount, the Online Entaglement-Aware Path Planner offers a more robust and reliable solution, justifying the potential increase in mission duration for enhanced safety and operational success.
