# Progress: Tether Planner

## What Works

*   ROS Bag data extraction (`rostopic echo`) and processing (`process_ros_txt_to_csv.py`) into CSV format for odometry, orientation, and tether path.
*   Single-run trajectory visualization (`simulate_trajectory.py`) which also calculates and saves coverage data (`coverage_data.csv`). (Coverage calculation bug fixed 2025-04-06 PM).
*   Comparison of two processed planner runs (`compare_planner_runs.py`) using the generated `coverage_data.csv` files, generating metrics (total time, time max tether exceeded, final coverage) and plots (coverage vs. time, tether length vs. time) in PDF format.
*   Generation of a comparative analysis document (`results/comparison/comparison_analysis.md`) summarizing results.

## What's Left to Build / Implement

*   Outstanding features or requirements from the project brief.
*   Planned enhancements or refactoring.

## Current Status (High-Level)

*   Overall project health: On track.
*   Current development phase: Data processing and analysis.
*   (As of 2025-04-06): Completed extraction and processing of `run3` data (`ta_planner_good`, `fc_planner`) into CSV format. Data is organized in `results/run3/` subdirectories.
*   (2025-04-06): Refined workflow: `simulate_trajectory.py` now generates `coverage_data.csv` for a run, and `compare_planner_runs.py` uses these files for comparison.
*   (2025-04-06 PM): Successfully compared `run3` data using specific planner names ("Online Entaglement-Aware PP" and "CPP (without entaglement awareness)"). Results (metrics table, PDF plots) saved in `results/comparison/`.
*   (2025-04-06 PM): Created `results/comparison/comparison_analysis.md` summarizing the comparison results and discussion.
*   (2025-04-07): Attempted `run4` simulation for `fc_planner`, but data extraction failed.
*   (2025-04-07): Updated `compare_planner_runs.py` plot styles (larger text, thicker lines, no titles).
*   (2025-04-07): Re-ran comparison for `run3` data using `CPP` and `OEA-PP` names, saving results to `results/comparison/`.
*   (2025-04-07): Updated results table in `results/comparison/comparison_analysis.md`.

## Known Issues & Bugs

*   List any known defects, performance bottlenecks, or areas needing improvement.
*   Link to issue tracker items if applicable.
