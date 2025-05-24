# Active Context: Tether Planner

## Current Focus

*   (As of 2025-04-07): Finalize comparison results and documentation for `run3` data (`CPP` vs `OEA-PP`).

## Recent Changes

*   (2025-04-06):
    *   Extracted `/rope_path`, `/mobula/rov/odometry`, `/mobula/rov/orientation` topics from `ta_planner_good.bag` and `fc_planner.bag` into `.txt` files in `results/run3/`.
    *   Refactored `process_run2_txt_to_csv.py` into `process_ros_txt_to_csv.py` to accept command-line arguments.
    *   Updated `process_ros_txt_to_csv.py` to convert orientation data to degrees.
    *   Processed the `.txt` files for both `ta_planner_good` and `fc_planner` into `.csv` files using the updated script.
    *   Organized the processed `.csv` files into `results/run3/ta_planner_good/` and `results/run3/fc_planner/`.
    *   Updated `.clinerules` with standard topics and the data extraction/processing workflow.
    *   Created `compare_planner_runs.py` script to analyze and compare two planner runs based on their processed CSV data.
    *   Modified `simulate_trajectory.py` to calculate and save coverage data (`coverage_data.csv`) to the run directory before displaying the plot. Also increased `FRAME_STEP` for faster visualization.
    *   Modified `compare_planner_runs.py` to load `coverage_data.csv` instead of using a placeholder.
    *   Ran `simulate_trajectory.py` for both `fc_planner` and `ta_planner` (`run3` data) to generate their respective `coverage_data.csv` files.
    *   Successfully ran `compare_planner_runs.py` using the generated coverage data. Final comparison results (table and plots) saved in `results/comparison/`.
    *   Updated `.clinerules` again to include `compare_planner_runs.py` and the refined workflow.
    *   Fixed bug in `simulate_trajectory.py` where coverage calculation did not reset correctly between runs.
    *   Modified `compare_planner_runs.py` to save plots as PDF instead of PNG.
    *   Re-ran comparison using specific names: "Online Entaglement-Aware PP" (for `ta_planner_good`) and "CPP (without entaglement awareness)" (for `fc_planner`). Updated PDF plots and metrics table saved in `results/comparison/`.
    *   Created `results/comparison/comparison_analysis.md` summarizing the comparison results and discussion in a scientific format.
*   (2025-04-07):
    *   Attempted to run `fc_planner` simulation and record bag for `run4`, but data extraction failed.
    *   Ran `simulate_trajectory.py` for `results/run3/fc_planner/` to ensure `coverage_data.csv` was up-to-date.
    *   Modified `compare_planner_runs.py` to increase plot text sizes (labels, legends, ticks) and line thickness, and remove plot titles.
    *   Re-ran comparison using `run3` data with planner names `CPP` (for `fc_planner`) and `OEA-PP` (for `ta_planner_good`), saving results to `results/comparison/`.
    *   Updated the results table in `results/comparison/comparison_analysis.md` with the latest `CPP` vs `OEA-PP` data.

## Next Steps

*   Update `progress.md` to reflect the completion of the comparison task and the updates to the analysis document.
*   Await further instructions from the user.

## Active Decisions & Considerations

*   The `run4` data extraction failed. Need to investigate the `fc_planner_run4.bag` file if this run is needed later.
*   The comparison analysis (`comparison_analysis.md`) now reflects the `CPP` vs `OEA-PP` results from `run3`. The discussion text might need review to ensure it aligns perfectly with the updated table values and planner names.
