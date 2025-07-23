#!/usr/bin/env python3

import argparse
import os
import sys

def get_first_timestamp(file_path):
    """
    Reads a TUM file and returns the timestamp from the first valid data line.
    
    Args:
        file_path (str): The path to the TUM file.

    Returns:
        float: The first timestamp in seconds.
    
    Raises:
        ValueError: If no valid data lines are found in the file.
    """
    with open(file_path, 'r') as f:
        for line in f:
            # Skip comments or empty lines
            if line.strip().startswith('#') or not line.strip():
                continue
            
            parts = line.strip().split()
            if len(parts) >= 1:
                try:
                    return float(parts[0])
                except ValueError:
                    # Continue if the first part is not a float, in case of malformed lines
                    continue
    
    raise ValueError(f"Could not find any valid timestamped data in file: {file_path}")


def align_trajectory_timestamps(ref_file, input_file, output_file):
    """
    Aligns the timestamps of an input trajectory file to a reference trajectory file.

    Args:
        ref_file (str): Path to the reference TUM file (e.g., ground truth).
        input_file (str): Path to the input TUM file to be aligned (e.g., SLAM output).
        output_file (str): Path to save the new, time-aligned TUM file.
    """
    print("--- Trajectory Time Synchronization ---")
    try:
        ref_start_time = get_first_timestamp(ref_file)
        input_start_time = get_first_timestamp(input_file)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Calculate the offset in seconds
    offset_s = ref_start_time - input_start_time

    print(f"Reference start time: {ref_start_time:.6f} s")
    print(f"Input start time:     {input_start_time:.6f} s")
    print(f"Calculated offset:    {offset_s:.6f} s")
    print(f"Applying this offset to all timestamps in '{os.path.basename(input_file)}'.")
    
    aligned_lines = []
    with open(input_file, 'r') as f:
        for line in f:
            if line.strip().startswith('#') or not line.strip():
                aligned_lines.append(line)
                continue

            parts = line.strip().split()
            if len(parts) == 8:
                original_timestamp = float(parts[0])
                aligned_timestamp = original_timestamp + offset_s
                new_line = f"{aligned_timestamp:.6f} {' '.join(parts[1:])}\n"
                aligned_lines.append(new_line)
            else:
                aligned_lines.append(line)

    with open(output_file, 'w') as f:
        f.writelines(aligned_lines)

    print(f"\n✅ Time alignment complete!")
    print(f"   Aligned trajectory saved to '{output_file}'")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Aligns a TUM trajectory file's timestamps to a reference file.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        'reference_traj',
        type=str,
        help="Path to the reference trajectory file (e.g., ground_truth.txt)."
    )
    parser.add_argument(
        'input_traj',
        type=str,
        help="Path to the trajectory file you want to align (e.g., traj_lidar.txt)."
    )
    parser.add_argument(
        '--output_traj',
        type=str,
        default=None,
        help=("Path for the output time-aligned file.\n"
              "Defaults to 'input_filename_aligned.txt'.")
    )
    args = parser.parse_args()

    if args.output_traj is None:
        base, ext = os.path.splitext(args.input_traj)
        args.output_traj = f"{base}_aligned{ext}"

    align_trajectory_timestamps(args.reference_traj, args.input_traj, args.output_traj)
