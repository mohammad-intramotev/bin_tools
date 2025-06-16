import os
from common import config

def bin_merge():
    print(f"Merging...")

    # Ensure input directory exists
    if not os.path.exists(config.INPUT_MERGE_DIR):
        print(f"Input directory {config.INPUT_MERGE_DIR} does not exist. Creating it.")
        os.makedirs(config.INPUT_MERGE_DIR)

    # Ensure output directory exists
    if not os.path.exists(config.OUTPUT_MERGE_DIR):
        print(f"Output directory {config.OUTPUT_MERGE_DIR} does not exist. Creating it.")
        os.makedirs(config.OUTPUT_MERGE_DIR)

    # Get all .bin files in the folder, sorted by name
    bin_files = sorted([
        f for f in os.listdir(config.INPUT_MERGE_DIR)
        if f.endswith('.bin') and f.startswith('log_')
    ])

    if not bin_files:
        print("No .bin files found to merge.")
        return

    # Build output filename using first and last file
    first_file = os.path.splitext(bin_files[0])[0]
    last_file = os.path.splitext(bin_files[-1])[0]
    output_filename = f"{first_file}_to_{last_file}.bin"
    output_path = os.path.join(config.OUTPUT_MERGE_DIR, output_filename)

    # Combine files
    with open(output_path, 'wb') as outfile:
        for filename in bin_files:
            file_path = os.path.join(config.INPUT_MERGE_DIR, filename)
            with open(file_path, 'rb') as infile:
                outfile.write(infile.read())

    print(f"Combined {len(bin_files)} files into {output_path}")

if __name__ == "__main__":
    bin_merge()
