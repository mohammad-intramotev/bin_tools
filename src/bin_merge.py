import os
from common import config

def bin_merge():
    print(f"Merging...")

    # Get all .bin files in the folder, sorted by name
    bin_files = sorted([
        f for f in os.listdir(config.INPUT_MERGE_DIR)
        if f.endswith('.bin') and f.startswith('log_')
    ])

    # Combine files
    with open(config.OUTPUT_MERGE_BIN, 'wb') as outfile:
        for filename in bin_files:
            file_path = os.path.join(config.INPUT_MERGE_DIR, filename)
            with open(file_path, 'rb') as infile:
                outfile.write(infile.read())

    print(f"Combined {len(bin_files)} files into {config.OUTPUT_MERGE_BIN}")

if __name__ == "__main__":
    bin_merge()