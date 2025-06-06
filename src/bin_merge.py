import os
from common import constants

def merge():
    # Get all .bin files in the folder, sorted by name
    bin_files = sorted([
        f for f in os.listdir(constants.INPUT_MERGE_BIN)
        if f.endswith('.bin') and f.startswith('log_')
    ])

    # Combine files
    with open(constants.OUTPUT_MERGE_BIN, 'wb') as outfile:
        for filename in bin_files:
            file_path = os.path.join(constants.INPUT_MERGE_BIN, filename)
            with open(file_path, 'rb') as infile:
                outfile.write(infile.read())

    print(f"Combined {len(bin_files)} files into {constants.OUTPUT_MERGE_BIN}")