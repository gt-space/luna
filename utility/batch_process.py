import subprocess
import argparse
import time
from pathlib import Path

def main():
    # Setup the command line interface
    parser = argparse.ArgumentParser(description="Batch process postcard logs using cargo run.")
    
    # Define the flags for input and output
    parser.add_argument("-i", "--input", required=True, help="Directory containing .postcard files")
    parser.add_argument("-o", "--output", required=True, help="Directory where .csv files will be saved")
    parser.add_argument("-s", "--sleep", type=int, default=3, help="Seconds to wait between files (default: 3)")

    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    # 1. Ensure the output directory exists
    if not output_path.exists():
        print(f"Creating output directory: {output_path}")
        output_path.mkdir(parents=True, exist_ok=True)

    # 2. Get all .postcard files and sort them
    postcard_files = sorted(list(input_path.glob("*.postcard")))

    if not postcard_files:
        print(f"No .postcard files found in {input_path}")
        return

    total_files = len(postcard_files)
    print(f"Found {total_files} files. Starting batch process with 'cargo run'...\n")

    # 3. Execution Loop
    for i, file in enumerate(postcard_files, 1):
        # Swap extension for the output filename
        csv_filename = file.with_suffix(".csv").name
        destination = output_path / csv_filename

        print(f"[{i}/{total_files}] Processing: {file.name}")
        
        # Build the cargo run command
        # Everything after the '--' is passed directly to your Rust program
        cmd = [
            "cargo", "run", "--release", "--",
            "--input", str(file),
            "--output", str(destination)
        ]

        try:
            # Run the command and wait for it to finish
            # We don't capture output so you can see your Rust println! logs in real-time
            subprocess.run(cmd, check=True)
            print(f"Finished: {file.name}")
        except subprocess.CalledProcessError:
            print(f"!! Error: Cargo failed to process {file.name}.")
        except KeyboardInterrupt:
            print("\nBatch process stopped by user.")
            break

        # 4. Cooldown (3 second wait)
        if i < total_files:
            print(f"Waiting {args.sleep} seconds...")
            time.sleep(args.sleep)

    print("\n--- All files processed. ---")

if __name__ == "__main__":
    main()
