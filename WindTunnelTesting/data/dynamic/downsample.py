import pandas as pd
from pathlib import Path
import os

# Original data stored in Google Drive

factor = 75
rolling_window = factor

project_root = Path(__file__).parent
data_dir = project_root
downsampled_dir = data_dir / "downsampled"
downsampled_dir.mkdir(exist_ok=True)

csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

for csv in csv_files:
    file = data_dir / csv
    chunk_size = 100000
    downsampled_chunks = []

    for chunk in pd.read_csv(file, chunksize=chunk_size):
        if "Time" not in chunk.columns:
            raise KeyError(f"The CSV file {csv} does not contain a 'Time' column.")

        # Apply rolling mean to smooth numeric columns
        numeric_cols = chunk.select_dtypes(include=["number"]).columns
        chunk[numeric_cols] = chunk[numeric_cols].rolling(rolling_window, min_periods=1).mean()

        # Downsample by taking every Nth row
        chunk = chunk.iloc[::factor]

        # Split "Time" column into Date and ClockTime
        chunk[["Date", "ClockTime"]] = chunk["Time"].str.split(" ", expand=True)

        # Convert "ClockTime" to total seconds since midnight
        chunk["Seconds"] = pd.to_timedelta(chunk["ClockTime"], errors='coerce').dt.total_seconds()

        downsampled_chunks.append(chunk)

    # Combine all chunks
    downsampled_df = pd.concat(downsampled_chunks, ignore_index=True)

    file_out = downsampled_dir / f"{csv[:-4]}_downsampled.csv"
    downsampled_df.to_csv(file_out, index=False)

    print(f"Processed and saved: {file_out}")
