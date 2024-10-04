import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import os
import seaborn as sns

sns.set_style("whitegrid")

def clean_openrocket_data(df, alpha):
    assert 0 <= alpha <= 1, 'Alpha must be between 0 and 1'

    # Skip rows with event information based on the '#' identifier in the 'Time' column
    df = df[~df['# Time (s)'].astype(str).str.startswith('#')]

    # Convert the 'Time' column to integers
    df["time"] = df['# Time (s)'].astype(float)

    # Apply Exponential Weighted Moving Average (EWMA) smoothing
    df["smoothed_altitude"] = df['Altitude (ft)'].ewm(alpha=alpha, min_periods=1).mean()
    df["smoothed_velocity"] = df['Vertical velocity (ft/s)'].ewm(alpha=alpha, min_periods=1).mean()
    df["smoothed_acceleration"] = df['Vertical acceleration (ft/s²)'].ewm(alpha=alpha, min_periods=1).mean()

    return df

# def values_for_ahmand(df):
#     try:
#         # Extract velocity and density

#         filtered_df = df[(df['time'] >= 2.129) & (df['time'] <= 17.457) & (df['smoothed_velocity'] > 0)]

#         velocity = filtered_df['smoothed_velocity'] 
#         rho = filtered_df['smoothed_altitude'].apply(density)
#         mach = filtered_df['Mach number (​)']

#         # Plot velocity vs density
#         plt.figure(figsize=(10, 6))
#         plt.plot(velocity, rho, 'o', markersize=4)
#         plt.ylabel('Density (slug/ft³)')
#         plt.xlabel('Velocity (ft/s)')
#         plt.title('Velocity vs Density')
#         plt.grid(True)
#         plt.show()

#         # Export as a CSV file
#         export_df = pd.DataFrame({
#             'Density (slug/ft³)': rho,
#             'Velocity (ft/s)': velocity,
#             'Mach': mach
#         })
#         export_df.to_csv('/Users/andrewwehmeyer/Library/Mobile Documents/com~apple~CloudDocs/NUSTARS/Coding/Monte/velocity_density.csv', index=False)
#         print('Exported velocity and density data to CSV.')
#     except KeyError as e:
#         print(f'Error: Missing column in DataFrame - {e}')
#     except Exception as e:
#         print(f'An error occurred: {e}')

# def combined_plot(dfs):
#     fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

#     # Titles and labels for the axes
#     axs[0].set_title('Altitude')
#     axs[0].set_ylabel('Altitude [ft]')
    
#     axs[1].set_title('Velocity vs. Time')
#     axs[1].set_ylabel('Velocity [ft/s]')
    
#     axs[2].set_title('Acceleration vs Time')
#     axs[2].set_xlabel('Time [s]')
#     axs[2].set_ylabel('Acceleration [ft/s²]')
    
#     wind_speeds = [0, 5, 10, 15, 20]
#     colors = ['b', 'g', 'r', 'c', 'm']
#     print("Plotting data front matter...")
    
#     for i, df in enumerate(dfs):
#         print(f"Plotting data for {wind_speeds[i]} mph...")
#         time = df["time"]  
#         altitude = df['smoothed_altitude'] 
#         velocity = df['smoothed_velocity'] 
#         acceleration = df['smoothed_acceleration']
        
#         axs[0].plot(time, altitude, label=f'{wind_speeds[i]} mph', color=colors[i])
#         axs[1].plot(time, velocity, label=f'{wind_speeds[i]} mph', color=colors[i])
#         axs[2].plot(time, acceleration, label=f'{wind_speeds[i]} mph', color=colors[i])

#     # Add legend to each subplot
#     for ax in axs:
#         ax.legend(loc='upper right')

#     plt.tight_layout()

# def main(properties):
#     # DATA PREPPING
#     path_start = "/Users/andrewwehmeyer/Library/Mobile Documents/com~apple~CloudDocs/NUSTARS/Coding/Monte/data/"
#     df_0mph = pd.read_csv(path_start + "0mph.csv")
#     df_5mph = pd.read_csv(path_start + "5mph.csv")
#     df_10mph = pd.read_csv(path_start + "10mph.csv")
#     df_15mph = pd.read_csv(path_start + "15mph.csv")
#     df_20mph = pd.read_csv(path_start + "20mph.csv")
#     print("Loaded data...")

#     dfs = [df_0mph, df_5mph, df_10mph, df_15mph, df_20mph]
#     alpha = 1
#     clean_dfs = [None] * len(dfs)
#     for i in range(len(dfs)):
#         clean_dfs[i] = clean_data(dfs[i], alpha)
#     print("Cleaned data...")    

#     combined_plot(clean_dfs)
#     # gps_plot(clean_dfs[0])
#     # values_for_ahmand(clean_dfs[0])
#     print("Plotted data...")

#     save_figures = False
#     if save_figures:
#         for i in plt.get_fignums():
#             plt.figure(i).savefig(f'/Users/andrewwehmeyer/Library/Mobile Documents/com~apple~CloudDocs/NUSTARS/Coding/Monte/Figures/figure_{i}.png', dpi=300)
#         print("Saved figures...")

#     plt.show()
#     print("Done!")

project_root = Path(__file__).parent  # Gets the current directory where the script is located
file_path = project_root / "data/"
csv_list = [file for file in os.listdir(file_path) if file.endswith('.csv')]

plt.figure(figsize=(10, 6))

for current_csv in csv_list:
    print(file_path / current_csv)
    df = pd.read_csv(file_path / current_csv, skiprows=5)
    df_cleaned = clean_openrocket_data(df, 1)
    # plt.plot(df_cleaned['time'][0:400], df_cleaned['smoothed_altitude'][0:400], label=current_csv)
    # plot only the range where velocity > 0
    plt.plot(df_cleaned['time'][df_cleaned['smoothed_velocity'] > 0], df_cleaned['smoothed_altitude'][df_cleaned['smoothed_velocity'] > 0], label=current_csv)

plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Altitude [ft]')
plt.show()