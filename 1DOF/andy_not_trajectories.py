# import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
# import os
# from pathlib import Path

# # SOMETHING IS VERY BROKEN IN MY CODE
# # I WILL WORK ON IT

# project_root = Path(__file__).parent  # Gets the current directory where the script is located
# file_path = project_root / "../openrocket_data/"
# csv_list = [file for file in os.listdir(file_path) if file.endswith('.csv')]

# # import the function ode from 1dof_sim.py
# from onedof_sim import ode_solver

# ic = type('ics', (object,), {'t_0': 2.8, 'v_0': 656.6, 'h_0': 1000})
# properties = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 0})
# ode_solver(ic, properties, dt=0.01)

# def simulate(csv_file, N):
#     df = pd.read_csv(csv_file, skiprows=5)
#     total_rows = len(df)
    
#     # Calculate step size based on N
#     step_size = max(total_rows // N, 1)  # Ensure step size is at least 1
    
#     all_tracjectories = []
    
#     # Iterate through evenly spaced rows
#     for index in range(0, total_rows, step_size):
#         row = df.iloc[index]
#         ic = type('ics', (object,), {'t_0': row['time'], 'v_0': row['velocity'], 'h_0': row['altitude']})
#         properties = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 6/144})
#         states = ode_solver(ic, properties, dt=0.01)
#         all_tracjectories.append(states)
        
#     return all_tracjectories

# N = 100  
# all_tracjectories = simulate(file_path, N)

# # Plot results
# plt.figure()
# # plt.plot(ic_times_mach, apogees_mach, label=f'ADS Area = {round(A_ads_mach * 144, 2)} in²')
# plt.title("Apogee vs Actuation Time")
# plt.xlabel("T+X ADS Starts Actuating [s]")
# plt.ylabel("Apogee [ft]")
# plt.grid(True)
# plt.legend()
# plt.show()