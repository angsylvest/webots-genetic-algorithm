import os
import pandas as pd
import matplotlib.pyplot as plt

# Replace 'folder_path' with the path to your folder containing the CSV files
folder_path = '/path/to/your/folder'

# Initialize an empty DataFrame to store data from CSV files
data = pd.DataFrame()

# Iterate over each CSV file in the folder
for file in os.listdir(folder_path):
    if file.endswith('.csv'):
        file_path = os.path.join(folder_path, file)
        df = pd.read_csv(file_path)
        df['file_name'] = os.path.splitext(file)[0]  # Add file name as a column
        data = data.append(df, ignore_index=True)

# List of variables for plotting and summary statistics
variables_to_plot = ['straight', 'alternating-left', 'alternating-right', 'true random', 'time since last block',
                     'size', 'fitness', 'collected']
summary_variables = ['size', 'fitness', 'collected']

# Group data by 'trial' for plotting and summary statistics
grouped_data = data.groupby(['trial'])

# Create plots and summary statistics for each trial
for trial, df in grouped_data:
    plt.figure(figsize=(10, 6))
    for variable in variables_to_plot:
        plt.plot(df['time step'], df[variable], label=variable)
    plt.xlabel('Time Step')
    plt.ylabel('Value')
    plt.title(f'Line Plot for Trial {trial}')
    plt.legend()
    plt.savefig(f'trial_{trial}_plot.png')  # Save plot as PNG
    plt.close()

    summary_df = df[summary_variables].describe()
    summary_df.to_csv(f'trial_{trial}_summary_statistics.csv')  # Save summary statistics as CSV
