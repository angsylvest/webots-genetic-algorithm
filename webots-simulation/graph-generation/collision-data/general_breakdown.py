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


# Example coordinates data for a specific trial
coordinates_data = {
    'x': [1, 2, 3, 4, 5],
    'y': [2, 4, 5, 4, 6]
}

# Convert the dictionary to a DataFrame
df = pd.DataFrame(coordinates_data)

# Calculate relevant statistics
distance_traveled = df.apply(lambda row: ((row['x'].diff()**2 + row['y'].diff()**2)**0.5).sum(), axis=1)
total_distance = distance_traveled.sum()
average_distance_per_step = distance_traveled.mean()

# Visualize the coordinates data
plt.figure(figsize=(8, 6))
plt.plot(df['x'], df['y'], marker='o')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.title('Foraging Performance - Trial X')
plt.show()

# Print relevant statistics
print(f"Total Distance Traveled: {total_distance}")
print(f"Average Distance Traveled per Step: {average_distance_per_step}")


# Calculate coverage based on the dimensions of the shared space
shared_space_width = 10  # Replace with the actual width of the shared space
shared_space_height = 10  # Replace with the actual height of the shared space

x_coverage = df['x'].max() - df['x'].min()
y_coverage = df['y'].max() - df['y'].min()

total_coverage = (x_coverage * y_coverage) / (shared_space_width * shared_space_height) * 100

# Visualize the coordinates data
plt.figure(figsize=(8, 6))
plt.plot(df['x'], df['y'], marker='o')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.title('Foraging Performance - Trial X')
plt.show()

# Print coverage statistics
print(f"Total Coverage: {total_coverage}%")