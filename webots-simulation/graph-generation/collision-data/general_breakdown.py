import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

generate_trial_summaries = False 
generate_position_summaries = False 
# Replace 'folder_path' with the path to your folder containing the CSV files
folder_path = '/Users/angelsylvester/Downloads/graph-generation/collision-data'

# Initialize an empty Da
# taFrame to store data from CSV files
data = pd.DataFrame()
# Iterate over each CSV file in the folder
for file in os.listdir(folder_path):
    if file.endswith('.csv'):
        file_path = os.path.join(folder_path, file)
        df = pd.read_csv(file_path)
        df['SourceFile'] = os.path.splitext(file)[0]  # Add file name as a column
        data = data.append(df, ignore_index=True)

# List of variables for plotting and summary statistics
variables_to_plot = ['fitness', 'collected', ] #['straight', 'alternating-left', 'alternating-right', 'true random', 'time since last block',
                     
summary_variables = ['fitness', 'collected']

if generate_trial_summaries: 
    # Group data by 'trial' for plotting and summary statistics
    grouped_data = data.groupby(['trial', 'size', 'SourceFile'])

    # Create an empty DataFrame to store the combined summary statistics for all trials
    combined_summary_df = pd.DataFrame()

    # Create plots for each variable with different colors for each agent id
    for (trial, size, SourceFile), df in grouped_data:
        identifier = f'Trial_{trial}_Size_{size}_SourceFile_{SourceFile}'
        summary_df = pd.DataFrame()

        for variable in variables_to_plot:
            plt.figure(figsize=(10, 6))
            for agent_id, agent_data in df.groupby('agent id'):
                plt.plot(agent_data['time step'], agent_data[variable], label=f'Agent {agent_id}')
            plt.xlabel('Time Step')
            plt.ylabel('Value')
            plt.title(f'Line Plot for {variable} - {identifier} (DataFrame: df)')
            plt.legend()
            plt.savefig(f'{identifier}_{variable}_plot.png')  # Save plot as PNG
            plt.close()

        for variable in summary_variables:
            plt.figure(figsize=(10, 6))
            for agent_id, agent_data in df.groupby('agent id'):
                plt.plot(agent_data.index, agent_data[variable], label=f'Agent {agent_id}')
            plt.xlabel('Time Step')
            plt.ylabel('Value')
            plt.title(f'Summary Plot for {variable} - {identifier}')
            plt.legend()
            plt.savefig(f'{identifier}_{variable}_summary_plot.png')  # Save plot as PNG
            plt.close()

            # Append the summary statistics for each trial to the combined_summary_df
            summary_df = pd.concat([summary_df, df[summary_variables].describe()], axis=1)

        combined_summary_df = pd.concat([combined_summary_df, summary_df], axis=1)

    # Save the combined summary statistics for all trials to a CSV file
    combined_summary_df.to_csv('combined_summary_statistics.csv')


if generate_position_summaries: 
    # Group data by 'trial' for plotting and summary statistics
    grouped_data = data.groupby(['trial', 'size', 'SourceFile','agent id' ])

    # Define a function to calculate the distance traveled
    def calculate_distance_traveled(group):
        x_diff = group['pos x'].diff()
        y_diff = group['pos y'].diff()
        group['distance_traveled'] = (x_diff ** 2 + y_diff ** 2) ** 0.5
        group['sum_distance_traveled'] = group['distance_traveled'].sum()
        return group

    # Apply the function to each group in grouped_data
    grouped_data = grouped_data.apply(calculate_distance_traveled)

    # Calculate relevant statistics broken down by trial, size, and agent id
    summary_statistics = grouped_data.groupby(['size', 'SourceFile', 'agent id'])['distance_traveled'].agg(['sum', 'mean'])

    # Print relevant statistics broken down by trial, size, and agent id
    # print("Summary Statistics:")
    # print(summary_statistics)

    # # Calculate coverage based on the dimensions of the shared space for each trial, size, and SourceFile
    # shared_space_width = 10  # Replace with the actual width of the shared space
    # shared_space_height = 10  # Replace with the actual height of the shared space

    # grouped_data['x_coverage'] = grouped_data['pos x'].transform(lambda x: x.max() - x.min())
    # grouped_data['y_coverage'] = grouped_data['pos y'].transform(lambda y: y.max() - y.min())

    # grouped_data['total_coverage'] = (grouped_data['x_coverage'] * grouped_data['y_coverage']) / (
    #         shared_space_width * shared_space_height) * 100

    # # Print coverage statistics broken down by trial, size, and SourceFile
    # print("Coverage Statistics:")
    # print(grouped_data['total_coverage'].mean())

    # Visualize the coordinates data for each trial, size, and SourceFile
    # Assuming df is your DataFrame
    grouped_data = df.groupby('trial')

    # Iterate over each group and create plots
    for trial, group in grouped_data:
        plt.figure(figsize=(8, 6))
        for agent_id, agent_data in group.groupby('agent id'):
            plt.plot(agent_data['pos x'], agent_data['pos y'], marker='o', linestyle='-', label=f'Agent {agent_id}')
        plt.xlabel('X-coordinate')
        plt.ylabel('Y-coordinate')
        plt.title(f'Positions for Trial {trial}')
        plt.legend()
        plt.show()