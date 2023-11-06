import os
import pandas as pd
import matplotlib.pyplot as plt

# # Replace 'folder_path' with the path to your folder containing the CSV files
folder_path = '/Users/angelsylvester/Downloads/graph-generation/collection-data/'

# Initialize an empty DataFrame to store data from CSV files
data = pd.DataFrame()

# Iterate over each CSV file in the folder
for file in os.listdir(folder_path):
    if file.endswith('.csv'):
        file_path = os.path.join(folder_path, file)
        df = pd.read_csv(file_path)
        data = data.append(df, ignore_index=True)

# # Replace 'x_variable' and 'y_variable' with the column names of your shared x and y variables
# x_variable = 'x_column_name'
# y_variable = 'y_column_name'

# # Create a line plot
# plt.figure(figsize=(10, 6))
# for label, df in data.groupby('file_name_column'):
#     plt.plot(df[x_variable], df[y_variable], label=label)

# plt.xlabel('X Axis Label')
# plt.ylabel('Y Axis Label')
# plt.title('Line Plot for Shared X and Y Variables')
# plt.legend()
# plt.show()

# List all the files in the directory
file_list = os.listdir("/Users/angelsylvester/Downloads/graph-generation/collection-data")

# Specify the column for which you want the summary statistics
column_of_interest = "objects retrieved"

# Loop through each file and extract the summary statistics
for file in file_list:
    if file.endswith(".csv"):
        file_path = os.path.join(folder_path, file)
        df = pd.read_csv(file_path)
        grouped_summary = df.groupby("size")[column_of_interest].describe()
        print(f"Summary statistics for {file}:")
        print(grouped_summary)
        print("\n")