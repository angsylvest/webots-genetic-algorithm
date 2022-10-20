import pandas as pd
from pathlib import Path
import csv
import os

# desired columns
desired_columns = ['trial', 'objects retrieved', 'size', 'type']
desired_rows = []

# combines files into one csv that will generate summary data for collected obj
for file in os.listdir('./collection-data'):
    with open('./collection-data/' + file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try: 
                desired_rows.append({c: row[c] for c in desired_columns})
            except: 
                print('action no possible')


with open('compiled-collected.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=desired_columns)
    writer.writeheader()
    writer.writerows(desired_rows)


# desired columns
desired_columns = ['agent id', 'time since last block', 'size', 'collisions', 'type']
desired_rows = []

# combines files into one csv that will generate summary data for collisions
for file in os.listdir('./collision-data'):
    with open('./collision-data/' + file, 'r') as f: 
        reader = csv.DictReader(f)
        for row in reader:
            try: 
                print('curr row:' ,row, file)
                desired_rows.append({c: row[c] for c in desired_columns})
            except: 
                print('action no possible')


with open('compiled-collisions.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=desired_columns)
    writer.writeheader()
    writer.writerows(desired_rows)
