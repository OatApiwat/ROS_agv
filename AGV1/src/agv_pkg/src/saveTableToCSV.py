import mysql.connector
import csv

# Establish the database connection
connection = mysql.connector.connect(
    host='localhost',
    user='agv_mysql',
    password='123qwe',
    database='AGV_db'
)

# Create a cursor object
cursor = connection.cursor()

# Define the query to fetch all data from the table
query = "SELECT * FROM record_tb"

# Execute the query
cursor.execute(query)

# Fetch all rows from the executed query
rows = cursor.fetchall()

# Get column names from the cursor description
column_names = [desc[0] for desc in cursor.description]

# Define the CSV file path
csv_file_path = '/home/ros3/ros1_ws/src/agv_pkg/src/distanceData8-9__07__2024.csv'

# Write data to CSV file
with open(csv_file_path, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    
    # Write the column names as the first row
    csvwriter.writerow(column_names)
    
    # Write the data rows
    csvwriter.writerows(rows)

# Close the cursor and connection
cursor.close()
connection.close()

print(f"Data has been successfully written to {csv_file_path}")
