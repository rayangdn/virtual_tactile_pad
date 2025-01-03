import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def identify_throws(df):
    throws = []
    current_throw = []
    current_times = []
    current_wrench = []
    start_time = None
    
    # Iterate through the rows with filtering
    for index, row in df.iterrows():
        x, y = row['ft_x'], row['ft_y']
        wrench_z = row['ft_wrench_force_z']
        timestamp = row['timestamp']
        
        # Check if point is within the specified ranges
        x_in_range = 0.0 <= x <= 0.12
        y_in_range = 0.0 <= y <= 0.14
        
        # If we hit zeros and we have points collected
        if x == 0 and y == 0:
            if current_throw and len(current_throw) >= 10:  # Only add throws with 10 or more points
                throws.append({
                    'positions': np.array(current_throw),
                    'times': np.array(current_times),
                    'wrench_z': np.array(current_wrench)
                })
            current_throw = []
            current_times = []
            current_wrench = []
            start_time = None
        # If coordinates are within range, add to current throw
        elif x_in_range and y_in_range:
            if start_time is None:
                start_time = timestamp
            current_throw.append([x, y])
            current_times.append(timestamp - start_time)  # Time relative to throw start
            current_wrench.append(wrench_z)
    
    # Add the last throw if there are points remaining and it has enough points
    if current_throw and len(current_throw) >= 10:
        throws.append({
            'positions': np.array(current_throw),
            'times': np.array(current_times),
            'wrench_z': np.array(current_wrench)
        })
    
    return throws

def plot_wrench_z(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path)
    
    # Get throws
    throws = identify_throws(df)
    
    # Create the plot
    plt.figure(figsize=(12, 6))
    
    # Colors for different throws
    colors = ['red', 'green', 'blue', 'purple', 'orange', 'cyan', 'magenta', 'yellow']
    
    # Plot each throw
    for i, throw in enumerate(throws):
        color = colors[i % len(colors)]
        times = throw['times']
        wrench_z = throw['wrench_z']
        
        plt.plot(times, wrench_z, 
                label=f'Throw {i+1} ({len(times)} pts)',
                color=color, marker='o', markersize=4)
    
    # Customize the plot
    plt.xlabel('Time (s)')
    plt.ylabel('Wrench Force Z')
    plt.title('Wrench Force Z over Time for Each Throw')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    
    # Adjust layout
    plt.tight_layout()
    
    # Show the plot
    plt.show()

# Usage
# Replace 'your_file.csv' with your actual file path
file_path = 'data_throwing_panda_ft.csv'
plot_wrench_z(file_path)