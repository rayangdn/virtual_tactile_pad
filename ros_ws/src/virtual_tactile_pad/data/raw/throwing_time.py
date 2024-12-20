import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def identify_throws(df):
    throws = []
    current_throw = []
    current_times = []
    current_panda_x = []
    current_panda_y = []
    start_time = None
    
    # Iterate through the rows with filtering
    for index, row in df.iterrows():
        x, y = row['ft_x'], row['ft_y']
        panda_x = row['panda_x']  # Add panda positions
        panda_y = row['panda_y']
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
                    'panda_x': np.array(current_panda_x),
                    'panda_y': np.array(current_panda_y)
                })
            current_throw = []
            current_times = []
            current_panda_x = []
            current_panda_y = []
            start_time = None
        # If coordinates are within range, add to current throw
        elif x_in_range and y_in_range:
            if start_time is None:
                start_time = timestamp
            current_throw.append([x, y])
            current_times.append(timestamp - start_time)  # Time relative to throw start
            current_panda_x.append(panda_x)
            current_panda_y.append(panda_y)
    
    # Add the last throw if there are points remaining and it has enough points
    if current_throw and len(current_throw) >= 10:
        throws.append({
            'positions': np.array(current_throw),
            'times': np.array(current_times),
            'panda_x': np.array(current_panda_x),
            'panda_y': np.array(current_panda_y)
        })
    
    return throws

def plot_time_series(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path)
    
    # Get throws
    throws = identify_throws(df)
    
    # Create four subplots
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 16), sharex=True)
    
    # Colors for different throws
    colors = ['red', 'green', 'blue', 'purple', 'orange', 'cyan', 'magenta', 'yellow']
    
    # Plot each throw
    for i, throw in enumerate(throws):
        color = colors[i % len(colors)]
        positions = throw['positions']
        times = throw['times']
        panda_x = throw['panda_x']
        panda_y = throw['panda_y']
        
        # Plot contact X position over time
        ax1.plot(times, positions[:, 0], 
                label=f'Throw {i+1} ({len(positions)} pts)',
                color=color, marker='o', markersize=4)
        
        # Plot contact Y position over time
        ax2.plot(times, positions[:, 1],
                label=f'Throw {i+1} ({len(positions)} pts)',
                color=color, marker='o', markersize=4)
                
        # Plot Panda X position over time
        ax3.plot(times, panda_x,
                label=f'Throw {i+1}',
                color=color, marker='o', markersize=4)
                
        # Plot Panda Y position over time
        ax4.plot(times, panda_y,
                label=f'Throw {i+1}',
                color=color, marker='o', markersize=4)
    
    # Customize the plots
    ax1.set_ylabel('Contact X Position')
    ax1.set_title('Contact X Position over Time')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend()
    
    ax2.set_ylabel('Contact Y Position')
    ax2.set_title('Contact Y Position over Time')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend()
    
    ax3.set_ylabel('Panda X Position')
    ax3.set_title('Panda X Position over Time')
    ax3.grid(True, linestyle='--', alpha=0.7)
    ax3.legend()
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Panda Y Position')
    ax4.set_title('Panda Y Position over Time')
    ax4.grid(True, linestyle='--', alpha=0.7)
    ax4.legend()
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Show the plot
    plt.show()

# Usage
file_path = 'data_throwing_panda_ft.csv'
plot_time_series(file_path)