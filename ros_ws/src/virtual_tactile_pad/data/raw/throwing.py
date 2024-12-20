import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def identify_throws(df):
    throws = []
    current_throw = []
    
    # Iterate through the rows with filtering
    for index, row in df.iterrows():
        x, y = row['ft_x'], row['ft_y']
        
        # Check if point is within the specified ranges
        x_in_range = 0.0 <= x <= 0.12
        y_in_range = 0.0 <= y <= 0.14
        
        # If we hit zeros and we have points collected
        if x == 0 and y == 0:
            if current_throw:  # If we have points in current throw
                if len(current_throw) >= 10:  # Only add throws with 10 or more points
                    throws.append(np.array(current_throw))
                current_throw = []
        # If coordinates are within range, add to current throw
        elif x_in_range and y_in_range:
            current_throw.append([x, y])
    
    # Add the last throw if there are points remaining and it has enough points
    if current_throw and len(current_throw) >= 10:
        throws.append(np.array(current_throw))
    
    return throws

def plot_throws(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path)
    
    # Get throws
    throws = identify_throws(df)
    
    # Print information about throws
    print(f"Number of valid throws (with >= 10 points): {len(throws)}")
    for i, throw in enumerate(throws):
        print(f"Throw {i+1}: {len(throw)} points")
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Colors for different throws
    colors = ['red', 'green', 'blue', 'purple', 'orange', 'cyan', 'magenta', 'yellow']
    
    # Plot each throw with a different color
    for i, throw in enumerate(throws):
        color = colors[i % len(colors)]
        plt.scatter(throw[:, 0], throw[:, 1], 
                   label=f'Throw {i+1} ({len(throw)} pts)', 
                   color=color,
                   alpha=0.6)
    
    # Customize the plot
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Ball Contact Positions for Different Throws\n(Filtered: x=[0.0-0.12], y=[0.0-0.14], min 10 points)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    
    # Set axis limits to match the filtered ranges
    plt.xlim(-0.01, 0.13)
    plt.ylim(-0.01, 0.15)
    
    # Make the plot equal aspect ratio
    plt.axis('equal')
    
    # Show the plot
    plt.show()

# Usage
# Replace 'your_file.csv' with your actual file path
file_path = 'data_20241220_101805.csv'
plot_throws(file_path)