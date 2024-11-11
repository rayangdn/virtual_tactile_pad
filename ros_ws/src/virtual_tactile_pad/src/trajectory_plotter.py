#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import ast

def prepare_data(df):
    """
    Prepare data by removing duplicates and ensuring unique timestamps
    """
    # Calculate relative time
    df['time_relative'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # Add tiny increments to duplicate timestamps
    df['unique_time'] = df['timestamp'].copy()
    mask = df['timestamp'].duplicated()
    if mask.any():
        # Add small increments (1e-6) to duplicate timestamps
        duplicates = df[mask].index
        for i, idx in enumerate(duplicates):
            df.loc[idx, 'unique_time'] += (i + 1) * 1e-6
    
    return df

def create_trajectory(df, num_interpolation_points=100):
    """
    Create interpolated trajectory from data points using numpy.interp
    """
    # Get timestamps for interpolation
    times = df['unique_time'].values
    x = df['contact_x'].values
    y = df['contact_y'].values
    
    # Create new time points for smooth interpolation
    t_new = np.linspace(times.min(), times.max(), num_interpolation_points)
    
    # Generate interpolated points using numpy.interp
    x_interp = np.interp(t_new, times, x)
    y_interp = np.interp(t_new, times, y)
    
    return x_interp, y_interp

def plot_data(csv_path):
    """
    Read and plot the data
    """
    try:
        # Read CSV file
        df = pd.read_csv(csv_path)
        
        if len(df) < 2:  # Need at least 2 points for interpolation
            return
            
        # Prepare data
        df = prepare_data(df)

        grid_data = df['grid'].iloc[-1]
        prediction = df['prediction'].iloc[-1]
        confidence = df['confidence'].iloc[-1]
        
        # Convert string representation back to numpy array
        grid = np.array(ast.literal_eval(grid_data))
            
        # Create interpolated trajectory
        x_interp, y_interp = create_trajectory(df)
        
        # Clear previous plot
        plt.clf()

        plt.subplot(1, 3, 1)
        # Plot points and line
        plt.fill([0, 0.12, 0.12, 0], [0, 0, 0.14, 0.14], color='blue', alpha=0.2)
        plt.scatter(df['contact_x'], df['contact_y'], label='Original Points')
        plt.plot(x_interp, y_interp, 'r-', alpha=0.5, label='Interpolated')

        # Add labels and grid
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Trajectory')
        plt.grid(False)
        plt.axis('equal')
        plt.legend()
        
        plt.subplot(1, 3, 2)
        # Plot grid
        plt.imshow(grid, cmap='gray')
        plt.title('Grid')
        plt.axis('on')
        plt.grid(True, alpha=0.3)

        plt.subplot(1, 3, 3)
        # Prediction results
        prediction_text = [
            f'Predicted Digit: {prediction}',
            f'Confidence: {confidence:.2%}',
        ]    
        plt.text(0.1, 0.6, '\n'.join(prediction_text), fontsize=12)
        plt.axis('off')
        plt.title('Recognition Results')
 
        # Update display
        plt.draw()
        plt.pause(0.01)
        
    except Exception as e:
        print(f"Error plotting data: {e}")

def main():
    # Get the CSV file path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(os.path.dirname(current_dir), 'data', 'raw')
    csv_path = os.path.join(data_dir, "trajectory_data.csv")
    
    print(f"Monitoring {csv_path} for changes...")
    
    # Initialize plot
    plt.ion()  # Enable interactive mode
    plt.figure(figsize=(12, 6))
    
    last_modified = 0
    
    try:
        while True:
            # Check if file has been modified
            try:
                current_modified = os.path.getmtime(csv_path)
                if current_modified > last_modified:
                    plot_data(csv_path)
                    last_modified = current_modified
            except FileNotFoundError:
                pass
            
            plt.pause(0.1)  # Small delay to prevent high CPU usage
            
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    finally:
        plt.close('all')

if __name__ == "__main__":
    main()