#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import ast
from matplotlib.image import imread

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
        
        prediction = df['prediction'].iloc[-1]
        confidence = df['confidence'].iloc[-1]
            
        # Create interpolated trajectory
        x_interp, y_interp = create_trajectory(df)
        
        # Clear previous plot
        plt.clf()

        # First subplot (top) - Trajectory
        plt.subplot(2, 1, 1)
        
        # Add blue background
        plt.fill([0, 0.12, 0.12, 0], [0, 0, 0.14, 0.14], color='blue', alpha=0.2)
        
        plt.scatter(df['contact_x'], df['contact_y'], label='Original Points')
        plt.plot(x_interp, y_interp, 'r-', alpha=0.5, label='Interpolated')

        plt.title('Trajectory', fontsize=20)
        plt.axis('equal')  # Keep equal aspect ratio
        plt.axis('off')    # Hide axes
        plt.legend()

        # Second subplot (bottom) - Digit Image or Text
        plt.subplot(2, 1, 2)
        plt.title('Recognition Results', fontsize=20)
        plt.axis('off')
        
        # Check confidence threshold
        if confidence < 0.5:  # 50% threshold
            plt.text(0.5, 0.5, 'No digit recognized', ha='center', va='center', fontsize=20)
        else:
            try:
                # Construct image path based on predicted digit
                current_dir = os.path.dirname(os.path.abspath(__file__))
                img_path = os.path.join(os.path.dirname(current_dir), 'scripts', 'digits_img', f'{int(prediction)}.jpeg')
                
                if os.path.exists(img_path):
                    img = imread(img_path)
                    plt.imshow(img)
                else:
                    plt.text(0.5, 0.5, 'Image not found', ha='center', va='center', fontsize=16)
                    
            except Exception as e:
                plt.text(0.5, 0.5, f'Error loading image: {str(e)}', ha='center', va='center', fontsize=16)
        
        # Adjust layout to prevent overlap
        plt.tight_layout()
        
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
    plt.figure(figsize=(8, 10))
    
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