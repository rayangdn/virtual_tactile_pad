import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def plot_wrench_and_error(df, save_dir='plots'):
    # Create output directory if it doesn't exist
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Convert timestamp to relative time for better readability
    start_time = df['timestamp'].iloc[0]
    relative_time = (df['timestamp'] - start_time).to_numpy()

    # Create subplots: 2 rows (wrench and error) x 6 plots (3 force, 3 torque)
    fig, axes = plt.subplots(4, 3, figsize=(15, 16))
    fig.suptitle('Wrench Measurements and Errors', fontsize=14)

    # Plot each axis
    for idx, axis in enumerate(['x', 'y', 'z']):
        # Plot forces
        axes[0, idx].plot(relative_time, df[f'ft_wrench_force_{axis}'].to_numpy(), 
                         label='FT Sensor', marker='o', markersize=3)
        axes[0, idx].plot(relative_time, df[f'panda_wrench_force_{axis}'].to_numpy(), 
                         label='Panda', marker='o', markersize=3, linestyle='--')
        axes[0, idx].set_title(f'Force {axis.upper()}-axis')
        axes[0, idx].set_xlabel('Time (seconds)')
        axes[0, idx].set_ylabel('Force')
        axes[0, idx].grid(True)
        axes[0, idx].legend()

        # Plot torques
        axes[1, idx].plot(relative_time, df[f'ft_wrench_torque_{axis}'].to_numpy(), 
                         label='FT Sensor', marker='o', markersize=3)
        axes[1, idx].plot(relative_time, df[f'panda_wrench_torque_{axis}'].to_numpy(), 
                         label='Panda', marker='o', markersize=3, linestyle='--')
        axes[1, idx].set_title(f'Torque {axis.upper()}-axis')
        axes[1, idx].set_xlabel('Time (seconds)')
        axes[1, idx].set_ylabel('Torque')
        axes[1, idx].grid(True)
        axes[1, idx].legend()

        # Calculate and plot force errors
        force_error = np.abs(df[f'panda_wrench_force_{axis}'] - df[f'ft_wrench_force_{axis}'])
        axes[2, idx].plot(relative_time, force_error.to_numpy(), 
                         label=f'Force {axis}-axis', marker='o', markersize=3)
        axes[2, idx].set_title(f'Force Error {axis.upper()}-axis')
        axes[2, idx].set_xlabel('Time (seconds)')
        axes[2, idx].set_ylabel('Absolute Error (Force)')
        axes[2, idx].grid(True)
        mean_force_error = force_error.mean()
        axes[2, idx].legend([f'Mean Error: {mean_force_error:.6f}'])

        # Calculate and plot torque errors
        torque_error = np.abs(df[f'panda_wrench_torque_{axis}'] - df[f'ft_wrench_torque_{axis}'])
        axes[3, idx].plot(relative_time, torque_error.to_numpy(), 
                         label=f'Torque {axis}-axis', marker='o', markersize=3)
        axes[3, idx].set_title(f'Torque Error {axis.upper()}-axis')
        axes[3, idx].set_xlabel('Time (seconds)')
        axes[3, idx].set_ylabel('Absolute Error (Torque)')
        axes[3, idx].grid(True)
        mean_torque_error = torque_error.mean()
        axes[3, idx].legend([f'Mean Error: {mean_torque_error:.6f}'])

    # Adjust layout to prevent label clipping
    plt.tight_layout()
  
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'wrench_plot_{timestamp}.png'
    filepath = os.path.join(save_dir, filename)
    
    # Save the figure with high DPI
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved as: {filepath}")

    # Show the plot
    plt.show()
    
    # Close the figure to free memory
    plt.close()

# Example usage:
if __name__ == "__main__":
    # Read the CSV file
    df = pd.read_csv('data_pos_10.csv')
    
    # Create the combined plots
    plot_wrench_and_error(df)