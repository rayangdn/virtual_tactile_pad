import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def plot_tau(df, save_dir='plots'):

    # Create output directory if it doesn't exist
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Convert timestamp to relative time for better readability
    start_time = df['timestamp'].iloc[0]
    relative_time = (df['timestamp'] - start_time).to_numpy()

    # Create the plot
    plt.figure(figsize=(12, 6))

    # Plot each tau_ext with smaller markers
    for i in range(7):
        column = f'tau_ext_{i}'
        plt.plot(relative_time, df[column].to_numpy(), label=f'tau_ext_{i+1}', marker='o', markersize=3)

    # Customize the plot
    plt.title('Tau Ext Values Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Tau Ext Value')
    plt.grid(True)
    plt.legend()

    # Adjust layout to prevent label clipping
    plt.tight_layout()
  
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'tau_plot_{timestamp}.png'
    filepath = os.path.join(save_dir, filename)
    
    # Save the figure with high DPI
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved as: {filepath}")

    # Show the plot
    plt.show()

if __name__ == "__main__":
    # Read the CSV file
    df = pd.read_csv('data_pos_10.csv')
    
    # Create the tau plot
    plot_tau(df)