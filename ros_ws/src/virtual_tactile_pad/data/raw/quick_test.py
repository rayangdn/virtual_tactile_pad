import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def plot_comparison(df1, df2, title1='Dataset 1', title2='Dataset 2', save_dir='plots'):
    # Create output directory if it doesn't exist
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    # Filter out rows where both x and y are 0 for each system in both datasets
    df1_filtered = df1.copy()
    df2_filtered = df2.copy()
    for df in [df1_filtered, df2_filtered]:
        nonzero_ft = (df['ft_x'] != 0) | (df['ft_y'] != 0)
        nonzero_panda = (df['panda_x'] != 0) | (df['panda_y'] != 0)
        panda_range = (df['panda_x'] <= 0.4) & (df['panda_x'] >= -0.4) & (df['panda_y'] <= 0.4) & (df['panda_y'] >= -0.4)
        ft_range = (df['ft_x'] <= 0.2) & (df['ft_x'] >= -0.2) & (df['ft_y'] <= 0.2) & (df['ft_y'] >= -0.2)
        
        # Apply all filters at once
        df.drop(df[~(nonzero_ft & nonzero_panda & panda_range & ft_range)].index, inplace=True)

    # Create a figure with multiple subplots
    fig = plt.figure(figsize=(15, 12))

    # 1. Scatter plots (top row)
    ax1 = plt.subplot(321)
    ax1.scatter(df1_filtered['ft_x'], df1_filtered['ft_y'], c='blue', label='FT', alpha=0.6, s=10)
    ax1.scatter(df1_filtered['panda_x'], df1_filtered['panda_y'], c='red', label='Panda', alpha=0.6, s=10)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title(f'Position Comparison - {title1}')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')

    ax2 = plt.subplot(322)
    ax2.scatter(df2_filtered['ft_x'], df2_filtered['ft_y'], c='blue', label='FT', alpha=0.6, s=10)
    ax2.scatter(df2_filtered['panda_x'], df2_filtered['panda_y'], c='red', label='Panda', alpha=0.6, s=10)
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_title(f'Position Comparison - {title2}')
    ax2.grid(True)
    ax2.legend()
    ax2.axis('equal')

    # 2. X-axis histograms (middle row)
    ax3 = plt.subplot(323)
    ax3.hist(df1_filtered['ft_x'], bins=30, alpha=0.5, color='blue', label='FT X')
    ax3.hist(df1_filtered['panda_x'], bins=30, alpha=0.5, color='red', label='Panda X')
    ax3.set_xlabel('X Position (m)')
    ax3.set_ylabel('Count')
    ax3.set_title(f'X Position Distribution - {title1}')
    ax3.legend()

    ax4 = plt.subplot(324)
    ax4.hist(df2_filtered['ft_x'], bins=30, alpha=0.5, color='blue', label='FT X')
    ax4.hist(df2_filtered['panda_x'], bins=30, alpha=0.5, color='red', label='Panda X')
    ax4.set_xlabel('X Position (m)')
    ax4.set_ylabel('Count')
    ax4.set_title(f'X Position Distribution - {title2}')
    ax4.legend()

    # 3. Y-axis histograms (bottom row)
    ax5 = plt.subplot(325)
    ax5.hist(df1_filtered['ft_y'], bins=30, alpha=0.5, color='blue', label='FT Y')
    ax5.hist(df1_filtered['panda_y'], bins=30, alpha=0.5, color='red', label='Panda Y')
    ax5.set_xlabel('Y Position (m)')
    ax5.set_ylabel('Count')
    ax5.set_title(f'Y Position Distribution - {title1}')
    ax5.legend()

    ax6 = plt.subplot(326)
    ax6.hist(df2_filtered['ft_y'], bins=30, alpha=0.5, color='blue', label='FT Y')
    ax6.hist(df2_filtered['panda_y'], bins=30, alpha=0.5, color='red', label='Panda Y')
    ax6.set_xlabel('Y Position (m)')
    ax6.set_ylabel('Count')
    ax6.set_title(f'Y Position Distribution - {title2}')
    ax6.legend()

    plt.tight_layout()

    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'comparison_plot_{timestamp}.png'
    filepath = os.path.join(save_dir, filename)
    
    # Save the figure with high DPI
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved as: {filepath}")

    # Print detailed statistics for both datasets
    for i, (df, title) in enumerate([(df1_filtered, title1), (df2_filtered, title2)]):
        print(f"\nDetailed Position Statistics for {title}:")
        print("\nX axis:")
        print(f"FT X:")
        print(f"  Mean: {df['ft_x'].mean():.3f}")
        print(f"  Std:  {df['ft_x'].std():.3f}")
        print(f"  Min:  {df['ft_x'].min():.3f}")
        print(f"  Max:  {df['ft_x'].max():.3f}")
        print(f"  IQR:  {df['ft_x'].quantile(0.75) - df['ft_x'].quantile(0.25):.3f}")

        print(f"\nPanda X:")
        print(f"  Mean: {df['panda_x'].mean():.3f}")
        print(f"  Std:  {df['panda_x'].std():.3f}")
        print(f"  Min:  {df['panda_x'].min():.3f}")
        print(f"  Max:  {df['panda_x'].max():.3f}")
        print(f"  IQR:  {df['panda_x'].quantile(0.75) - df['panda_x'].quantile(0.25):.3f}")

        print("\nY axis:")
        print(f"FT Y:")
        print(f"  Mean: {df['ft_y'].mean():.3f}")
        print(f"  Std:  {df['ft_y'].std():.3f}")
        print(f"  Min:  {df['ft_y'].min():.3f}")
        print(f"  Max:  {df['ft_y'].max():.3f}")
        print(f"  IQR:  {df['ft_y'].quantile(0.75) - df['ft_y'].quantile(0.25):.3f}")

        print(f"\nPanda Y:")
        print(f"  Mean: {df['panda_y'].mean():.3f}")
        print(f"  Std:  {df['panda_y'].std():.3f}")
        print(f"  Min:  {df['panda_y'].min():.3f}")
        print(f"  Max:  {df['panda_y'].max():.3f}")
        print(f"  IQR:  {df['panda_y'].quantile(0.75) - df['panda_y'].quantile(0.25):.3f}")

        # Print number of data points
        print(f"\nNumber of data points: {len(df)}")

    plt.show()

# Example usage:
# Read the CSV files
df1 = pd.read_csv('traj_raw.csv')  # First dataset
df2 = pd.read_csv('traj_pf.csv')   # Second dataset

# Create the comparison plots
plot_comparison(df1, df2, 'First Dataset', 'Second Dataset')