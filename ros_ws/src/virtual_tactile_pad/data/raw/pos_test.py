import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def plot_comparison(df, save_dir='plots'):
    # Create output directory if it doesn't exist
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    df_filtered = df.copy()
    
    # Filter conditions
    nonzero_ft = (df['ft_x'] != 0) | (df['ft_y'] != 0)
    nonzero_raw = (df['raw_contact_x'] != 0) | (df['raw_contact_y'] != 0)
    nonzero_filtered = (df['panda_x'] != 0) | (df['panda_y'] != 0)
    
    raw_range = (df['raw_contact_x'] <= 0.4) & (df['raw_contact_x'] >= -0.4) & \
                (df['raw_contact_y'] <= 0.4) & (df['raw_contact_y'] >= -0.4)
    filtered_range = (df['panda_x'] <= 0.4) & (df['panda_x'] >= -0.4) & \
                    (df['panda_y'] <= 0.4) & (df['panda_y'] >= -0.4)
    ft_range = (df['ft_x'] <= 0.2) & (df['ft_x'] >= -0.2) & \
               (df['ft_y'] <= 0.2) & (df['ft_y'] >= -0.2)
    
    # Apply filters
    df_filtered.drop(df[~(nonzero_ft & nonzero_raw & nonzero_filtered & 
                         raw_range & filtered_range & ft_range)].index, inplace=True)

    # Create figure
    fig = plt.figure(figsize=(15, 12))

    # First row: Scatter plots
    # Raw Contact vs FT
    ax1 = plt.subplot(321)
    ax1.scatter(df_filtered['ft_x'], df_filtered['ft_y'], 
                c='blue', label='FT Sensor', alpha=0.6, s=10)
    ax1.scatter(df_filtered['raw_contact_x'], df_filtered['raw_contact_y'], 
                c='red', label='Raw Contact', alpha=0.6, s=10)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Raw Contact vs FT Sensor')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')

    # Filtered Contact vs FT
    ax2 = plt.subplot(322)
    ax2.scatter(df_filtered['ft_x'], df_filtered['ft_y'], 
                c='blue', label='FT Sensor', alpha=0.6, s=10)
    ax2.scatter(df_filtered['panda_x'], df_filtered['panda_y'], 
                c='red', label='Filtered Contact', alpha=0.6, s=10)
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_title('Filtered Contact vs FT Sensor')
    ax2.grid(True)
    ax2.legend()
    ax2.axis('equal')

    # Second row: X Position histograms
    # Raw Contact X
    ax3 = plt.subplot(323)
    ax3.hist(df_filtered['ft_x'], bins=30, alpha=0.5, color='blue', label='FT X')
    ax3.hist(df_filtered['raw_contact_x'], bins=30, alpha=0.5, color='red', label='Raw Contact X')
    ax3.set_xlabel('X Position (m)')
    ax3.set_ylabel('Count')
    ax3.set_title('X Position Distribution (Raw vs FT)')
    ax3.legend()

    # Filtered Contact X
    ax4 = plt.subplot(324)
    ax4.hist(df_filtered['ft_x'], bins=30, alpha=0.5, color='blue', label='FT X')
    ax4.hist(df_filtered['panda_x'], bins=30, alpha=0.5, color='red', label='Filtered Contact X')
    ax4.set_xlabel('X Position (m)')
    ax4.set_ylabel('Count')
    ax4.set_title('X Position Distribution (Filtered vs FT)')
    ax4.legend()

    # Third row: Y Position histograms
    # Raw Contact Y
    ax5 = plt.subplot(325)
    ax5.hist(df_filtered['ft_y'], bins=30, alpha=0.5, color='blue', label='FT Y')
    ax5.hist(df_filtered['raw_contact_y'], bins=30, alpha=0.5, color='red', label='Raw Contact Y')
    ax5.set_xlabel('Y Position (m)')
    ax5.set_ylabel('Count')
    ax5.set_title('Y Position Distribution (Raw vs FT)')
    ax5.legend()

    # Filtered Contact Y
    ax6 = plt.subplot(326)
    ax6.hist(df_filtered['ft_y'], bins=30, alpha=0.5, color='blue', label='FT Y')
    ax6.hist(df_filtered['panda_y'], bins=30, alpha=0.5, color='red', label='Filtered Contact Y')
    ax6.set_xlabel('Y Position (m)')
    ax6.set_ylabel('Count')
    ax6.set_title('Y Position Distribution (Filtered vs FT)')
    ax6.legend()

    plt.tight_layout()

    # Save plot
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'pos_plot_{timestamp}.png'
    filepath = os.path.join(save_dir, filename)
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved as: {filepath}")

    plt.show()

if __name__ == "__main__":
    # Read the CSV file
    df = pd.read_csv('data_20241216_154337.csv')  
    # Create comparison plot
    plot_comparison(df)