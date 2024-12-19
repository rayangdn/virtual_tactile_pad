import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_coordinates(csv_file):
    """
    Create a scatter plot of FT and OptiTrack coordinates and calculate Euclidean error in cm
    All measurements displayed in centimeters, filtered for specific coordinate ranges
    
    Parameters:
    csv_file (str): Path to the CSV file
    """
    # Read the CSV file
    df = pd.read_csv(csv_file)
    
    # Create initial mask for non-zero ft values
    mask = (df['ft_x'] != 0) | (df['ft_y'] != 0)
    
    # Add range constraints (in meters since original data is in meters)
    mask = mask & (df['ft_x'] > -0.02) & (df['ft_x'] < 0.14)
    mask = mask & (df['ft_y'] > -0.02) & (df['ft_y'] < 0.16)
    # mask = mask & (df['opti_x'] > -0.02) & (df['opti_x'] < 0.14)
    # mask = mask & (df['opti_y'] > -0.02) & (df['opti_y'] < 0.16)
   # mask = mask & (df['opti_z'] > -0.005) & (df['opti_z'] < 0.005)
    
    df_filtered = df[mask]
    
    # Convert coordinates to cm
    ft_x_cm = df_filtered['ft_x'] * 100
    ft_y_cm = df_filtered['ft_y'] * 100

    
    # # Calculate Euclidean distance between OptiTrack and FT coordinates (in cm)
    # euclidean_distances = np.sqrt(
    #     (opti_x_cm - ft_x_cm)**2 + 
    #     (opti_y_cm - ft_y_cm)**2
    # )
    
    # # Calculate error statistics
    # mean_error = np.mean(euclidean_distances)
    # std_error = np.std(euclidean_distances)
    # max_error = np.max(euclidean_distances)
    # min_error = np.min(euclidean_distances)
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Plot FT coordinates
    plt.scatter(ft_x_cm, ft_y_cm, 
               label='Force/Torque', color='blue', alpha=0.6, s=10)
    
    # Plot OptiTrack coordinates
    # plt.scatter(opti_x_cm, opti_y_cm, 
    #            label='OptiTrack', color='red', alpha=0.6, s=10)
    
    plt.xlabel('X Coordinate (cm)')
    plt.ylabel('Y Coordinate (cm)')
    plt.title('Force/Torque vs OptiTrack Coordinates')
    plt.grid(True)
    plt.legend()
    
    # Make axes equal to preserve shape
    plt.axis('equal')
    
    # # Add error statistics to the plot
    # error_text = (f'Error Statistics (cm):\n'
    #              f'Mean Error: {mean_error:.2f} cm\n'
    #              f'Std Dev: {std_error:.2f} cm\n'
    #              f'Max Error: {max_error:.2f} cm\n'
    #              f'Min Error: {min_error:.2f} cm')
    
    # plt.text(0.02, 0.98, error_text,
    #          transform=plt.gca().transAxes,
    #          verticalalignment='top',
    #          bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Show the plot
    plt.show()
    
    # Print error statistics
    # print("\nError Statistics (in centimeters):")
    # print(f"Mean Euclidean Error: {mean_error:.2f} cm")
    # print(f"Standard Deviation: {std_error:.2f} cm")
    # print(f"Maximum Error: {max_error:.2f} cm")
    # print(f"Minimum Error: {min_error:.2f} cm")
    # print(f"Number of points: {len(euclidean_distances)}")

# Example usage
if __name__ == "__main__":
    # RELATIVE TO BASE FRAME 
    plot_coordinates('data_20241219_152612.csv')