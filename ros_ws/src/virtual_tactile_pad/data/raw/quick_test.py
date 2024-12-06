import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('data_20241206_091048.csv')

# Filter out rows where both x and y are 0 for each system
df = df[(df['ft_x'] != 0) | (df['ft_y'] != 0)]


# Create figure
plt.figure(figsize=(10, 8))

# Plot X-Y points
plt.scatter(df['ft_x'], df['ft_y'], c='blue', label='FT', alpha=0.6, s=10)
plt.scatter(df['panda_x'], df['panda_y'], c='red', label='Panda', alpha=0.6, s=10)

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Position Comparison: FT vs Panda vs OptiTrack')
plt.grid(True)
plt.legend()
plt.axis('equal')  # Make axes equal scale

# Print some basic statistics
print("\nPosition Statistics (mean ± std):")
print(f"\nX axis:")
print(f"FT: {df['ft_x'].mean():.3f} ± {df['ft_x'].std():.3f}")
print(f"Panda: {df['panda_x'].mean():.3f} ± {df['panda_x'].std():.3f}")

print(f"\nY axis:")
print(f"FT: {df['ft_y'].mean():.3f} ± {df['ft_y'].std():.3f}")
print(f"Panda: {df['panda_y'].mean():.3f} ± {df['panda_y'].std():.3f}")

plt.show()