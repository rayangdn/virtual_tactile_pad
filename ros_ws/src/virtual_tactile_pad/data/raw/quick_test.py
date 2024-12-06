import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('data_PF_4.csv')

# Filter out rows where both x and y are 0 for each system
df = df[(df['ft_x'] != 0) | (df['ft_y'] != 0)]
df = df[(df['panda_x'] != 0) | (df['panda_y'] != 0)]
df = df[(df['panda_x'] <= 0.2) & (df['panda_x'] >= -0.2)]
df = df[(df['panda_y'] <= 0.2) & (df['panda_y'] >= -0.2)]

# Create a figure with multiple subplots
fig = plt.figure(figsize=(15, 12))

# 1. Scatter plot (top left)
ax1 = plt.subplot(321)
ax1.scatter(df['ft_x'], df['ft_y'], c='blue', label='FT', alpha=0.6, s=10)
ax1.scatter(df['panda_x'], df['panda_y'], c='red', label='Panda', alpha=0.6, s=10)
ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')
ax1.set_title('Position Comparison')
ax1.grid(True)
ax1.legend()
ax1.axis('equal')

# 2. X-axis histogram (top right)
ax2 = plt.subplot(322)
ax2.hist(df['ft_x'], bins=30, alpha=0.5, color='blue', label='FT X')
ax2.hist(df['panda_x'], bins=30, alpha=0.5, color='red', label='Panda X')
ax2.set_xlabel('X Position (m)')
ax2.set_ylabel('Count')
ax2.set_title('X Position Distribution')
ax2.legend()

# 3. Y-axis histogram (middle left)
ax3 = plt.subplot(323)
ax3.hist(df['ft_y'], bins=30, alpha=0.5, color='blue', label='FT Y')
ax3.hist(df['panda_y'], bins=30, alpha=0.5, color='red', label='Panda Y')
ax3.set_xlabel('Y Position (m)')
ax3.set_ylabel('Count')
ax3.set_title('Y Position Distribution')
ax3.legend()

# 4. Box plots positions (middle right)
ax4 = plt.subplot(324)
pos_data = [df['ft_x'], df['panda_x'], df['ft_y'], df['panda_y']]
ax4.boxplot(pos_data, labels=['FT X', 'Panda X', 'FT Y', 'Panda Y'])
ax4.set_title('Position Distributions')
ax4.grid(True)

# 5. Z-force histogram (bottom left)
ax5 = plt.subplot(325)
ax5.hist(df['ft_wrench_force_z'], bins=30, alpha=0.5, color='blue', label='FT Force Z')
ax5.hist(df['panda_wrench_force_z'], bins=30, alpha=0.5, color='red', label='Panda Force Z')
ax5.set_xlabel('Force Z (N)')
ax5.set_ylabel('Count')
ax5.set_title('Z Force Distribution')
ax5.legend()

# 6. Box plots forces (bottom right)
ax6 = plt.subplot(326)
force_data = [df['ft_wrench_force_z'], df['panda_wrench_force_z']]
ax6.boxplot(force_data, labels=['FT Force Z', 'Panda Force Z'])
ax6.set_title('Z Force Distributions')
ax6.grid(True)

plt.tight_layout()

# Print detailed statistics
print("\nDetailed Position Statistics:")
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

print("\nZ Force:")
print(f"FT Force Z:")
print(f"  Mean: {df['ft_wrench_force_z'].mean():.3f}")
print(f"  Std:  {df['ft_wrench_force_z'].std():.3f}")
print(f"  Min:  {df['ft_wrench_force_z'].min():.3f}")
print(f"  Max:  {df['ft_wrench_force_z'].max():.3f}")
print(f"  IQR:  {df['ft_wrench_force_z'].quantile(0.75) - df['ft_wrench_force_z'].quantile(0.25):.3f}")

print(f"\nPanda Force Z:")
print(f"  Mean: {df['panda_wrench_force_z'].mean():.3f}")
print(f"  Std:  {df['panda_wrench_force_z'].std():.3f}")
print(f"  Min:  {df['panda_wrench_force_z'].min():.3f}")
print(f"  Max:  {df['panda_wrench_force_z'].max():.3f}")
print(f"  IQR:  {df['panda_wrench_force_z'].quantile(0.75) - df['panda_wrench_force_z'].quantile(0.25):.3f}")

plt.show()