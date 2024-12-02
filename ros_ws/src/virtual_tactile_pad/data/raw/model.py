import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neural_network import MLPRegressor
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error, r2_score
import joblib
import os
import rospkg

# Get package path using rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('virtual_tactile_pad')

# Read the CSV file
df = pd.read_csv('data_20241202_141405.csv')

# Filter out rows where both x and y are 0 for each system

# Prepare input features (Panda wrench)
X = df[[
    'panda_wrench_force_x', 'panda_wrench_force_y', 'panda_wrench_force_z',
    'panda_wrench_torque_x', 'panda_wrench_torque_y', 'panda_wrench_torque_z'
]]

# Prepare target values (FT sensor wrench - ground truth)
y = df[[
    'ft_wrench_force_x', 'ft_wrench_force_y', 'ft_wrench_force_z',
    'ft_wrench_torque_x', 'ft_wrench_torque_y', 'ft_wrench_torque_z'
]]

# Split the data
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Scale the features
scaler_X = StandardScaler()
scaler_y = StandardScaler()

X_train_scaled = scaler_X.fit_transform(X_train)
X_test_scaled = scaler_X.transform(X_test)
y_train_scaled = scaler_y.fit_transform(y_train)
y_test_scaled = scaler_y.transform(y_test)

# Create and train the model
model = MLPRegressor(
    hidden_layer_sizes=(100, 50),  # Two hidden layers
    activation='relu',
    solver='adam',
    max_iter=1000,
    random_state=42,
    verbose=True
)

model.fit(X_train_scaled, y_train_scaled)

# Make predictions
y_pred_scaled = model.predict(X_test_scaled)
y_pred = scaler_y.inverse_transform(y_pred_scaled)

# Calculate errors
mse = mean_squared_error(y_test, y_pred)
r2 = r2_score(y_test, y_pred)

print(f"Mean Squared Error: {mse}")
print(f"R² Score: {r2}")

# Plot results for each component
components = ['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']
fig, axes = plt.subplots(3, 2, figsize=(15, 15))
axes = axes.ravel()

for idx, component in enumerate(components):
    true_values = y_test.iloc[:, idx]
    predicted_values = y_pred[:, idx]
    
    axes[idx].scatter(true_values, predicted_values, alpha=0.5)
    axes[idx].plot([true_values.min(), true_values.max()], 
                   [true_values.min(), true_values.max()], 
                   'r--', lw=2)
    axes[idx].set_xlabel(f'True {component}')
    axes[idx].set_ylabel(f'Predicted {component}')
    axes[idx].set_title(f'{component} Prediction vs Ground Truth')
    
plt.tight_layout()
plt.show()

# Plot error distribution
errors = y_test.values - y_pred
fig, axes = plt.subplots(3, 2, figsize=(15, 15))
axes = axes.ravel()

for idx, component in enumerate(components):
    axes[idx].hist(errors[:, idx], bins=50)
    axes[idx].set_title(f'{component} Error Distribution')
    axes[idx].set_xlabel('Error')
    axes[idx].set_ylabel('Frequency')
    
plt.tight_layout()
plt.show()

# Print component-wise metrics
for idx, component in enumerate(components):
    component_mse = mean_squared_error(y_test.iloc[:, idx], y_pred[:, idx])
    component_r2 = r2_score(y_test.iloc[:, idx], y_pred[:, idx])
    print(f"\n{component}:")
    print(f"MSE: {component_mse:.6f}")
    print(f"R²: {component_r2:.6f}")

# In your training script:
import joblib
import os

# After training your model
model_dir = os.path.join(package_path, 'models')
os.makedirs(model_dir, exist_ok=True)

joblib.dump(model, os.path.join(model_dir, 'wrench_correction_model.pkl'))
joblib.dump(scaler_X, os.path.join(model_dir, 'scaler_X.pkl'))
joblib.dump(scaler_y, os.path.join(model_dir, 'scaler_y.pkl'))