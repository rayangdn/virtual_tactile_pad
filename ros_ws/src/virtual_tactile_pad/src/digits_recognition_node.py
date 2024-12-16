#!/usr/bin/env python
import rospy
import numpy as np
from virtual_tactile_pad.msg import ContactForce
import csv
import os
import yaml
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms
import rospkg

# Load configuration file
config_path = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

rospack = rospkg.RosPack()
package_path = rospack.get_path('virtual_tactile_pad')
            
class MNISTCNN(nn.Module):
    def __init__(self):
        super(MNISTCNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, 3, 1)
        self.conv2 = nn.Conv2d(32, 64, 3, 1)
        self.dropout1 = nn.Dropout2d(0.25)
        self.dropout2 = nn.Dropout2d(0.5)
        self.fc1 = nn.Linear(9216, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = self.conv1(x)
        x = F.relu(x)
        x = self.conv2(x)
        x = F.relu(x)
        x = F.max_pool2d(x, 2)
        x = self.dropout1(x)
        x = torch.flatten(x, 1)
        x = self.fc1(x)
        x = F.relu(x)
        x = self.dropout2(x)
        x = self.fc2(x)
        output = F.log_softmax(x, dim=1)
        return output
    
class DigitRecognizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('digits_recognizer', anonymous=True)

        # Get model path from ROS package
        model_path = os.path.join(package_path, 'models', 'digits_recognition', 'mnist_cnn.pth')

        # Load the model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = MNISTCNN()
        self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        self.model.to(self.device)  
        self.model.eval()

        # Transform for MNIST format
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.1307,), (0.3081,))
        ])
        
        # Configuration parameters
        self.FORCE_THRESHOLD = config['processing']['force_threshold'] # Threshold to start/end trajectory
        self.TIME_WINDOW = config['digits_recognition']['time_window'] # Time window to start/end trajectory
        self.position_buffer = []    # Store trajectory positions
        self.timestamps = []        # Store timestamps
        
        # State variables
        self.trajectory_active = False
        self.last_force_time = None
        self.trajectory_start_time = None
        
        # Get output directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(os.path.dirname(current_dir), 'data', 'raw')
        file_name = "trajectory_data.csv"
        self.csv_path = os.path.join(data_dir, file_name)

        # Create initial CSV file
        self.clear_trajectory_file()
        
        # Subscribe to contact force topic
        rospy.Subscriber("/ft_process_node/contact_force", 
                         ContactForce, 
                         self.contact_callback)
        rospy.loginfo("Subscribed to FT sensor contact force topic")
        rospy.loginfo("Digit recognizer node started")
    
    def clear_trajectory_file(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'contact_x', 'contact_y', 'grid', 'prediction', 'confidence'])

    def save_positions(self, positions, timestamps, grid=None, prediction=None, confidence=None):
        # Convert grid to string representation
        grid_str = np.array2string(grid, separator=',', threshold=np.inf) if grid is not None else ''
        
        # Append data to CSV file
        with open(self.csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write each point with its corresponding timestamp
            for point, timestamp in zip(positions, timestamps):
                if grid is not None and prediction is not None and confidence is not None and point == positions[-1]:  # Add grid only to last point
                    writer.writerow([timestamp, point[0], point[1], grid_str, prediction, confidence])
                else:
                    writer.writerow([timestamp, point[0], point[1], '', '', ''])

        rospy.loginfo(f"Saved {len(positions)} points to {self.csv_path}")
        
    
    def check_trajectory_start(self, force_magnitude, current_time, position):
        if not self.trajectory_active and force_magnitude > self.FORCE_THRESHOLD:
            if self.last_force_time is None:
                self.last_force_time = current_time
                
            elif current_time - self.last_force_time >= self.TIME_WINDOW:
                self.last_force_time = None
                rospy.loginfo("Starting new trajectory...")
                self.trajectory_start_time = current_time
                self.position_buffer = [(position.x, position.y)]
                return True
        elif force_magnitude <= self.FORCE_THRESHOLD:
            self.last_force_time = None
            
        return False
    
    def check_trajectory_end(self, force_magnitude, current_time, position):
        if self.trajectory_active and force_magnitude <= self.FORCE_THRESHOLD:
            if self.last_force_time is None:
                self.last_force_time = current_time
            elif current_time - self.last_force_time >= self.TIME_WINDOW:
                self.last_force_time = None
                rospy.loginfo("Ending trajectory...")
                return True
        elif force_magnitude > self.FORCE_THRESHOLD:
            self.last_force_time = None
            
        return False
    
    def contact_callback(self, msg):
        # Calculate force magnitude
        force_magnitude = np.sqrt(msg.force.x**2 + msg.force.y**2 + msg.force.z**2)
        current_time = msg.header.stamp.to_sec()
        
        if not self.trajectory_active:
            if self.check_trajectory_start(force_magnitude, current_time, msg.position):
                # Clear the CSV file for the new trajectory
                self.clear_trajectory_file()
                self.timestamps = [current_time]
                self.trajectory_active = True
        
        # If trajectory is active, store position
        if self.trajectory_active:
            if not (abs(msg.position.x) < 1e-6 and abs(msg.position.y) < 1e-6):
                self.position_buffer.append((msg.position.x, msg.position.y))
                self.timestamps.append(current_time)

            if self.check_trajectory_end(force_magnitude, current_time, msg.position):

                grid, prediction, confidence = self.process_digit()
                # Save positions with grid
                self.save_positions(self.position_buffer, self.timestamps, grid, prediction, confidence)
                
                self.position_buffer = []
                self.timestamps = []
                self.trajectory_active = False

    def process_digit(self):
        try:
            x_coords = np.array([p[0] for p in self.position_buffer])
            y_coords = np.array([p[1] for p in self.position_buffer])

            # Create grid from trajectory
            grid = self.trajectory_to_grid(x_coords, y_coords)

            # Convert to tensor and get prediction
            tensor_image = self.transform(grid.astype(np.float32))
            tensor_image = tensor_image.to(self.device)
            
            with torch.no_grad():
                output = self.model(tensor_image.unsqueeze(0))
                probabilities = torch.nn.functional.softmax(output, dim=1)
                predicted_class = output.argmax(dim=1).item()
                confidence = probabilities[0][predicted_class].item()
            
            rospy.loginfo(f"Recognized digit: {predicted_class} (confidence: {confidence:.2%})")
            
            return grid, predicted_class, confidence
        except Exception as e:
            rospy.logerr(f"Error processing digit: {e}")
        
        return
        
    def trajectory_to_grid(self, x_coords, y_coords, grid_size=28, line_thickness=0):
        # Normalize coordinates with padding
        x_min, x_max = x_coords.min(), x_coords.max()
        y_min, y_max = y_coords.min(), y_coords.max()
        
        # Add padding
        x_padding = (x_max - x_min) * config['digits_recognition']['padding']['x']
        y_padding = (y_max - y_min) * config['digits_recognition']['padding']['y']
        
        x_min -= x_padding
        x_max += x_padding
        y_min -= y_padding
        y_max += y_padding
        
        x_normalized = ((x_coords - x_min) / (x_max - x_min) * (grid_size - 1))
        y_normalized = ((y_coords - y_min) / (y_max - y_min) * (grid_size - 1))
        
        # Invert y coordinates to switch from bottom-left to top-left origin
        y_normalized = (grid_size - 1) - y_normalized
        
        grid = np.zeros((grid_size, grid_size))
        
        # Draw trajectory with thickness
        for i in range(len(x_normalized)-1):
            x1, y1 = int(round(x_normalized[i])), int(round(y_normalized[i]))
            x2, y2 = int(round(x_normalized[i+1])), int(round(y_normalized[i+1]))
            
            num_steps = max(abs(x2-x1), abs(y2-y1)) * 2
            if num_steps > 0:
                xs = np.linspace(x1, x2, num_steps+1)
                ys = np.linspace(y1, y2, num_steps+1)
                
                for x, y in zip(xs, ys):
                    x, y = int(round(x)), int(round(y))
                    for dx in range(-line_thickness, line_thickness+1):
                        for dy in range(-line_thickness, line_thickness+1):
                            if 0 <= x+dx < grid_size and 0 <= y+dy < grid_size:
                                grid[y+dy, x+dx] = 1
        
        return grid
    
def main():
    try:
        DigitRecognizer()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down data collector...")
    finally:
        rospy.signal_shutdown("User interrupted")

if __name__ == '__main__':
    main()