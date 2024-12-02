import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import time
import numpy as np
from pathlib import Path
import pandas as pd

class TorqueCorrectionMLP(nn.Module):
    def __init__(self, input_size=7, hidden_sizes=[64, 32, 16]):
        super(TorqueCorrectionMLP, self).__init__()
        
        layers = []
        layers.append(nn.Linear(input_size, hidden_sizes[0]))
        layers.append(nn.ReLU())
        layers.append(nn.BatchNorm1d(hidden_sizes[0]))
        
        for i in range(len(hidden_sizes)-1):
            layers.append(nn.Linear(hidden_sizes[i], hidden_sizes[i+1]))
            layers.append(nn.ReLU())
            layers.append(nn.BatchNorm1d(hidden_sizes[i+1]))
        
        layers.append(nn.Linear(hidden_sizes[-1], input_size))
        self.network = nn.Sequential(*layers)
        self.residual_weight = nn.Parameter(torch.tensor([0.5]))
        
    def forward(self, x):
        correction = self.network(x)
        return x + self.residual_weight * correction

class TorqueDataset(Dataset):
    def __init__(self, tau_ext, ft_wrench, jacobian):
        self.tau_ext = torch.FloatTensor(tau_ext)
        self.ft_wrench = torch.FloatTensor(ft_wrench)
        self.jacobian = torch.FloatTensor(jacobian)

    def __len__(self):
        return len(self.tau_ext)

    def __getitem__(self, idx):
        return (self.tau_ext[idx], self.ft_wrench[idx], self.jacobian[idx])

class TorqueCorrectionTrainer:
    def __init__(self, batch_size=32, epochs=60, learning_rate=0.001):
        self.batch_size = batch_size
        self.epochs = epochs
        self.learning_rate = learning_rate
        
        self.train_losses = []
        self.val_losses = []
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        self.model = None
        self.criterion = None
        self.optimizer = None
        self.train_loader = None
        self.val_loader = None

    def prepare_data(self, tau_ext, ft_wrench, jacobian, val_split=0.2):
        """Prepare and split the data into training and validation sets"""
        dataset_size = len(tau_ext)
        val_size = int(dataset_size * val_split)
        train_size = dataset_size - val_size
        
        train_dataset = TorqueDataset(
            tau_ext[:train_size], 
            ft_wrench[:train_size], 
            jacobian[:train_size]
        )
        
        val_dataset = TorqueDataset(
            tau_ext[train_size:], 
            ft_wrench[train_size:], 
            jacobian[train_size:]
        )
        
        self.train_loader = DataLoader(
            train_dataset, 
            batch_size=self.batch_size, 
            shuffle=True
        )
        
        self.val_loader = DataLoader(
            val_dataset, 
            batch_size=self.batch_size, 
            shuffle=False
        )

    def setup_model(self):
        self.model = TorqueCorrectionMLP().to(self.device)
        self.criterion = nn.MSELoss()
        self.optimizer = torch.optim.Adam(
            self.model.parameters(),
            lr=self.learning_rate
        )

    def compute_wrench_loss(self, predicted_torque, ft_wrench, jacobian):
        batch_size = predicted_torque.shape[0]
        jacobian_t = jacobian.transpose(1, 2)
        computed_wrench = torch.bmm(
            torch.linalg.pinv(jacobian_t.float()), 
            predicted_torque.view(batch_size, -1, 1)
        ).squeeze(-1)
        return self.criterion(computed_wrench, ft_wrench)

    def train_epoch(self):
        self.model.train()
        total_loss = 0
        
        for batch_idx, (tau_ext, ft_wrench, jacobian) in enumerate(self.train_loader):
            tau_ext = tau_ext.to(self.device)
            ft_wrench = ft_wrench.to(self.device)
            jacobian = jacobian.to(self.device)
            
            self.optimizer.zero_grad()
            predicted_torque = self.model(tau_ext)
            loss = self.compute_wrench_loss(predicted_torque, ft_wrench, jacobian)
            
            loss.backward()
            self.optimizer.step()
            
            total_loss += loss.item()
            
            if batch_idx % 10 == 0:
                print(f'Batch [{batch_idx}/{len(self.train_loader)}], Loss: {loss.item():.6f}')
        
        return total_loss / len(self.train_loader)

    def validate(self):
        self.model.eval()
        total_loss = 0
        
        with torch.no_grad():
            for tau_ext, ft_wrench, jacobian in self.val_loader:
                tau_ext = tau_ext.to(self.device)
                ft_wrench = ft_wrench.to(self.device)
                jacobian = jacobian.to(self.device)
                
                predicted_torque = self.model(tau_ext)
                loss = self.compute_wrench_loss(predicted_torque, ft_wrench, jacobian)
                total_loss += loss.item()
        
        return total_loss / len(self.val_loader)

    def train(self):
        print("Starting training...")
        start_time = time.time()
        
        for epoch in range(1, self.epochs + 1):
            print(f"\nEpoch {epoch}/{self.epochs}")
            train_loss = self.train_epoch()
            val_loss = self.validate()
            
            self.train_losses.append(train_loss)
            self.val_losses.append(val_loss)
            
            print(f"Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}")
        
        training_time = time.time() - start_time
        print(f"\nTraining completed in {training_time:.2f} seconds")

    def plot_losses(self):
        plt.figure(figsize=(10, 6))
        plt.plot(range(1, self.epochs + 1), self.train_losses, label='Train')
        plt.plot(range(1, self.epochs + 1), self.val_losses, label='Validation')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.legend()
        plt.title('Training and Validation Loss')
        plt.show()

    def save_model(self, path='torque_correction_model.pth'):
        torch.save(self.model.state_dict(), path)
        print(f"Model saved to {path}")

    def run_training(self, tau_ext, ft_wrench, jacobian):
        self.prepare_data(tau_ext, ft_wrench, jacobian)
        self.setup_model()
        self.train()

def load_data(data_path):
    """Load and preprocess data from CSV"""
    df = pd.read_csv(data_path)
    
    # Extract tau_ext (7 dimensions)
    tau_ext = df[[f'tau_ext_{i}' for i in range(7)]].values
    
    # Extract ft_wrench (6 dimensions: force + torque)
    ft_wrench = df[['ft_wrench_force_x', 'ft_wrench_force_y', 'ft_wrench_force_z',
                    'ft_wrench_torque_x', 'ft_wrench_torque_y', 'ft_wrench_torque_z']].values
    
    # Extract jacobian (6x7 matrix)
    jacobian = np.zeros((len(df), 6, 7))
    for i in range(6):
        for j in range(7):
            jacobian[:, i, j] = df[f'jacobian_{i}_{j}'].values
    
    print(f"Loaded data shapes:")
    print(f"tau_ext: {tau_ext.shape}")
    print(f"ft_wrench: {ft_wrench.shape}")
    print(f"jacobian: {jacobian.shape}")
    
    return tau_ext, ft_wrench, jacobian

if __name__ == "__main__":
    torch.manual_seed(42)
    np.random.seed(42)
    
    CONFIG = {
        'data_path': '../../../data/raw/data_20241202_161741.csv',
        'batch_size': 64,
        'epochs': 100,
        'learning_rate': 0.001,
        'model_save_path': '../torque_correction_model.pth',
        'hidden_sizes': [64, 32, 16]
    }
    
    Path(CONFIG['data_path']).parent.mkdir(parents=True, exist_ok=True)
    Path(CONFIG['model_save_path']).parent.mkdir(parents=True, exist_ok=True)
    
    print("Loading data...")
    try:
        tau_ext, ft_wrench, jacobian = load_data(CONFIG['data_path'])
    except FileNotFoundError as e:
        print(f"Error loading data: {e}")
        exit(1)
    
    print("\nInitializing trainer...")
    trainer = TorqueCorrectionTrainer(
        batch_size=CONFIG['batch_size'],
        epochs=CONFIG['epochs'],
        learning_rate=CONFIG['learning_rate']
    )
    
    print("\nStarting training pipeline...")
    try:
        trainer.run_training(tau_ext, ft_wrench, jacobian)
        trainer.save_model(CONFIG['model_save_path'])
        
        print("\nTraining completed successfully!")
        print(f"Model saved to: {CONFIG['model_save_path']}")
        
        print("\nGenerating loss plots...")
        trainer.plot_losses()
        
    except Exception as e:
        print(f"\nError during training: {e}")
        raise