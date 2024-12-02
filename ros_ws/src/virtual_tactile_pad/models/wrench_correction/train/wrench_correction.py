import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import time
import numpy as np
from pathlib import Path
import pandas as pd

class WrenchCorrectionMLP(nn.Module):
    def __init__(self, input_size=6, hidden_sizes=[64, 32, 16]):
        super(WrenchCorrectionMLP, self).__init__()
        
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

class WrenchDataset(Dataset):
    def __init__(self, panda_wrench, ft_wrench):
        self.panda_wrench = torch.FloatTensor(panda_wrench)
        self.ft_wrench = torch.FloatTensor(ft_wrench)

    def __len__(self):
        return len(self.panda_wrench)

    def __getitem__(self, idx):
        return self.panda_wrench[idx], self.ft_wrench[idx]

class WrenchCorrectionTrainer:
    def __init__(self, batch_size=64, epochs=100, learning_rate=0.001):
        self.batch_size = batch_size
        self.epochs = epochs
        self.learning_rate = learning_rate
        self.train_losses = []
        self.val_losses = []
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
    def prepare_data(self, panda_wrench, ft_wrench, val_split=0.2):
        dataset_size = len(panda_wrench)
        val_size = int(dataset_size * val_split)
        train_size = dataset_size - val_size
        
        train_dataset = WrenchDataset(panda_wrench[:train_size], ft_wrench[:train_size])
        val_dataset = WrenchDataset(panda_wrench[train_size:], ft_wrench[train_size:])
        
        self.train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
        self.val_loader = DataLoader(val_dataset, batch_size=self.batch_size, shuffle=False)

    def setup_model(self):
        self.model = WrenchCorrectionMLP().to(self.device)
        self.criterion = nn.MSELoss()
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)

    def train_epoch(self):
        self.model.train()
        total_loss = 0
        
        for batch_idx, (panda_wrench, ft_wrench) in enumerate(self.train_loader):
            panda_wrench = panda_wrench.to(self.device)
            ft_wrench = ft_wrench.to(self.device)
            
            self.optimizer.zero_grad()
            predicted_wrench = self.model(panda_wrench)
            loss = self.criterion(predicted_wrench, ft_wrench)
            
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
            for panda_wrench, ft_wrench in self.val_loader:
                panda_wrench = panda_wrench.to(self.device)
                ft_wrench = ft_wrench.to(self.device)
                
                predicted_wrench = self.model(panda_wrench)
                loss = self.criterion(predicted_wrench, ft_wrench)
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
        
        print(f"\nTraining completed in {time.time() - start_time:.2f} seconds")

    
    def plot_losses(self):
        plt.figure(figsize=(10, 6))
        plt.plot(range(1, self.epochs + 1), self.train_losses, label='Train')
        plt.plot(range(1, self.epochs + 1), self.val_losses, label='Validation')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.legend()
        plt.title('Training and Validation Loss')
        plt.show()

    def save_model(self, path='wrench_correction_model.pth'):
        torch.save(self.model.state_dict(), path)
        print(f"Model saved to {path}")

if __name__ == "__main__":
    CONFIG = {
        'data_path': '../../../data/raw/data_20241202_161741.csv',
        'batch_size': 64,
        'epochs': 100,
        'learning_rate': 0.001
    }
    
    df = pd.read_csv(CONFIG['data_path'])
    panda_wrench = df[['panda_wrench_force_x', 'panda_wrench_force_y', 'panda_wrench_force_z',
                       'panda_wrench_torque_x', 'panda_wrench_torque_y', 'panda_wrench_torque_z']].values
    ft_wrench = df[['ft_wrench_force_x', 'ft_wrench_force_y', 'ft_wrench_force_z',
                    'ft_wrench_torque_x', 'ft_wrench_torque_y', 'ft_wrench_torque_z']].values
    
    trainer = WrenchCorrectionTrainer(**{k:v for k,v in CONFIG.items() if k != 'data_path'})
    trainer.prepare_data(panda_wrench, ft_wrench)
    trainer.setup_model()
    trainer.train()
    trainer.save_model('../wrench_correction_model.pth')
    trainer.plot_losses()