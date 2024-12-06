import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import StandardScaler
import pandas as pd

class WrenchCorrectionMLP(nn.Module):
    def __init__(self, input_size=6, hidden_sizes=[32, 16]): 
        super(WrenchCorrectionMLP, self).__init__()
        
        # Simplified layer structure with dropout
        layers = []
        layers.append(nn.Linear(input_size, hidden_sizes[0]))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0.2))
        
        for i in range(len(hidden_sizes)-1):
            layers.append(nn.Linear(hidden_sizes[i], hidden_sizes[i+1]))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(0.2))
        
        layers.append(nn.Linear(hidden_sizes[-1], input_size))
        self.network = nn.Sequential(*layers)
        self.residual_weight = nn.Parameter(torch.tensor([0.1]))  # Start with smaller residual weight
        
    def forward(self, x):
        correction = self.network(x)
        return x + torch.tanh(self.residual_weight) * correction  # Bounded residual weight

class WrenchDataset(Dataset):
    def __init__(self, panda_wrench, ft_wrench, scaler=None):
        if scaler is None:
            self.scaler = StandardScaler()
            panda_wrench = self.scaler.fit_transform(panda_wrench)
        else:
            self.scaler = scaler
            panda_wrench = self.scaler.transform(panda_wrench)
            
        self.panda_wrench = torch.FloatTensor(panda_wrench)
        self.ft_wrench = torch.FloatTensor(ft_wrench)

    def __len__(self):
        return len(self.panda_wrench)

    def __getitem__(self, idx):
        return self.panda_wrench[idx], self.ft_wrench[idx]

class WrenchCorrectionTrainer:
    def __init__(self, batch_size=128, epochs=100, learning_rate=0.0005):
        self.batch_size = batch_size
        self.epochs = epochs
        self.learning_rate = learning_rate
        self.train_losses = []
        self.val_losses = []
        self.best_val_loss = float('inf')
        self.patience = 10
        self.patience_counter = 0
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
    def prepare_data(self, panda_wrench, ft_wrench, val_split=0.2):
        dataset_size = len(panda_wrench)
        indices = np.random.permutation(dataset_size)
        val_size = int(dataset_size * val_split)
        
        train_indices = indices[:-val_size]
        val_indices = indices[-val_size:]
        
        # Create and fit scaler on training data only
        self.scaler = StandardScaler()
        panda_wrench_train = self.scaler.fit_transform(panda_wrench[train_indices])
        panda_wrench_val = self.scaler.transform(panda_wrench[val_indices])
        
        train_dataset = WrenchDataset(panda_wrench[train_indices], ft_wrench[train_indices], self.scaler)
        val_dataset = WrenchDataset(panda_wrench[val_indices], ft_wrench[val_indices], self.scaler)
        
        self.train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
        self.val_loader = DataLoader(val_dataset, batch_size=self.batch_size, shuffle=False)

    def setup_model(self):
        self.model = WrenchCorrectionMLP().to(self.device)
        self.criterion = nn.MSELoss()
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=0.01)
        self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(self.optimizer, mode='min', factor=0.5, patience=5)

    def train(self):
        print("Starting training...")
        for epoch in range(1, self.epochs + 1):
            train_loss = self.train_epoch()
            val_loss = self.validate()
            
            self.train_losses.append(train_loss)
            self.val_losses.append(val_loss)
            
            self.scheduler.step(val_loss)
            
            print(f"Epoch {epoch}/{self.epochs}")
            print(f"Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}")
            
            # Early stopping check
            if val_loss < self.best_val_loss:
                self.best_val_loss = val_loss
                self.patience_counter = 0
                # Save best model
                torch.save(self.model.state_dict(), 'best_model.pth')
            else:
                self.patience_counter += 1
                if self.patience_counter >= self.patience:
                    print(f"Early stopping triggered after {epoch} epochs")
                    break
        
        # Load best model
        self.model.load_state_dict(torch.load('best_model.pth'))

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
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            self.optimizer.step()
            
            total_loss += loss.item()
            
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

    def plot_losses(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.train_losses, label='Training Loss', color='blue', alpha=0.7)
        plt.plot(self.val_losses, label='Validation Loss', color='red', alpha=0.7)
        plt.yscale('log')  # Use log scale for better visualization
        plt.xlabel('Epoch')
        plt.ylabel('Loss (log scale)')
        plt.title('Training and Validation Loss Over Time')
        plt.grid(True, which="both", ls="-", alpha=0.2)
        plt.legend()
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    CONFIG = {
        'data_path': '../../../data/raw/data_20241206_091048.csv',
        'batch_size': 128,
        'epochs': 200,
        'learning_rate': 0.005
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
    trainer.plot_losses()  # Added plotting at the end

