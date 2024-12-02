import matplotlib.pyplot as plt
import torch
import torchvision
import torchvision.transforms as transforms
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
import time
import random

class MNISTNet(nn.Module):
    def __init__(self):
        super(MNISTNet, self).__init__()
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

class MNISTTrainer:
    def __init__(self, data_path='../../data/external', batch_size=64, epochs=5, learning_rate=0.001):
        self.data_path = data_path
        self.batch_size = batch_size
        self.epochs = epochs
        self.learning_rate = learning_rate
        
        # Initialize lists to store metrics
        self.train_losses = []
        self.train_accuracies = []
        self.test_losses = []
        self.test_accuracies = []
        
        # Set device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        # Initialize model, criterion, and optimizer
        self.model = None
        self.criterion = None
        self.optimizer = None
        
        # Initialize data loaders
        self.train_loader = None
        self.test_loader = None
        self.train_set = None
        self.test_set = None

    def load_data(self):
        """Load and prepare MNIST dataset"""
        print("Loading MNIST dataset...")
        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.1307,), (0.3081,))
        ])

        self.train_set = torchvision.datasets.MNIST(
            root=self.data_path,
            train=True,
            download=True,
            transform=transform
        )
        
        self.test_set = torchvision.datasets.MNIST(
            root=self.data_path,
            train=False,
            download=True,
            transform=transform
        )

        self.train_loader = DataLoader(
            self.train_set,
            batch_size=self.batch_size,
            shuffle=True
        )
        
        self.test_loader = DataLoader(
            self.test_set,
            batch_size=self.batch_size,
            shuffle=False
        )
        
        print("MNIST dataset loaded successfully!")

    def setup_model(self):
        """Initialize model, criterion, and optimizer"""
        self.model = MNISTNet().to(self.device)
        self.criterion = nn.CrossEntropyLoss()
        self.optimizer = torch.optim.Adam(
            self.model.parameters(),
            lr=self.learning_rate
        )

    def train_epoch(self, epoch):
        """Train for one epoch"""
        self.model.train()
        correct = 0
        train_loss = 0
        
        for batch_idx, (data, target) in enumerate(self.train_loader):
            data, target = data.to(self.device), target.to(self.device)
            
            self.optimizer.zero_grad()
            output = self.model(data)
            loss = self.criterion(output, target)
            loss.backward()
            self.optimizer.step()
            
            train_loss += loss.item()
            pred = output.argmax(dim=1, keepdim=True)
            correct += pred.eq(target.view_as(pred)).sum().item()
            
            if batch_idx % 100 == 0:
                print(f'Train Epoch: {epoch} [{batch_idx * len(data)}/{len(self.train_loader.dataset)} '
                      f'({100. * batch_idx / len(self.train_loader):.0f}%)]\tLoss: {loss.item():.6f}')
        
        train_loss /= len(self.train_loader)
        train_accuracy = 100. * correct / len(self.train_loader.dataset)
        
        self.train_losses.append(train_loss)
        self.train_accuracies.append(train_accuracy)
        
        return train_loss, train_accuracy

    def test(self):
        """Test the model"""
        self.model.eval()
        test_loss = 0
        correct = 0
        
        with torch.no_grad():
            for data, target in self.test_loader:
                data, target = data.to(self.device), target.to(self.device)
                output = self.model(data)
                test_loss += self.criterion(output, target).item()
                pred = output.argmax(dim=1, keepdim=True)
                correct += pred.eq(target.view_as(pred)).sum().item()

        test_loss /= len(self.test_loader)
        test_accuracy = 100. * correct / len(self.test_loader.dataset)
        
        self.test_losses.append(test_loss)
        self.test_accuracies.append(test_accuracy)
        
        print(f'\nTest set: Average loss: {test_loss:.4f}, '
              f'Accuracy: {correct}/{len(self.test_loader.dataset)} '
              f'({test_accuracy:.2f}%)\n')
        
        return test_loss, test_accuracy

    def train(self):
        """Complete training process"""
        print("Starting training...")
        start_time = time.time()
        
        for epoch in range(1, self.epochs + 1):
            train_loss, train_accuracy = self.train_epoch(epoch)
            test_loss, test_accuracy = self.test()

        training_time = time.time() - start_time
        print(f"Training completed in {training_time:.2f} seconds")

    def plot_results(self):
        """Plot training and testing results"""
        plt.figure(figsize=(12, 5))
        
        # Plot loss
        plt.subplot(1, 2, 1)
        plt.plot(range(1, self.epochs + 1), self.train_losses, label='Train')
        plt.plot(range(1, self.epochs + 1), self.test_losses, label='Test')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.legend()
        plt.title('Training and Test Loss')

        # Plot accuracy
        plt.subplot(1, 2, 2)
        plt.plot(range(1, self.epochs + 1), self.train_accuracies, label='Train')
        plt.plot(range(1, self.epochs + 1), self.test_accuracies, label='Test')
        plt.xlabel('Epoch')
        plt.ylabel('Accuracy (%)')
        plt.legend()
        plt.title('Training and Test Accuracy')

        plt.tight_layout()
        plt.show()

    def predict_and_display(self, num_images=5):
        """Display predictions for random test images"""
        fig, axes = plt.subplots(1, num_images, figsize=(15, 3))
        self.model.eval()

        with torch.no_grad():
            for i in range(num_images):
                idx = random.randint(0, len(self.test_set) - 1)
                img, true_label = self.test_set[idx]
                
                # Make prediction
                img = img.unsqueeze(0).to(self.device)
                output = self.model(img)
                pred_label = output.argmax(dim=1, keepdim=True).item()
                
                # Display image and prediction
                axes[i].imshow(img.cpu().squeeze(), cmap='gray')
                axes[i].set_title(f'Pred: {pred_label}\nTrue: {true_label}')
                axes[i].axis('off')

        plt.tight_layout()
        plt.show()

    def evaluate(self):
        """Calculate overall accuracy on test set"""
        self.model.eval()
        correct = 0
        total = 0
        
        with torch.no_grad():
            for data, target in self.test_loader:
                data, target = data.to(self.device), target.to(self.device)
                outputs = self.model(data)
                _, predicted = torch.max(outputs.data, 1)
                total += target.size(0)
                correct += (predicted == target).sum().item()

        accuracy = 100 * correct / total
        print(f'Overall accuracy on the test set: {accuracy:.2f}%')
        return accuracy

    def save_model(self, path='../mnist_cnn.pth'):
        """Save the trained model"""
        torch.save(self.model.state_dict(), path)
        print(f"Model saved to {path}")

    def run_training(self):
        """Run complete training pipeline"""
        self.load_data()
        self.setup_model()
        self.train()
        self.plot_results()
        print("\nMaking predictions for 5 random images from the test set:")
        self.predict_and_display()
        self.evaluate()
        self.save_model()

if __name__ == "__main__":
    # Create trainer instance and run training
    trainer = MNISTTrainer(
        data_path='../../../data/external',
        batch_size=64,
        epochs=5,
        learning_rate=0.001
    )
    trainer.run_training()