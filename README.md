# Tocatl Hexapod 

## Python Virtual Environment Setup

This project uses a Python virtual environment to manage dependencies separately from system Python packages.

### Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy installed
- Python 3.10+

---

## Initial Setup

### 1. Create Python Virtual Environment
```bash
# Navigate to project root
cd ~/romela_tocatl

# Create virtual environment
python3 -m venv .venv

# Activate virtual environment
source .venv/bin/activate
```

### 2. Install Python Dependencies
```bash
# Upgrade pip
pip install --upgrade pip

# Install core dependencies
pip install numpy scipy matplotlib

# Install ROS2 Python packages (if needed beyond ros-jazzy-desktop)
pip install transforms3d

# For development/testing
pip install pytest black flake8