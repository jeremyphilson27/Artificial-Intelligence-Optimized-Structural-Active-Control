# PID Parameter Optimization using Q-Learning

Optimize PID controller parameters (Kp, Ki, Kd) using Q-Learning reinforcement learning with MATLAB Simulink models.

## Project Overview

This project implements a reinforcement learning pipeline that:
- Runs MATLAB Simulink simulations (TF.slx) with different PID parameters
- Extracts RMSE (Root Mean Square Error) values from simulations
- Uses Q-Learning to find optimal Kp, Ki, Kd values that minimize RMSE
- Visualizes training progress and optimization results

## Prerequisites

- **Python 3.8+** (tested with Python 3.13)
- **MATLAB** (with Simulink installed)
- **Python MATLAB Engine** (automatically installed via uv)
- **uv** (modern Python package manager)

### Install uv

If you don't have `uv` installed, install it using:

```bash
# On macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# On Windows
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

# Or using pip
pip install uv
```

See [astral.sh/uv](https://astral.sh/uv) for more installation options.

## Setup

### 1. Clone or navigate to the repository

```bash
cd /Users/franszhafran/code/master/ai/ai-final-project
```

### 2. Create a virtual environment and install dependencies

Using `uv` (recommended):

```bash
# Create virtual environment and install all dependencies
uv sync

# For development (includes testing/linting tools)
uv sync --all-extras
```

**What this does:**
- Creates a `.venv` virtual environment
- Installs all dependencies from `pyproject.toml`
- Creates `uv.lock` for reproducible installations

### 3. Activate the virtual environment

```bash
# On macOS/Linux
source .venv/bin/activate

# On Windows
.venv\Scripts\activate

# Or use uv to run commands directly (no activation needed)
uv run python --version
```

## File Structure

```
.
├── README.md                          # This file
├── pyproject.toml                     # Project configuration (uv/pip)
├── uv.lock                            # Locked dependency versions
├── pid_rl_optimization.ipynb          # Main Jupyter notebook
├── TF.slx                             # MATLAB Simulink model
├── EQ1.mat                            # MATLAB data file
├── pid_training_results.png           # Generated training visualization
└── pid_optimization_results.json      # Generated optimization results
```

## Usage

### Running the Optimization Notebook

#### Option 1: Using Jupyter directly (after activating venv)

```bash
# Activate virtual environment
source .venv/bin/activate

# Start Jupyter
jupyter notebook

# Navigate to and open pid_rl_optimization.ipynb
```

#### Option 2: Using uv directly (no activation needed)

```bash
uv run jupyter notebook
```

### Running the Notebook Cells

The notebook `pid_rl_optimization.ipynb` has 12 cells:

1. **Imports and Setup** - Import necessary libraries
2. **MATLAB Engine Connection** - Connect to MATLAB and load EQ1.mat
3. **Simulink Runner Function** - Function to run simulations with PID parameters
4. **Q-Learning Environment Setup** - Define parameter ranges and hyperparameters
5. **State and Action Functions** - Convert between continuous and discrete parameter spaces
6. **Reward Function** - Convert RMSE to reward signal
7. **Q-Learning Training Loop** - Main training loop (200 episodes)
8. **Results Visualization** - Plot RMSE and parameter evolution
9. **Q-Table Statistics** - Analyze learned Q-values
10. **Validation Run** - Test best parameters found
11. **Export Results** - Save results to JSON
12. **Cleanup** - Close MATLAB engine

**Run the cells sequentially from top to bottom.**

### Expected Output

After running the notebook:

```
Starting Q-Learning Training...
Initial state parameters: (5.05, 2.5275, 1.0549999999999998)

Episode 20/200 | Avg RMSE: 0.523456 | Best RMSE: 0.412345 | Epsilon: 0.3698 | Elapsed: 45.2s
Episode 40/200 | Avg RMSE: 0.451234 | Best RMSE: 0.398765 | Epsilon: 0.1368 | Elapsed: 89.5s
...
Episode 200/200 | Avg RMSE: 0.391234 | Best RMSE: 0.385123 | Epsilon: 0.0100 | Elapsed: 450.3s

Best Parameters Found:
  Kp = 6.2500
  Ki = 2.7500
  Kd = 1.2000
  RMSE = 0.385123
```

Files generated:
- `pid_training_results.png` - Visualization of training progress
- `pid_optimization_results.json` - Optimized parameters and metrics

### Customizing the Optimization

Edit **Cell 4** in the notebook to change:

```python
# Parameter ranges and discretization
KP_MIN, KP_MAX, KP_STEP = 0.1, 10.0, 0.5
KI_MIN, KI_MAX, KI_STEP = 0.01, 5.0, 0.25
KD_MIN, KD_MAX, KD_STEP = 0.01, 2.0, 0.1

# Q-Learning hyperparameters
ALPHA = 0.1          # Learning rate
GAMMA = 0.95         # Discount factor
EPSILON = 1.0        # Initial exploration rate
EPSILON_DECAY = 0.995 # Exploration decay
NUM_EPISODES = 200   # Training episodes
```

## Dependency Management with uv

### View locked dependencies

```bash
uv pip list
```

### Update dependencies

```bash
# Update to latest compatible versions
uv sync --upgrade

# Lock new dependency versions
uv lock
```

### Install additional packages

```bash
# Add a new package and update lock file
uv pip install scipy

# Or use uv to manage project dependencies
uv sync
```

### Common uv Commands

```bash
# Show Python info
uv python list
uv python pin 3.11

# Run commands without activating venv
uv run python script.py
uv run pytest

# Sync with lock file
uv sync

# Upgrade all dependencies
uv sync --upgrade
```

## Troubleshooting

### MATLAB Engine Not Found

If you get an error about MATLAB Engine:

```bash
# Ensure MATLAB is installed and in PATH
which matlab

# Install MATLAB Python Engine (run in MATLAB)
python setup.py install
```

Or from the MATLAB installation directory:
```bash
cd /Applications/MATLAB_R2023b/extern/engines/python
python -m pip install -e .
```

### Virtual Environment Issues

```bash
# Remove and recreate virtual environment
rm -rf .venv
uv sync

# Or use uv directly without venv
uv run jupyter notebook
```

### Port Already in Use (Jupyter)

```bash
# Run on different port
jupyter notebook --port 8889
```

## Project Dependencies

| Package | Purpose | Version |
|---------|---------|---------|
| numpy | Numerical computations | >=1.20.0 |
| matplotlib | Visualization | >=3.5.0 |
| jupyter | Notebook environment | >=1.0.0 |
| matlab-engine | MATLAB integration | >=9.13.0 |

Development dependencies (optional):
- pytest: Testing
- black: Code formatting
- ruff: Linting
- mypy: Type checking

## Algorithm Details

### Q-Learning Configuration

- **State Space**: Discretized (Kp, Ki, Kd) parameters
- **Action Space**: 6 actions (increase/decrease each parameter)
- **Reward Function**: `-RMSE` (minimize error)
- **Update Rule**: `Q(s,a) ← Q(s,a) + α[r + γ·max(Q(s',a')) - Q(s,a)]`

### Learning Hyperparameters

- **Learning Rate (α)**: 0.1 - Controls how much new information overrides old
- **Discount Factor (γ)**: 0.95 - Weights future rewards
- **Exploration Rate (ε)**: Decays from 1.0 to 0.01 - Balance exploration vs exploitation

## License

MIT License

## Support

For issues or questions:
1. Check the troubleshooting section
2. Verify MATLAB and Python versions match requirements
3. Ensure all dependencies are installed: `uv sync`
4. Check that EQ1.mat and TF.slx files exist in the project directory
