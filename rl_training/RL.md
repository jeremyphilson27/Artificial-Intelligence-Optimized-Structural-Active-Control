# Reinforcement Learning Configuration for PID Optimization

## Overview
This document describes the Q-Learning approach used to optimize PID controller parameters (Kp, Ki, Kd) for the TF.slx Simulink model.

## State Space

### State Representation
The state is represented as a tuple of three discrete indices:
```
state = (kp_idx, ki_idx, kd_idx)
```

### Parameter Ranges and Discretization
- **Kp (Proportional Gain)**
  - Range: [0.0, 5.0]
  - Step size: 0.1
  - Number of values: 51

- **Ki (Integral Gain)**
  - Range: [0.0, 5.0]
  - Step size: 0.1
  - Number of values: 51

- **Kd (Derivative Gain)**
  - Range: [0.0, 5.0]
  - Step size: 0.1
  - Number of values: 51

### Total State Space
- **Size**: 51 × 51 × 51 = 132,651 possible states
- **Representation**: Each state maps to continuous PID parameter values

### State Conversion Functions
- `params_to_state(kp, ki, kd)`: Converts continuous parameters to discrete state indices
- `state_to_params(state)`: Converts discrete state indices back to continuous parameters
- Values are clamped to valid ranges during conversion

## Action Space

### Number of Actions
6 discrete actions

### Action Definitions
| Action ID | Description | Effect |
|-----------|-------------|--------|
| 0 | Increase Kp | `kp_idx = min(kp_idx + 1, max_idx)` |
| 1 | Decrease Kp | `kp_idx = max(kp_idx - 1, 0)` |
| 2 | Increase Ki | `ki_idx = min(ki_idx + 1, max_idx)` |
| 3 | Decrease Ki | `ki_idx = max(ki_idx - 1, 0)` |
| 4 | Increase Kd | `kd_idx = min(kd_idx + 1, max_idx)` |
| 5 | Decrease Kd | `kd_idx = max(kd_idx - 1, 0)` |

### Action Constraints
- Actions are bounded to prevent going outside the valid parameter ranges
- Each action modifies exactly one parameter by one step

## Reward Function

### Reward Calculation
```python
reward = -rmse  # Negative RMSE (minimizing error)
```

### Special Cases
- **Simulation Failure**: reward = -100 (severe penalty)
- **Valid Simulation**: reward = -RMSE (lower RMSE → higher reward)

### Objective
Maximize cumulative reward by minimizing RMSE (Root Mean Square Error) from the Simulink simulation.

## Q-Learning Hyperparameters

### Learning Parameters
- **Learning Rate (α)**: 0.1
  - Controls how much new information overrides old information

- **Discount Factor (γ)**: 0.95
  - Determines importance of future rewards

- **Initial Exploration Rate (ε)**: 1.0
  - Starting probability of taking random actions

- **Epsilon Decay**: 0.995
  - Multiplicative decay applied after each episode

- **Minimum Exploration Rate**: 0.01
  - Lower bound for epsilon during training

### Training Configuration
- **Number of Episodes**: 2000
- **Steps per Episode**: 10
- **Q-Table Implementation**: Dictionary with default zero values
- **Initial State**: (0, 0, 0) corresponding to Kp=0.0, Ki=0.0, Kd=0.0

## Q-Learning Update Rule

### Update Equation
```python
Q(s, a) ← Q(s, a) + α × [R + γ × max Q(s', a') - Q(s, a)]
```

Where:
- `s`: current state
- `a`: action taken
- `s'`: next state
- `R`: reward received
- `α`: learning rate
- `γ`: discount factor

### Implementation
```python
max_next_q = np.max(Q_table[new_state])
Q_table[state][action] = Q_table[state][action] + ALPHA * (reward + GAMMA * max_next_q - Q_table[state][action])
```

## Action Selection Strategy

### Epsilon-Greedy Policy
```python
if random() < epsilon:
    action = random_action()  # Explore
else:
    action = argmax(Q_table[state])  # Exploit
```

- **Exploration**: Random action selection to discover new states
- **Exploitation**: Choose action with highest Q-value for current state
- Balance shifts from exploration to exploitation as epsilon decays

## Environment Interface

### Simulink Simulation
- **Model**: TF.slx
- **Input**: PID parameters (Kp, Ki, Kd)
- **Output**: RMSE value from out.mat file
- **Data Source**: EQ1.mat (loaded at initialization)

### Simulation Execution
```python
rmse = run_simulink(Kp, Ki, Kd, eng)
```

### Error Handling
- Failed simulations return `None`
- Failures are tracked and receive penalty reward of -100

## Early Stopping

### Configuration
- **Enabled**: True
- **Target RMSE**: 0.00001
- **Behavior**: Training stops when best RMSE ≤ target

### Benefits
- Saves computational resources
- Prevents unnecessary training after optimal solution found

## Convergence Tracking

### Convergence Detection
- **Window Size**: 50 episodes
- **Threshold**: RMSE variance < 0.001

### Metrics Tracked
- RMSE variance over recent episodes
- Steps without improvement
- Convergence episode number

## Performance Metrics

### Training History
- Episode number
- Episode average RMSE
- Best RMSE so far
- Current PID parameters (Kp, Ki, Kd)
- Current epsilon value

### Q-Value Statistics
- Average Q-value
- Maximum Q-value
- Minimum Q-value
- Q-value standard deviation

### Simulation Statistics
- Total simulations run
- Successful simulations
- Failed simulations
- Success rate percentage

## State Space Exploration

### Coverage Metrics
- Unique states visited
- Percentage of total state space explored
- State space coverage = `(states_visited / total_states) × 100%`

### Q-Table Structure
- Implemented as `defaultdict(lambda: np.zeros(NUM_ACTIONS))`
- Automatically initializes new states with zero Q-values
- Sparse representation (only visited states stored)

## Optimization Results Format

### Output Files
1. **pid_optimization_results.json**: Complete results in JSON format
2. **RL_gains.txt**: Final PID parameters (one per line)

### Result Contents
- Best parameters found (Kp, Ki, Kd)
- Best RMSE achieved
- Training episodes completed
- Training time in seconds
- Improvement percentage vs. initial parameters
