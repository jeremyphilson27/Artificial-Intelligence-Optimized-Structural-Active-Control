# AI Optimized Structural Active Control #
This project implements an **AI-driven Active Mass Damper (AMD)** system to safeguard Single Degree of Freedom (SDOF) structures against earthquake excitations. It utilizes a hybrid workflow where **Reinforcement Learning (RL)** determines baseline PID gains, and a **Neural Network (NN)** acts as an adaptive controller for comparison.
## 📂 Repository Structure ##
### 1. Reinforcement Learning (Python) ### 
- `pid_rl_optimization.ipynb`: The main Jupyter Notebook. Trains the RL agent (PPO/Q-Learning) and exports the optimal gains. Inside the `rl_training` folder. For more information, open `README.md` inside the folder.
- `RL_gains.txt`: (Auto-generated) The output file containing the tuned parameters ($K_p, K_i, K_d$) passed from Python to MATLAB.
### 2. Neural Network & Validation (MATLAB) ### 
- `AI_Comparison.m`: **Run this file**. It is the main automation script that:
  1) Reads `RL_gains.txt`.
  2) Trains the Neural Network.
  3) Runs the Simulink simulation.
  4) Generates comparison plots and animations.
- `TF.slx`: The Simulink model containing the SDOF plant and controller blocks.
- `EQ1.mat`: Simulated earthquake data for training.
- `Hualien.mat` / `Elcentro.mat`: Real-world seismic datasets used for validation.

## 🚀 Usage Tutorial ## 
This project uses a **sequential workflow**. You must run the Python module first to generate the necessary gain parameters for the MATLAB simulation.
### Step 1: RL Training (Python) ### 
The RL agent interacts with a simulated environment to learn the best PID parameters.
1. **Install Dependencies**:
   ```bash
   pip install stable-baselines3 gymnasium shimmy scipy notebook
   ```

   Also you need to install `matlab.engine` dependency. You might check the [official documentation](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html) for the installation guide. Otherwise, you might not be able to run `pid_rl_optimization.ipynb` to do the RL training.
   
2. **Run the Notebook**:
   - Open `pid_rl_optimization.ipynb`.
   - Execute all cells to train the agent.
   - **Verify Output**: Upon completion, ensure a file named `RL_gains.txt` has been created in your directory. This file contains the optimized $K_p, K_i, K_d$ values.
### Step 2: NN Training & Comparison (MATLAB) ###
The MATLAB module picks up where Python left off. It imports the RL gains, trains a separate Neural Network, and compares both against a Passive system.
1. Setup:
   - Move the latest `RL_gains.txt` to the parent folder.
   - Ensure `RL_gains.txt`, `AI_Comparison.m`, `TF.slx`, and all `.mat` data files are in the **same folder**.
3. Run the Automation Script:
   - Open MATLAB.
   - Open `AI_Comparison.m`.
   - Click **Run**.
2. Process:
   - The script will automatically fetch the gains from `RL_gains.txt`.
   - It will train the Neural Network using `EQ1.mat`.
   - It will simulate the response for **Hualien** and **El Centro** earthquakes.
3. Results:
   Wait for the script to finish. It will generate comparison plots and calculate the **RMSE (Root Mean Square Error)** for:
   - Passive (No Control)
   - RL Optimized
   - NN Optimized

## 📊 Expected Results ##
After running `AI_Comparison.m`, you will see a performance summary similar to this:
| Earthquake | Method | RMSE | Result |
| --- | --- | --- | --- |
| Hualien | Passive | 0.0114 | High Vibration |
|  | **RL Optimized** | **0.0000** | **Best Performance** |
|  | NN Optimized | 0.0003 | Very Stable |
| El Centro | Passive | 0.0448 | High Vibration |
|  | RL Optimized | 0.0048 | Good Damping |
|  | **NN Optimized** | **0.0013** | **Best Robustness** |

## 🛠️ Troubleshooting ## 
- **"File not found: RL_gains.txt"**: Ensure you have successfully run the Python notebook (Step 1) and that the file was saved in the same directory as your MATLAB scripts.
- **Simulink Errors**: If `TF.slx` fails to load, ensure you are using a compatible version of MATLAB (R2023a or newer is recommended) and have the Control System Toolbox installed.

## 👥 Authors ##
- Murry Raditya (D11305817)
- Jeremy Philson (M11305826)
- Ishaq Adheltyo (M11402805)
- Muhammad Zhafran Musyaffa (M11402813)

_National Taiwan University of Science and Technology (NTUST)_

## Notes ##
`Experiment` folder contains data of experiments conducted to determine the parameters of NN. There are also `Experiment.m` which is the code used to run the parameter experiment, and `Manual.m` that can be used to experiment with manually inputted PID gains.
