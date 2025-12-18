# Tutorial #
1. Save all the files and place them into the same folder ***(IMPORTANT)***.
2. Run the Reinforcement Learning (.ipynb) file to train the RL model using the artificial earthquake data **EQ1.mat**.
3. Ensure the code has successfully finish the training and exported a .txt file named **RL_gains.txt**. (The existing RL_gains.txt file is generated from previous run of the code, it can be directly used.)
4. Open MATLAB and ensure that Simulink is already installed by typing **simulink** at the command window.
5. Open the **AI_Comparison.m** and **TF.slx** files.
6. Run the **AI_Comparison.m** file. After fetching the results of RL training from **RL_gains.txt**, MATLAB will continue and train the NN model using **EQ1.mat**. 
7. After the NN model has been trained, MATLAB will compare both models and passive control (no PID control)'s RMSE scores by running them through two different real-world earthquake datasets (**Hualien.mat** and **Elcentro.mat**).
8. MATLAB will visualize the comparison using an animation that visualizes the structural response to earthquake excitation.
9. After all the comparisons have been completed, MATLAB will export the animations into .mp4 files (The existing videos are generated from previous run).

## Notes ##
- **Experiment.m** is used to experiment the parameters of the NN, which results can be seen inside the **LF Experiment.xlsx** and **LR Experiment.xlsx** files.
- **Manual.m** file can be used to experiment with manually added PID gains.
