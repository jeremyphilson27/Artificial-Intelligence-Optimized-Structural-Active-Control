%% nn_pid_experiment.m
% EXPERIMENT MODE: Runs 10 times, exports to TXT and saves Plots.

clc; clear; close all;

%% ===== 0. EXPERIMENT SETUP =====
nRuns = 10;                       % Number of experiments

lr          = 0.9;
lambdaFF    = 0.85;   

lrStr = sprintf('%.2f', lr);       
lfStr = sprintf('%.2f', lambdaFF);

lrStr = strrep(lrStr, '.',',');
lfStr = strrep(lfStr, '.',',');

txtFileName = sprintf('EQ1_LR%s_LF%s.txt', lrStr, lfStr);

resultsMatrix = zeros(nRuns, 6); % Store: [RunID, Epoch, RMSE_smooth, Kp, Ki, Kd]

%% ===== 1. Input error ===== 
e0   = 1.0;    
de0  = 0.0;    
ie0  = 0.0;    

x_in   = [e0; de0; ie0];   
nInput = numel(x_in);

%% ===== OPTIONAL: FAST RESTART =====
% Uncomment lines below if you want to speed up Simulink significantly
% load_system('TF');
% set_param('TF', 'FastRestart', 'on'); 

%% ===== OUTER LOOP: RUN EXPERIMENT =====
for iRun = 1:nRuns
    fprintf('\n========================================\n');
    fprintf('STARTING RUN %d of %d\n', iRun, nRuns);
    fprintf('========================================\n');

    %% ===== 2. Arsitektur NN (Re-initialize for every run) =====
    nH1  = 16;   
    nH2  = 16;   
    nOut = 3;    

    rng('shuffle'); % Ensure different random weights each time

    % Layer 1
    W1 = 0.1 * randn(nH1, nInput);   
    b1 = zeros(nH1, 1);

    % Layer 2
    W2 = 0.1 * randn(nH2, nH1);      
    b2 = zeros(nH2, 1);

    % Output layer
    W3 = 0.1 * randn(nOut, nH2);     
    b3 = zeros(nOut, 1);

    %% ===== 3. HYPERPARAMETER & STOP CONDITION =====
    maxEpochs   = 10000;       
    useRmseStop = true;      
    targetRMSE  = 0.0001;     

           
    eps_fd      = 5e-2;      
       

    lossHist        = zeros(maxEpochs,1);   
    lossHistSmooth  = zeros(maxEpochs,1);   

    % Reset Gradient smoothing variables
    gradEff_prev = zeros(3,1);
    havePrevGrad = false;

    %% ===== 4. Training Loop =====
    finalEpoch = maxEpochs; % Default if not stopped early
    
    for epoch = 1:maxEpochs

        % ---------- FORWARD PASS ----------
        z1 = W1 * x_in + b1;        
        a1 = tanh(z1);

        z2 = W2 * a1 + b2;          
        a2 = tanh(z2);

        z3 = W3 * a2 + b3;          

        gains = exp(z3);            
        Kp = gains(1);
        Ki = gains(2);
        Kd = gains(3);

        % ---------- RUN SIMULINK ----------
        L0 = run_sim_pid(Kp, Ki, Kd);   
        lossHist(epoch) = L0;

        % ---------- LOSS SMOOTHED ----------
        if epoch == 1
            Lsmooth = L0;
        else
            Lsmooth = 0.1 * lossHistSmooth(epoch-1) + (1-0.1) * L0;
        end
        lossHistSmooth(epoch) = Lsmooth;

        % ---------- STOP CONDITION ----------
        if useRmseStop && (Lsmooth < targetRMSE)
            % Keep this terse to not clutter terminal
            % fprintf('STOP: RMSE_smooth %.6f < target %.6f di epoch %d\n', ...
            %        Lsmooth, targetRMSE, epoch);
            lossHist       = lossHist(1:epoch);
            lossHistSmooth = lossHistSmooth(1:epoch);
            finalEpoch = epoch; 
            break;
        end

        % ---------- FINITE DIFFERENCE ----------
        grad_g_new = zeros(3,1);
        for j = 1:3
            g2 = gains;
            g2(j) = g2(j) + eps_fd;          

            Lp = run_sim_pid(g2(1), g2(2), g2(3));
            grad_g_new(j) = (Lp - L0) / eps_fd;
        end

        % ---------- GRADIENT FF ----------
        if ~havePrevGrad
            gradEff = grad_g_new;
            havePrevGrad = true;
        else
            gradEff = lambdaFF * gradEff_prev + (1-lambdaFF) * grad_g_new;
        end
        gradEff_prev = gradEff;

        % ---------- BACKPROP ----------
        dL_dz3 = gradEff .* gains;          
        dL_dW3 = dL_dz3 * a2.';             
        dL_db3 = dL_dz3;                    

        dL_da2  = W3.' * dL_dz3;            
        da2_dz2 = 1 - a2.^2;                
        dL_dz2  = dL_da2 .* da2_dz2;

        dL_dW2 = dL_dz2 * a1.';             
        dL_db2 = dL_dz2;                    

        dL_da1  = W2.' * dL_dz2;            
        da1_dz1 = 1 - a1.^2;                
        dL_dz1  = dL_da1 .* da1_dz1;        

        dL_dW1 = dL_dz1 * x_in.';           
        dL_db1 = dL_dz1;                    

        % ---------- UPDATE ----------
        W3 = W3 - lr * dL_dW3;
        b3 = b3 - lr * dL_db3;

        W2 = W2 - lr * dL_dW2;
        b2 = b2 - lr * dL_db2;

        W1 = W1 - lr * dL_dW1;
        b1 = b1 - lr * dL_db1;
        
        % Print progress every 50 epochs
        if mod(epoch, 50) == 0 || epoch == 1
             fprintf('   Running... Epoch %3d | RMSE_sm = %.6f\n', epoch, Lsmooth);
        end
    end

    %% ===== 5. SAVE & PRINT RESULTS FOR THIS RUN =====
    
    % Store Data
    finalRMSE = lossHistSmooth(end);
    resultsMatrix(iRun, :) = [iRun, finalEpoch, finalRMSE, Kp, Ki, Kd];
    
    % --- NEW: Print Final Stats to Terminal ---
    fprintf('------------------------------------------------------------\n');
    fprintf('RUN %d COMPLETED SUCCESSFULLY\n', iRun);
    fprintf('   Epochs : %d\n', finalEpoch);
    fprintf('   RMSE   : %.6f\n', finalRMSE);
    fprintf('   Kp     : %.4f\n', Kp);
    fprintf('   Ki     : %.4f\n', Ki);
    fprintf('   Kd     : %.4f\n', Kd);
    fprintf('------------------------------------------------------------\n');
    
    % Plot and Save Figure
    f = figure('Visible', 'off'); 
    plot(lossHist,'-o','LineWidth',1); hold on;
    plot(lossHistSmooth,'-x','LineWidth',1.5);
    xlabel('Epoch'); ylabel('RMSE');
    legend('RMSE raw','RMSE smoothed (\lambda)');
    
    title(sprintf('Run %d: NN-PID (Ep: %d, RMSE: %.5f)', iRun, finalEpoch, finalRMSE));
    grid on;
    
    figName = sprintf('EQ1_LR%s_LF%s_Run_%d.png', lrStr, lfStr, iRun);
    saveas(f, figName);
    close(f); 
    
end % End of Outer Loop

%% ===== 6. EXPORT TO TXT FILE =====
VarNames = {'Run_ID', 'Last_Epoch', 'RMSE_Smooth', 'Kp', 'Ki', 'Kd'};
T = array2table(resultsMatrix, 'VariableNames', VarNames);

% Write to TXT (Tab Delimited) - Readable by Excel, Notepad, etc.
writetable(T, txtFileName, 'Delimiter', '\t');
fprintf('\nAll experiments finished. Data exported to %s\n', txtFileName);

%% ==========================================================
% Local function: Run Simulink with TIMEOUT and ERROR CATCHING
%% ==========================================================
function RMSE_val = run_sim_pid(Kp, Ki, Kd)
    assignin('base','Kp',Kp);
    assignin('base','Ki',Ki);
    assignin('base','Kd',Kd);

    persistent modelLoaded
    if isempty(modelLoaded) || ~modelLoaded
        load_system('TF');  
        modelLoaded = true;
    end

    try
        % === TIMEOUT ADDED HERE ===
        % If simulation takes > 8 seconds (real time), kill it.
        % This prevents the code from getting stuck on bad PID values.
        simOut = sim('TF', 'ReturnWorkspaceOutputs', 'on', 'Timeout', 8);
        
        RMSE_ts = simOut.RMSE;
        RMSE_val = RMSE_ts.Data(end); 
        
        % Check for instability (NaN or Inf)
        if isnan(RMSE_val) || isinf(RMSE_val)
             RMSE_val = 1000.0; % High penalty
        end

    catch ME
        % If it times out or errors, return high penalty so NN learns to avoid this
        % fprintf('   > Unstable parameter detected (Timeout). Penalizing.\n');
        RMSE_val = 1000.0; 
    end
end