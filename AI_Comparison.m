clc; clear; close all;

%% ===== 0. LOAD RL GAINS (External Input) =====
fprintf('--- LOADING RL GAINS ---\n');
try
    % Reading standard text file with commas/newlines
    fid = fopen('RL_gains.txt', 'r');
    if fid == -1, error('RL_gains.txt not found.'); end
    
    % Read 3 floating point numbers separated by commas or whitespace
    rl_raw = textscan(fid, '%f', 'Delimiter', ','); 
    fclose(fid);
    
    rl_vals = rl_raw{1};
    if numel(rl_vals) < 3, error('Not enough data in RL_gains.txt'); end
    
    Kp_rl = rl_vals(1);
    Ki_rl = rl_vals(2);
    Kd_rl = rl_vals(3);
    
    fprintf('RL Gains Loaded: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_rl, Ki_rl, Kd_rl);
catch ME
    fprintf('Error loading RL gains: %s\nUsing default values.\n', ME.message);
    Kp_rl=1; Ki_rl=1; Kd_rl=1;
end


%% ===== 1. PHASE 1: TRAINING (EQ1 - NN Only) =====
fprintf('\n--- PHASE 1: TRAINING NEURAL NETWORK (EQ1) ---\n');

% Load EQ1
data_train = load('EQ1.mat');
if isfield(data_train, 'EQ'), EQ_train = data_train.EQ;
elseif isfield(data_train, 'EQ1'), EQ_train = data_train.EQ1;
else, error('EQ1.mat loaded but could not find EQ variable.'); end

if size(EQ_train,1) < size(EQ_train,2), EQ_train = EQ_train'; end
EQ_train(:,1) = EQ_train(:,1) - EQ_train(1,1);

T_train = EQ_train(end,1); 
fprintf('Training Duration: %.2f s\n', T_train);

% Update Simulink Workspace
assignin('base', 'EQ', EQ_train); 
if ~bdIsLoaded('TF'), load_system('TF'); end
set_param('TF', 'StopTime', num2str(T_train));

% --- HYPERPARAMETERS ---
x_in=[1.0;0.0;0.0]; nInput=3;
rng('shuffle'); 
nH1=16; nH2=16; nOut=3;    
W1=0.1*randn(nH1,nInput); b1=zeros(nH1,1);
W2=0.1*randn(nH2,nH1);    b2=zeros(nH2,1);
W3=0.1*randn(nOut,nH2);   b3=zeros(nOut,1);

lr=0.9; lambdaFF=0.85; maxEpochs=10000; targetRMSE=0.0001; 
lossHistSmooth=zeros(maxEpochs,1);   
gradEff_prev=zeros(3,1); havePrevGrad=false;

% --- TRAINING LOOP ---
fprintf('Starting NN Training...\n');
for epoch = 1:maxEpochs
    z1=W1*x_in+b1; a1=tanh(z1);
    z2=W2*a1+b2;   a2=tanh(z2);
    z3=W3*a2+b3;   gains=exp(z3);            
    Kp_nn=gains(1); Ki_nn=gains(2); Kd_nn=gains(3);

    assignin('base','Kp',Kp_nn); assignin('base','Ki',Ki_nn); assignin('base','Kd',Kd_nn);
    try
        simOut = sim('TF', 'ReturnWorkspaceOutputs', 'on', 'Timeout', 10);
        L0 = simOut.RMSE.Data(end);
        if isnan(L0), L0=1e3; end
    catch
        L0=1e3;
    end

    if epoch==1, Lsmooth=L0; else, Lsmooth=0.1*lossHistSmooth(epoch-1)+0.9*L0; end
    lossHistSmooth(epoch) = Lsmooth;

    if Lsmooth < targetRMSE, break; end

    % Gradients
    eps_fd=5e-2; grad_g=zeros(3,1);
    for j=1:3
        g2=gains; g2(j)=g2(j)+eps_fd; 
        assignin('base','Kp',g2(1)); assignin('base','Ki',g2(2)); assignin('base','Kd',g2(3));
        try, sP=sim('TF','ReturnWorkspaceOutputs','on','Timeout',10); Lp=sP.RMSE.Data(end); catch, Lp=1e3; end
        grad_g(j)=(Lp-L0)/eps_fd;
    end
    
    assignin('base','Kp',Kp_nn); assignin('base','Ki',Ki_nn); assignin('base','Kd',Kd_nn);

    if ~havePrevGrad, gradEff=grad_g; havePrevGrad=true;
    else, gradEff=lambdaFF*gradEff_prev+(1-lambdaFF)*grad_g; end
    gradEff_prev=gradEff;

    dL_dz3=gradEff.*gains; dL_dW3=dL_dz3*a2.'; dL_db3=dL_dz3;                    
    dL_da2=W3.'*dL_dz3; da2_dz2=1-a2.^2; dL_dz2=dL_da2.*da2_dz2;
    dL_dW2=dL_dz2*a1.'; dL_db2=dL_dz2;                    
    dL_da1=W2.'*dL_dz2; da1_dz1=1-a1.^2; dL_dz1=dL_da1.*da1_dz1;        
    dL_dW1=dL_dz1*x_in.'; dL_db1=dL_dz1;                    

    W3=W3-lr*dL_dW3; b3=b3-lr*dL_db3;
    W2=W2-lr*dL_dW2; b2=b2-lr*dL_db2;
    W1=W1-lr*dL_dW1; b1=b1-lr*dL_db1;
    
    if mod(epoch, 100)==0 || epoch==1, fprintf('Epoch %3d | RMSE: %.6f\n', epoch, Lsmooth); end
end
fprintf('NN Optimized Gains: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_nn, Ki_nn, Kd_nn);


%% ===== 2. PHASE 2: COMPARATIVE VALIDATION =====
fprintf('\n--- PHASE 2: VALIDATION (Passive vs RL vs NN) ---\n');

datasets = {
    'Hualien.mat', 'Hualien';
    'Elcentro.mat','El Centro'
};

for d_idx = 1:size(datasets, 1)
    fName = datasets{d_idx, 1};
    dispName = datasets{d_idx, 2};
    
    fprintf('\n>>> Processing Dataset: %s (%s) <<<\n', fName, dispName);
    
    try
        % 1. Load Data
        rawData = load(fName);
        fieldNames = fieldnames(rawData);
        found = false;
        for k=1:length(fieldNames)
            var = rawData.(fieldNames{k});
            if isnumeric(var) && (size(var,1)>10 || size(var,2)>10)
                EQ_val = var;
                found = true;
                break;
            end
        end
        if ~found, error('Could not find earthquake data in %s', fName); end
        
        if size(EQ_val,1) < size(EQ_val,2), EQ_val = EQ_val'; end
        EQ_val(:,1) = EQ_val(:,1) - EQ_val(1,1);
        
        T_val = EQ_val(end,1);
        dt_real = EQ_val(2,1) - EQ_val(1,1);
        fprintf('   Duration: %.2f s | Time Step: %.4f s\n', T_val, dt_real);

        % 2. Update Simulink
        assignin('base', 'EQ', EQ_val); 
        set_param('TF', 'StopTime', num2str(T_val));

        % 3. Run Simulations
        
        % A. Passive
        fprintf('   1. Simulating Passive...\n');
        [x1, s1, c1, t1, rmse1] = run_sim_full(0,0,0);
        fprintf('      -> RMSE: %.5f\n', rmse1);

        % B. RL (From .txt file)
        fprintf('   2. Simulating RL Optimized...\n');
        [x2, s2, c2, t2, rmse2] = run_sim_full(Kp_rl, Ki_rl, Kd_rl);
        fprintf('      -> RMSE: %.5f\n', rmse2);

        % C. NN (From Phase 1)
        fprintf('   3. Simulating NN Optimized...\n');
        [x3, s3, c3, t3, rmse3] = run_sim_full(Kp_nn, Ki_nn, Kd_nn);
        fprintf('      -> RMSE: %.5f\n', rmse3);

        % 4. Animate with RMSE labels
        videoName = sprintf('Video_Comparison_%s.mp4', dispName);
        fprintf('   Generating Video: %s ...\n', videoName);
        
        animateCombined(x1, s1, c1, t1, x2, s2, c2, t2, x3, s3, c3, t3, ...
            [dispName ' Response'], EQ_val(:,1), EQ_val(:,2), 0.05, videoName, ...
            rmse1, rmse2, rmse3);
        
    catch ME
        fprintf('   ERROR processing %s: %s\n', dispName, ME.message);
    end
end

fprintf('\n✅ All Simulations Complete!\n');


%% ===== HELPER FUNCTIONS =====
function [x, s, c, t, rmse] = run_sim_full(Kp, Ki, Kd)
    assignin('base','Kp',Kp); assignin('base','Ki',Ki); assignin('base','Kd',Kd);
    
    try
        simOut = sim('TF', 'ReturnWorkspaceOutputs', 'on');
    catch
        error('Simulation Failed. Check TF.slx configuration.');
    end

    % Extract RMSE
    try
        rmse = simOut.RMSE.Data(end);
        if isnan(rmse), rmse = 9999; end
    catch
        rmse = -1;
    end

    scale = 5; 
    getD = @(x) extract_data(x);
    x = getD(simOut.yout) / scale; 
    c = getD(simOut.control) / scale; 
    s = getD(simOut.seismic) / scale; 
    
    if isa(simOut.yout, 'double'), t = simOut.tout;
    else, t = simOut.yout.Time; end
end

function d = extract_data(obj)
    if isa(obj, 'double'), d = obj;
    elseif isa(obj, 'struct'), d = obj.Data;
    elseif isa(obj, 'timeseries'), d = obj.Data;
    else, d = obj.get(1).Values.Data; end
end

function animateCombined(x1, s1, c1, t1, x2, s2, c2, t2, x3, s3, c3, t3, eqName, eqTime, eqData, dt, vidName, rmse1, rmse2, rmse3)
    
    t_end = eqTime(end);
    t_anim = 0:dt:t_end;
    Nframe = numel(t_anim);

    allData = [s1; s2; s3; c1; c2; c3; x1; x2; x3];
    Xmax = max(abs(allData)) * 1.5; 
    if Xmax < 0.3, Xmax = 0.3; end
    gap = 2*Xmax + 0.3*Xmax;
    offs = [-gap, 0, gap];
    
    % UPDATED LABELS WITH RMSE
    names = {
        sprintf('Passive\n(RMSE: %.4f)', rmse1), ...
        sprintf('RL Optimized\n(RMSE: %.4f)', rmse2), ...
        sprintf('NN Optimized\n(RMSE: %.4f)', rmse3)
    };
    
    y_g = 0; y_b = 0.09; y_i = 0.17; y_m = 0.4; y_label = -0.15;

    fig = figure('Color','w','Position',[100 100 1200 600]);
    mainAx = axes(fig,'Position',[0.1 0.35 0.85 0.6]);
    hold(mainAx,'on'); axis(mainAx,'off'); axis(mainAx,'equal');
    eqAx   = axes(fig,'Position',[0.1 0.15 0.85 0.20]);

    % Plot EQ Graph
    plot(eqAx, eqTime, eqData, 'k'); hold(eqAx,'on'); grid(eqAx,'on');
    limitY = max(abs(eqData))*1.1; if limitY==0, limitY=1; end
    ylim(eqAx, [-limitY, limitY]); xlim(eqAx, [0 t_end]);
    eqLine = plot(eqAx, [0 0], [-limitY limitY], 'r--','LineWidth',2); 
    title(eqAx, eqName, 'FontSize',12, 'FontWeight','bold'); 
    xlabel(eqAx,'Time (s)'); ylabel(eqAx,'Acc (m/s^2)');

    % Set limits
    x_min = min(offs) - Xmax;  x_max = max(offs) + Xmax;
    y_min = y_g - 0.3;         y_max = y_m + 0.3;
    xlim(mainAx, [x_min, x_max]); ylim(mainAx, [y_min, y_max]);
    set(mainAx,'XLimMode','manual','YLimMode','manual');

    % Video Writer
    if nargin >= 17 && ~isempty(vidName)
        v = VideoWriter(vidName, 'MPEG-4');
        v.FrameRate = 20; 
        v.Quality = 95;
        open(v);
        recordVideo = true;
    else
        recordVideo = false;
    end

    for k = 1:Nframe
        t_now = t_anim(k);
        cla(mainAx); hold(mainAx,'on');

        % Static Lines & Labels
        for i=1:3
            plot(mainAx, [offs(i) offs(i)], [y_g-0.05, y_m+0.12], 'r:', 'LineWidth',1);
            text(mainAx, offs(i), y_label, names{i}, ...
                'HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
        end

        % Dynamic Drawing
        for i=1:3
            if i==1,     xx=x1; ss=s1; cc=c1; tt=t1;
            elseif i==2, xx=x2; ss=s2; cc=c2; tt=t2;
            else,        xx=x3; ss=s3; cc=c3; tt=t3; end

            idx = find(tt >= t_now, 1);
            if isempty(idx), idx = numel(tt); end

            xv = xx(idx); gv = ss(idx); cv = cc(idx); xo = offs(i);
            
            % Draw Components
            rectangle(mainAx,'Position',[xo+gv-Xmax/3, y_g, Xmax/1.5, 0.04],'FaceColor',[0.4 0.4 0.4],'EdgeColor','k','LineWidth',2);
            rectangle(mainAx,'Position',[xo+cv-0.2, y_b, 0.4, 0.06],'FaceColor',[0.8 0.8 0.9],'EdgeColor','k','LineWidth',2);
            rectangle(mainAx,'Position',[xo+cv-0.2, y_i, 0.4, 0.08],'FaceColor',[0.9 0.9 0.6],'EdgeColor','k','LineWidth',1.5);
            plot(mainAx, [xo+gv, xo+cv], [y_g+0.04, y_b], 'k-','LineWidth',4);
            
            % Springs
            springB = y_i + 0.08; springT = y_m;
            lb = xo + cv - 0.4/3; lm = xo + xv - 0.2/3;
            rb = xo + cv + 0.4/3; rm = xo + xv + 0.2/3;
            plot(mainAx, [lb, lm], [springB, springT], 'b--', 'LineWidth',2);
            plot(mainAx, [rb, rm], [springB, springT], 'b--', 'LineWidth',2);

            % Mass
            rectangle(mainAx,'Position',[xo+xv-0.1, y_m, 0.2, 0.12],'Curvature',0.1,'FaceColor','w','EdgeColor','k','LineWidth',2);
        end

        set(eqLine, 'XData', [t_now t_now]);
        
        if ishandle(findobj(eqAx,'Tag','timertext'))
            delete(findobj(eqAx,'Tag','timertext'));
        end
        xlim_eq = xlim(eqAx); ylim_eq = ylim(eqAx);
        text(eqAx, xlim_eq(2)-0.01*(xlim_eq(2)-xlim_eq(1)), ...
                 ylim_eq(2)-0.08*(ylim_eq(2)-ylim_eq(1)), ...
             sprintf('Time = %.2f s', t_now), ...
             'HorizontalAlignment','right','FontSize',14,'FontWeight','bold','Tag','timertext');

        drawnow;
        
        if recordVideo
            frame = getframe(fig);
            writeVideo(v, frame);
        end
    end
    
    if recordVideo
        close(v);
        fprintf('Video Saved: %s\n', vidName);
    end
end