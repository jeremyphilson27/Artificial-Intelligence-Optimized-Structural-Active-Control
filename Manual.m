%% Manual_PID_Control.m
clc; clear; close all;

%% ===== 1. USER CONFIGURATION =====
fprintf('--- CONFIGURATION ---\n');

% 1. Set Your Manual Gains Here
Kp_manual = 0.9973; 
Ki_manual = 0.9941; 
Kd_manual = 1.0080;

fprintf('Manual Gains Set: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_manual, Ki_manual, Kd_manual);

% 2. List of Datasets to Test
datasets = {
    'EQ1.mat',     'EQ1 (Training Data)'; % You can include the original training file here
    'Hualien.mat', 'Hualien';
    'Elcentro.mat','El Centro'
};

% Load Simulink System
if ~bdIsLoaded('TF'), load_system('TF'); end


%% ===== 2. SIMULATION LOOP =====
fprintf('\n--- STARTING SIMULATIONS ---\n');

for d_idx = 1:size(datasets, 1)
    fName = datasets{d_idx, 1};
    dispName = datasets{d_idx, 2};
    
    fprintf('\n>>> Processing Dataset: %s (%s) <<<\n', fName, dispName);
    
    try
        % --- 1. Load Data Robustly ---
        if ~exist(fName, 'file')
            fprintf('   [!] Warning: File %s not found. Skipping.\n', fName);
            continue;
        end
        
        rawData = load(fName);
        fieldNames = fieldnames(rawData);
        
        % Find EQ variable
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
        
        % Format Data
        if size(EQ_val,1) < size(EQ_val,2), EQ_val = EQ_val'; end
        EQ_val(:,1) = EQ_val(:,1) - EQ_val(1,1); % Zero start time
        
        T_val = EQ_val(end,1);
        dt_real = EQ_val(2,1) - EQ_val(1,1);
        fprintf('   Duration: %.2f s | Time Step: %.4f s\n', T_val, dt_real);

        % --- 2. Update Simulink Workspace ---
        assignin('base', 'EQ', EQ_val); 
        set_param('TF', 'StopTime', num2str(T_val));

        % --- 3. Run Simulations ---
        
        % A. Passive (Kp=0, Ki=0, Kd=0)
        fprintf('   Running Passive Simulation...\n');
        [x1, s1, c1, t1, rmse1] = run_sim_full(0, 0, 0);
        fprintf('      -> Passive RMSE: %.6f\n', rmse1);

        % B. Manual Control
        fprintf('   Running Manual Control (Kp=%.1f)...\n', Kp_manual);
        [x2, s2, c2, t2, rmse2] = run_sim_full(Kp_manual, Ki_manual, Kd_manual);
        fprintf('      -> Manual  RMSE: %.6f\n', rmse2);
        
        % Display Improvement
        imp = ((rmse1 - rmse2) / rmse1) * 100;
        fprintf('      -> Improvement:  %.2f%%\n', imp);

        % --- 4. Animate (2 Figures Only) ---
        videoName = sprintf('Video_Manual_%s.mp4', dispName);
        fprintf('   Generating Video: %s ...\n', videoName);
        
        animateDual(x1, s1, c1, t1, x2, s2, c2, t2, ...
            [dispName ' Response'], EQ_val(:,1), EQ_val(:,2), 0.05, videoName, ...
            rmse1, rmse2);
        
    catch ME
        fprintf('   ERROR processing %s: %s\n', dispName, ME.message);
    end
end

fprintf('\n✅ All Simulations Complete!\n');


%% ===== HELPER FUNCTIONS =====

function [x, s, c, t, rmse] = run_sim_full(Kp, Ki, Kd)
    % Assign Gains to Workspace
    assignin('base','Kp',Kp); 
    assignin('base','Ki',Ki); 
    assignin('base','Kd',Kd);
    
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
        rmse = -1; % Indicates RMSE block missing
    end

    scale = 5; % Visual scaling
    
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

function animateDual(x1, s1, c1, t1, x2, s2, c2, t2, eqName, eqTime, eqData, dt, vidName, rmse1, rmse2)
    % UPDATED FOR 2 FIGURES (Passive vs Manual)
    
    t_end = eqTime(end);
    t_anim = 0:dt:t_end;
    Nframe = numel(t_anim);

    allData = [s1; s2; c1; c2; x1; x2];
    Xmax = max(abs(allData)) * 1.5; 
    if Xmax < 0.3, Xmax = 0.3; end
    
    % Setup positions for 2 figures
    gap = 2*Xmax + 0.5*Xmax;
    offs = [-gap/2, gap/2]; % Left and Right
    names = {sprintf('Passive\n(RMSE: %.4f)', rmse1), ...
             sprintf('Manual\n(RMSE: %.4f)', rmse2)};
    
    y_g = 0; y_b = 0.09; y_i = 0.17; y_m = 0.4; y_label = -0.15;

    fig = figure('Color','w','Position',[100 100 1000 600]);
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
    if nargin >= 13 && ~isempty(vidName)
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
        for i=1:2
            plot(mainAx, [offs(i) offs(i)], [y_g-0.05, y_m+0.12], 'r:', 'LineWidth',1);
            text(mainAx, offs(i), y_label, names{i}, ...
                'HorizontalAlignment','center','FontSize',12,'FontWeight','bold');
        end

        % Dynamic Drawing
        for i=1:2
            if i==1, xx=x1; ss=s1; cc=c1; tt=t1;
            else,    xx=x2; ss=s2; cc=c2; tt=t2; end

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
        
        % Timer
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