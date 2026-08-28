function DualLoopPID_App_Enhanced()
% DUALLOoppid_APP_ENHANCED 雙環 PID 姿態控制互動模擬器（增強版）
% 
% 基於 Crazyflie & RotorPy 動力學模型補強：
%   1. 馬達動力學（一階延遲）: H_motor(s) = K_motor / (tau_m*s + 1)
%   2. 空氣動力阻尼（旋轉阻力矩）: G_body(s) = 1 / (I*s + gamma)
%   3. 執行鏈路 DC 增益 K_motor（含 Mixer+ESC+馬達+螺旋槳）
%
% 當 tau_m = 0 且 gamma = 0 且 K_motor = 1 時，自動還原為原始簡化模型。
%
% 參照文獻：
%   [1] Gräfe et al., "How to Model Your Crazyflie Brushless", arXiv 2025
%   [2] Folk et al., "RotorPy: A Python-based Multirotor Simulator", ICRA 2023
%   [3] ArduPilot System Identification Documentation
%   [4] Snider, "CrazyFlie 2.1 Cascaded PID/PD Control System", 2025
%
% 需要：MATLAB Control System Toolbox
% 作者：基於原 DualLoopPID_App 增強

    %% ==================== 清除與初始化 ====================
    close all; clc;

    %% ==================== 預設參數 ====================
    P = struct(...
        'Kp_rate', 0.15, ...   % 內環比例增益 [Nm/(rad/s)]
        'Ki_rate', 0.05, ...   % 內環積分增益 [Nm/(rad/s)/s]
        'Kd_rate', 0.005, ...  % 內環微分增益 [Nm/(rad/s^2)]
        'Kp_att',  2.0, ...    % 外環比例增益 [(rad/s)/rad]
        'Ki_att',  0.0, ...    % 外環積分增益 [(rad/s)/rad/s]
        'Kd_att',  0.0, ...    % 外環微分增益 [(rad/s)/(rad/s)]
        'I',       2.5e-5, ... % 機體轉動慣量 [kg*m^2]
        'tau_m',   0.05, ...   % 馬達+ESC 一階時間常數 [s]
        'gamma',   1e-4, ...   % 旋轉空氣阻尼係數 [N*m*s/rad]
        'K_motor', 1.0 ...     % 執行鏈路 DC 增益 [-]
    );

    % 參數名稱、顯示標籤、提示文字
    ParamNames = {'Kp_rate','Ki_rate','Kd_rate','Kp_att','Ki_att','Kd_att',...
                  'I','tau_m','gamma','K_motor'};
    ParamLabels = {'Kp (rate)', 'Ki (rate)', 'Kd (rate)', ...
                   'Kp (att)', 'Ki (att)', 'Kd (att)', ...
                   'I [kg*m^2]', '\tau_m [s]', '\gamma [N*m*s/rad]', 'K_{motor} [-]'};
    ParamTips = {
        '內環比例增益：角速率誤差→力矩指令。增加則反應更快，但過大會放大雜訊';
        '內環積分增益：消除穩態角速率誤差（如電池壓降造成的偏置）';
        '內環微分增益：等效虛擬增加轉動慣量，抑制過衝。實機需搭配濾波器';
        '外環比例增益：姿態角誤差→目標角速率。決定「多積極地回到目標角度」';
        '外環積分增益：消除姿態穩態誤差。會提高系統階數，不建議實機使用';
        '外環微分增益：理論上可加阻尼，但會放大雜訊，一般飛控外環不建議使用';
        '機體轉動慣量（含馬達轉子與螺旋槳）。模擬換電池/換機架的影響';
        '馬達+ESC 一階時間常數。Crazyflie 2.1(有刷)≈0.10s, Brushless≈0.05s, 中型機≈0.08~0.15s';
        '旋轉空氣阻尼係數。機體旋轉時的空氣阻力矩 M^D = -gamma*omega';
        '執行鏈路 DC 增益（Mixer x ESC x 馬達 x 螺旋槳的整體增益）。標稱值設為 1.0'
    };

    % 參數滑桿範圍 [min, max]
    Limits = struct();
    Limits.Kp_rate = [0, 1.0];
    Limits.Ki_rate = [0, 0.5];
    Limits.Kd_rate = [0, 0.05];
    Limits.Kp_att  = [0, 10.0];
    Limits.Ki_att  = [0, 2.0];
    Limits.Kd_att  = [0, 0.5];
    Limits.I       = [1e-6, 1e-3];
    Limits.tau_m   = [0, 0.5];
    Limits.gamma   = [0, 1e-2];
    Limits.K_motor = [0.1, 5.0];

    %% ==================== 建立 GUI ====================
    fig = uifigure('Name', '雙環 PID 姿態控制互動模擬器（增強版 - Crazyflie/RotorPy 動力學）', ...
                   'Position', [80 50 1450 950], ...
                   'Color', [0.96 0.96 0.96], ...
                   'CloseRequestFcn', @(~,~) delete(fig));

    % --- 左側參數面板 ---
    leftPanel = uipanel(fig, 'Title', '參數調整面板', ...
                        'Position', [10 10 340 930], ...
                        'FontSize', 13, 'FontWeight', 'bold', ...
                        'BackgroundColor', [0.98 0.98 0.98]);

    nParams = numel(ParamNames);
    sliders = gobjects(nParams, 1);
    edts    = gobjects(nParams, 1);

    yStart = 880;
    yStep  = 78;

    for i = 1:nParams
        pname = ParamNames{i};

        uilabel(leftPanel, 'Text', ParamLabels{i}, ...
                'Position', [15 yStart-(i-1)*yStep 160 24], ...
                'FontSize', 11, 'FontWeight', 'bold', ...
                'Tooltip', ParamTips{i});

        sliders(i) = uislider(leftPanel, ...
                              'Position', [15 yStart-(i-1)*yStep-32 210 3], ...
                              'Limits', Limits.(pname), ...
                              'Value', P.(pname), ...
                              'Tag', pname, ...
                              'MajorTicks', linspace(Limits.(pname)(1), Limits.(pname)(2), 5));
        sliders(i).ValueChangedFcn = @(src,~) sliderCallback(src);

        edts(i) = uieditfield(leftPanel, 'numeric', ...
                              'Position', [235 yStart-(i-1)*yStep-38 90 32], ...
                              'Value', P.(pname), ...
                              'Limits', Limits.(pname), ...
                              'Tag', pname, ...
                              'FontSize', 11);
        edts(i).ValueChangedFcn = @(src,~) editCallback(src);
    end

    % 快捷按鈕區
    btnY = 80;
    uibutton(leftPanel, 'push', ...
             'Text', 'Reset 預設值', ...
             'Position', [15 btnY+40 310 36], ...
             'FontSize', 12, 'FontWeight', 'bold', ...
             'BackgroundColor', [0.9 0.9 1.0], ...
             'ButtonPushedFcn', @(~,~) resetAll());

    uibutton(leftPanel, 'push', ...
             'Text', '還原「簡化模型」(tau_m=0, gamma=0)', ...
             'Position', [15 btnY 310 36], ...
             'FontSize', 11, ...
             'BackgroundColor', [1.0 0.95 0.9], ...
             'ButtonPushedFcn', @(~,~) setSimpleModel());

    uibutton(leftPanel, 'push', ...
             'Text', '載入 Crazyflie 2.1 參數', ...
             'Position', [15 btnY-40 150 32], ...
             'FontSize', 10, ...
             'ButtonPushedFcn', @(~,~) loadCrazyflie21());

    uibutton(leftPanel, 'push', ...
             'Text', '載入 Crazyflie Brushless', ...
             'Position', [175 btnY-40 150 32], ...
             'FontSize', 10, ...
             'ButtonPushedFcn', @(~,~) loadCrazyflieBL());

    % --- 右側 TabGroup ---
    tabGroup = uitabgroup(fig, 'Position', [370 130 1060 810]);

    tabStep = uitab(tabGroup, 'Title', 'Tab1 階躍響應', 'BackgroundColor', 'w');
    tabBode = uitab(tabGroup, 'Title', 'Tab2 Bode 圖', 'BackgroundColor', 'w');
    tabRL   = uitab(tabGroup, 'Title', 'Tab3 Root Locus', 'BackgroundColor', 'w');
    tabPZ   = uitab(tabGroup, 'Title', 'Tab4 Pole-Zero Map', 'BackgroundColor', 'w');

    axStep = axes('Parent', tabStep, 'Position', [0.08 0.12 0.88 0.82]);
    axBodeMag = axes('Parent', tabBode, 'Position', [0.08 0.55 0.88 0.40]);
    axBodePhase = axes('Parent', tabBode, 'Position', [0.08 0.08 0.88 0.40]);
    axRL = axes('Parent', tabRL, 'Position', [0.10 0.18 0.85 0.72]);
    axPZ = axes('Parent', tabPZ, 'Position', [0.08 0.12 0.88 0.82]);

    % Root Locus 控制面板
    rlCtrl = uipanel(tabRL, 'Title', 'Root Locus 掃描設定', ...
                     'Position', [20 10 500 70], ...
                     'FontSize', 11, 'BackgroundColor', [0.98 0.98 1.0]);
    uilabel(rlCtrl, 'Text', '掃描參數:', 'Position', [10 20 80 22], 'FontSize', 11);
    rlDrop = uidropdown(rlCtrl, 'Items', ParamLabels, ...
                        'Position', [95 18 200 26], ...
                        'Value', ParamLabels{4}, ...
                        'FontSize', 11);
    rlDrop.ValueChangedFcn = @(~,~) updateAll();

    % --- 底部資訊面板 ---
    infoPanel = uipanel(fig, 'Title', '系統資訊與穩定性分析', ...
                        'Position', [370 10 1060 110], ...
                        'FontSize', 12, 'FontWeight', 'bold', ...
                        'BackgroundColor', [0.95 0.98 1.0]);
    infoLbl = uilabel(infoPanel, 'Text', '初始化中...', ...
                      'Position', [10 5 1040 95], ...
                      'FontSize', 10.5, ...
                      'VerticalAlignment', 'top', ...
                      'WordWrap', 'on');

    %% ==================== 初始繪圖 ====================
    updateAll();

    %% ==================== 回調函數 ====================
    function sliderCallback(src)
        pname = src.Tag;
        P.(pname) = src.Value;
        idx = find(strcmp(ParamNames, pname));
        edts(idx).Value = src.Value;
        updateAll();
    end

    function editCallback(src)
        pname = src.Tag;
        P.(pname) = src.Value;
        idx = find(strcmp(ParamNames, pname));
        sliders(idx).Value = src.Value;
        updateAll();
    end

    function resetAll()
        P0 = struct('Kp_rate',0.15,'Ki_rate',0.05,'Kd_rate',0.005,...
                    'Kp_att',2.0,'Ki_att',0.0,'Kd_att',0.0,...
                    'I',2.5e-5,'tau_m',0.05,'gamma',1e-4,'K_motor',1.0);
        fn = fieldnames(P0);
        for k = 1:numel(fn)
            P.(fn{k}) = P0.(fn{k});
            idx = find(strcmp(ParamNames, fn{k}));
            sliders(idx).Value = P0.(fn{k});
            edts(idx).Value = P0.(fn{k});
        end
        updateAll();
    end

    function setSimpleModel()
        P.tau_m = 0; P.gamma = 0; P.K_motor = 1.0;
        idx = [find(strcmp(ParamNames,'tau_m')), ...
               find(strcmp(ParamNames,'gamma')), ...
               find(strcmp(ParamNames,'K_motor'))];
        sliders(idx(1)).Value = 0; edts(idx(1)).Value = 0;
        sliders(idx(2)).Value = 0; edts(idx(2)).Value = 0;
        sliders(idx(3)).Value = 1; edts(idx(3)).Value = 1;
        updateAll();
    end

    function loadCrazyflie21()
        P.I = 1.6e-5; P.tau_m = 0.10; P.gamma = 1e-5; P.K_motor = 1.0;
        P.Kp_rate = 0.12; P.Ki_rate = 0.04; P.Kd_rate = 0.003;
        P.Kp_att = 1.5; P.Ki_att = 0; P.Kd_att = 0;
        syncUI();
        updateAll();
    end

    function loadCrazyflieBL()
        P.I = 3.2e-5; P.tau_m = 0.05; P.gamma = 1e-5; P.K_motor = 1.0;
        P.Kp_rate = 0.18; P.Ki_rate = 0.06; P.Kd_rate = 0.004;
        P.Kp_att = 2.5; P.Ki_att = 0; P.Kd_att = 0;
        syncUI();
        updateAll();
    end

    function syncUI()
        for i = 1:nParams
            pname = ParamNames{i};
            sliders(i).Value = P.(pname);
            edts(i).Value = P.(pname);
        end
    end

    function updateAll()
        [Tin, Ttotal, info] = buildSystems(P);
        drawStep(Tin, Ttotal);
        drawBode(Ttotal);
        drawRootLocus();
        drawPoleZero(Ttotal);
        updateInfo(info);
    end

    %% ==================== 核心：建立 Transfer Function ====================
    function [Tin, Ttotal, info] = buildSystems(P)
        % 馬達動力學 H_motor(s) = K_motor / (tau_m*s + 1)
        if P.tau_m > 1e-10
            H_motor = tf(P.K_motor, [P.tau_m, 1]);
        else
            H_motor = tf(P.K_motor, 1);
        end

        % 機體轉動動力學（含空氣阻尼）G_body(s) = 1 / (I*s + gamma)
        % 物理意義：I*s*omega + gamma*omega = tau  =>  omega/tau = 1/(I*s + gamma)
        if P.gamma > 1e-12
            G_body = tf(1, [P.I, P.gamma]);
        else
            G_body = tf(1, [P.I, 0]);  % 還原為 1/(I*s)
        end

        % 內環控制器 Cin(s) = Kd*s + Kp + Ki/s = (Kd*s^2 + Kp*s + Ki) / s
        Cin = tf([P.Kd_rate, P.Kp_rate, P.Ki_rate], [1, 0]);

        % 內環開環：Cin -> H_motor -> G_body，回授 omega
        G_omega_ol = Cin * H_motor * G_body;

        % 內環閉環 Tin(s)
        Tin = feedback(G_omega_ol, 1);

        % 外環控制器 Cout(s) = Kd_att*s + Kp_att + Ki_att/s
        Cout = tf([P.Kd_att, P.Kp_att, P.Ki_att], [1, 0]);

        % 外環開環：Cout -> Tin -> 1/s，回授 theta
        G_theta_ol = Cout * Tin * tf(1, [1, 0]);

        % 外環閉環 Ttotal(s)
        Ttotal = feedback(G_theta_ol, 1);

        % 計算系統資訊
        info = computeInfo(Tin, Ttotal, P, G_omega_ol, G_theta_ol);
    end

    function info = computeInfo(Tin, Ttotal, P, G_omega_ol, G_theta_ol)
        info = struct();

        % --- 內環資訊 ---
        try
            sTin = stepinfo(Tin, 'SettlingTimeThreshold', 0.02);
            info.tin_bw = bandwidth(Tin);
            info.tin_os = sTin.Overshoot;
            info.tin_st = sTin.SettlingTime;
            info.tin_stable = isstable(Tin);

            % 相位裕度與增益裕度
            [Gm, Pm, Wcg, Wcp] = margin(G_omega_ol);
            info.tin_Gm_dB = 20*log10(Gm); info.tin_Pm_deg = Pm;
            info.tin_Wcg = Wcg; info.tin_Wcp = Wcp;
        catch
            info.tin_bw = NaN; info.tin_os = NaN; info.tin_st = NaN;
            info.tin_stable = false;
            info.tin_Gm_dB = NaN; info.tin_Pm_deg = NaN;
            info.tin_Wcg = NaN; info.tin_Wcp = NaN;
        end

        % --- 外環資訊 ---
        try
            sTtot = stepinfo(Ttotal, 'SettlingTimeThreshold', 0.02);
            info.ttot_bw = bandwidth(Ttotal);
            info.ttot_os = sTtot.Overshoot;
            info.ttot_st = sTtot.SettlingTime;
            info.ttot_stable = isstable(Ttotal);

            [Gm2, Pm2, Wcg2, Wcp2] = margin(G_theta_ol);
            info.ttot_Gm_dB = 20*log10(Gm2); info.ttot_Pm_deg = Pm2;
        catch
            info.ttot_bw = NaN; info.ttot_os = NaN; info.ttot_st = NaN;
            info.ttot_stable = false;
            info.ttot_Gm_dB = NaN; info.ttot_Pm_deg = NaN;
        end

        % --- 頻寬比 ---
        if ~isnan(info.tin_bw) && ~isnan(info.ttot_bw) && info.ttot_bw > 0
            info.bw_ratio = info.tin_bw / info.ttot_bw;
        else
            info.bw_ratio = NaN;
        end

        % --- 馬達動力學資訊 ---
        if P.tau_m > 1e-10
            info.motor_bw_hz = 1/(2*pi*P.tau_m);
            info.motor_pole = -1/P.tau_m;
        else
            info.motor_bw_hz = Inf;
            info.motor_pole = -Inf;
        end

        % --- 機體阻尼資訊 ---
        if P.gamma > 1e-12
            info.body_pole = -P.gamma / P.I;
            info.body_time_const = P.I / P.gamma;
        else
            info.body_pole = 0;
            info.body_time_const = Inf;
        end

        % --- 極點列表 ---
        try
            info.tin_poles = pole(Tin);
            info.ttot_poles = pole(Ttotal);
        catch
            info.tin_poles = [];
            info.ttot_poles = [];
        end

        % --- 特徵方程係數（內環閉環）---
        % tau_m*I*s^3 + (tau_m*gamma + I + K_m*Kd)*s^2 + (gamma + K_m*Kp)*s + K_m*Ki = 0
        info.char_poly = [P.tau_m*P.I, ...
                          P.tau_m*P.gamma + P.I + P.K_motor*P.Kd_rate, ...
                          P.gamma + P.K_motor*P.Kp_rate, ...
                          P.K_motor*P.Ki_rate];
    end

    %% ==================== 繪圖函數 ====================
    function drawStep(Tin, Ttotal)
        axes(axStep); cla; hold on; grid on;
        t = linspace(0, 5, 2000);
        try
            [y1, t1] = step(Tin, t);
            [y2, t2] = step(Ttotal, t);

            plot(t1, y1, 'b-', 'LineWidth', 1.8, 'DisplayName', '內環 T_{in}(s) - 角速率響應');
            plot(t2, y2, 'r-', 'LineWidth', 1.8, 'DisplayName', '外環 T_{total}(s) - 姿態角響應');

            % 標註穩態值
            if ~isempty(t1) && ~isempty(y1)
                plot(t1(end), y1(end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
            end
            if ~isempty(t2) && ~isempty(y2)
                plot(t2(end), y2(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            end

            legend('Location', 'best', 'FontSize', 10);
            xlabel('Time [s]', 'FontSize', 11); 
            ylabel('Normalized Response', 'FontSize', 11);
            title('Step Response: Inner Loop (Rate) vs Outer Loop (Attitude)', 'FontSize', 12);

            % 參考線
            plot(xlim, [1 1], 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
        catch ME
            text(0.5, 0.5, ['無法計算階躍響應：' ME.message], ...
                 'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
        end
        hold off;
    end

    function drawBode(Ttotal)
        cla(axBodeMag); cla(axBodePhase);
        try
            [mag, phase, w] = bode(Ttotal);
            mag = squeeze(mag); phase = squeeze(phase);
            f = w/(2*pi);

            semilogx(axBodeMag, f, 20*log10(mag), 'b-', 'LineWidth', 1.5);
            grid(axBodeMag, 'on');
            ylabel(axBodeMag, 'Magnitude [dB]', 'FontSize', 11);
            title(axBodeMag, 'Bode Plot: T_{total}(s) Magnitude', 'FontSize', 12);
            xlim(axBodeMag, [0.01, 100]);

            semilogx(axBodePhase, f, phase, 'b-', 'LineWidth', 1.5);
            grid(axBodePhase, 'on');
            xlabel(axBodePhase, 'Frequency [Hz]', 'FontSize', 11);
            ylabel(axBodePhase, 'Phase [deg]', 'FontSize', 11);
            title(axBodePhase, 'Bode Plot: T_{total}(s) Phase', 'FontSize', 12);
            xlim(axBodePhase, [0.01, 100]);
        catch ME
            axes(axBodeMag);
            text(0.5, 0.5, ['無法計算 Bode：' ME.message], ...
                 'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                 'Color', 'r', 'FontSize', 12);
        end
    end

    function drawRootLocus()
        axes(axRL); cla; hold on; grid on;

        selLabel = rlDrop.Value;
        selIdx = find(strcmp(ParamLabels, selLabel));
        selParam = ParamNames{selIdx};

        lim = Limits.(selParam);
        nPts = 100;
        vals = linspace(max(lim(1), 1e-6), lim(2), nPts);

        cmap = jet(nPts);
        allReal = []; allImag = [];

        for k = 1:nPts
            Pscan = P;
            Pscan.(selParam) = vals(k);
            [~, Tscan, ~] = buildSystems(Pscan);
            try
                p = pole(Tscan);
                allReal = [allReal; real(p)];
                allImag = [allImag; imag(p)];
                scatter(axRL, real(p), imag(p), 18, cmap(k,:), 'filled', ...
                        'MarkerFaceAlpha', 0.6, 'HandleVisibility', 'off');
            catch
            end
        end

        % 標記目前參數對應的極點（紅色五角星）
        try
            [~, Tnow, ~] = buildSystems(P);
            pNow = pole(Tnow);
            scatter(axRL, real(pNow), imag(pNow), 180, 'r', 'p', ...
                    'LineWidth', 2.5, 'DisplayName', '當前極點');
        catch
        end

        % ζ / ωn 格線
        plotZetaWnGrid(axRL);

        xlabel(axRL, 'Real Axis [rad/s]', 'FontSize', 11);
        ylabel(axRL, 'Imaginary Axis [rad/s]', 'FontSize', 11);
        title(axRL, ['Root Locus: 掃描 ' selLabel ' (0 \rightarrow ' sprintf('%.3g', lim(2)) ')'], 'FontSize', 12);

        % 設定對稱的 y 軸範圍
        if ~isempty(allImag)
            mx = max(abs(allImag));
            if mx > 0
                ylim(axRL, [-mx*1.2, mx*1.2]);
            end
        end

        % Colorbar
        colormap(axRL, jet); 
        c = colorbar(axRL, 'Location', 'eastoutside');
        c.Label.String = selLabel;
        caxis(axRL, lim);

        legend(axRL, 'Location', 'best');
        hold off;
    end

    function drawPoleZero(Ttotal)
        axes(axPZ); cla; hold on; grid on;
        try
            p = pole(Ttotal);
            z = zero(Ttotal);
            scatter(axPZ, real(p), imag(p), 120, 'rx', 'LineWidth', 2, 'DisplayName', 'Poles');
            if ~isempty(z)
                scatter(axPZ, real(z), imag(z), 120, 'bo', 'LineWidth', 2, 'DisplayName', 'Zeros');
            end
            legend(axPZ, 'Location', 'best', 'FontSize', 10);
        catch
        end
        plotZetaWnGrid(axPZ);
        xlabel(axPZ, 'Real Axis [rad/s]', 'FontSize', 11);
        ylabel(axPZ, 'Imaginary Axis [rad/s]', 'FontSize', 11);
        title(axPZ, 'Pole-Zero Map: T_{total}(s)', 'FontSize', 12);
        hold off;
    end

    function plotZetaWnGrid(ax)
        % 等阻尼比線 ζ = 0.1 ~ 0.9
        zetaVals = 0.1:0.1:0.9;
        for z = zetaVals
            theta = acos(z);
            t = linspace(0, 80, 200);
            plot(ax, -t.*cos(theta), t.*sin(theta), 'Color', [0.7 0.7 0.7], 'LineStyle', '--', 'LineWidth', 0.6, 'HandleVisibility', 'off');
            plot(ax, -t.*cos(theta), -t.*sin(theta), 'Color', [0.7 0.7 0.7], 'LineStyle', '--', 'LineWidth', 0.6, 'HandleVisibility', 'off');
            % 標註
            rtxt = 8;
            text(ax, -rtxt*cos(theta), rtxt*sin(theta), sprintf('\\zeta=%.1f', z), ...
                 'FontSize', 8, 'Color', [0.4 0.4 0.4], 'HorizontalAlignment', 'center');
        end

        % 等自然頻率線 ωn = 5, 10, 15, ..., 50 rad/s
        wnVals = 5:5:50;
        for w = wnVals
            th = linspace(pi/2, pi, 100);
            plot(ax, w*cos(th), w*sin(th), 'Color', [0.8 0.8 0.8], 'LineStyle', ':', 'LineWidth', 0.5, 'HandleVisibility', 'off');
            plot(ax, w*cos(th), -w*sin(th), 'Color', [0.8 0.8 0.8], 'LineStyle', ':', 'LineWidth', 0.5, 'HandleVisibility', 'off');
            text(ax, w*cos(pi*0.55), w*sin(pi*0.55), sprintf('\\omega_n=%d', w), ...
                 'FontSize', 7, 'Color', [0.5 0.5 0.5], 'HorizontalAlignment', 'center');
        end

        % 虛軸（穩定邊界）
        yl = ylim(ax);
        plot(ax, [0 0], yl, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');

        % 穩定區域著色
        xl = xlim(ax);
        if xl(1) < 0
            patch(ax, [xl(1) 0 0 xl(1)], [yl(1) yl(1) yl(2) yl(2)], ...
                  [0.9 1.0 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end

    function updateInfo(info)
        if info.ttot_stable
            stabStr = '【穩定】'; stabColor = [0 0.5 0];
        else
            stabStr = '【不穩定】'; stabColor = [0.8 0 0];
        end
        if info.tin_stable
            stabIn = '【穩定】';
        else
            stabIn = '【不穩定】';
        end

        % 內環特徵方程文字
        if P.tau_m > 1e-10 || P.gamma > 1e-12
            eqStr = sprintf('內環閉環特徵方程：(%.2e)s^3 + (%.2e)s^2 + (%.2e)s + (%.2e) = 0', ...
                info.char_poly(1), info.char_poly(2), info.char_poly(3), info.char_poly(4));
        else
            eqStr = sprintf('內環閉環特徵方程：(%.2e)s^2 + (%.2e)s + (%.2e) = 0 (簡化模型)', ...
                info.char_poly(2), info.char_poly(3), info.char_poly(4));
        end

        str = sprintf([...
            '<b>【內環 T_{in}(s)】</b> %s | 頻寬: %.2f rad/s | Overshoot: %.1f%% | Settling(2%%): %.3fs | PM: %.1f° | GM: %.1f dB\n' ...
            '<b>【外環 T_{total}(s)】</b> %s | 頻寬: %.2f rad/s | Overshoot: %.1f%% | Settling(2%%): %.3fs | PM: %.1f° | GM: %.1f dB\n' ...
            '<b>【頻寬比】</b> 內環/外環 = %.2f  (建議 ≥ 4~5，否則外環會與內環動態耦合)\n' ...
            '<b>【馬達動力學】</b> 時間常數 \tau_m = %.4f s | 馬達頻寬 = %.1f Hz | 馬達極點 = %.1f rad/s | DC增益 K_{motor} = %.2f\n' ...
            '<b>【機體阻尼】</b> 空氣阻尼 \gamma = %.2e N*m*s/rad | 機體開環極點 = %.1f rad/s | 機體時間常數 = %.3f s\n' ...
            '<b>【極點】</b> 內環: %s | 外環: %s\n' ...
            '<b>【特徵方程】</b> %s'], ...
            stabIn, info.tin_bw, info.tin_os, info.tin_st, info.tin_Pm_deg, info.tin_Gm_dB, ...
            stabStr, info.ttot_bw, info.ttot_os, info.ttot_st, info.ttot_Pm_deg, info.ttot_Gm_dB, ...
            info.bw_ratio, ...
            P.tau_m, info.motor_bw_hz, info.motor_pole, P.K_motor, ...
            P.gamma, info.body_pole, info.body_time_const, ...
            mat2str(info.tin_poles, 3), mat2str(info.ttot_poles, 3), ...
            eqStr);

        infoLbl.Text = str;
        if info.ttot_stable
            infoLbl.FontColor = [0 0.3 0];
        else
            infoLbl.FontColor = [0.7 0 0];
        end
    end
end
