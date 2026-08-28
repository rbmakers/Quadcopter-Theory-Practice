function DualLoopPID_App
% DualLoopPID_App  四軸飛行器雙環(Cascaded) PID 姿態控制互動模擬器
%
%   對應講義：「四軸飛行器雙環 PID 控制傳遞函數推導」(第四週補充)
%
%   架構：
%       外環(姿態環) Cout(s) = Kp_att + Ki_att/s + Kd_att*s
%       內環(角速率環) Cin(s) = Kp_rate + Ki_rate/s + Kd_rate*s
%       被控對象(機身動力學) Gw(s) = 1/(I*s)   (轉矩->角速率)
%                              Gtheta(s) = 1/s   (角速率->角度, 純積分)
%
%   內環閉環：
%       Tin(s) = Cin*Gw / (1 + Cin*Gw)
%              = (Kd_rate*s^2 + Kp_rate*s + Ki_rate) /
%                ((I+Kd_rate)*s^2 + Kp_rate*s + Ki_rate)
%
%   外環(全系統)閉環：
%       Ttotal(s) = Cout*Tin*(1/s) / (1 + Cout*Tin*(1/s))
%
%   使用方式：
%       1. 直接在 MATLAB 執行 >> DualLoopPID_App
%       2. 拖曳左側滑桿即時觀察：
%           - 時域階躍響應 (內環 vs 外環)
%           - Bode 圖 (外環全系統)
%           - Root Locus：可用下拉選單選擇「單一參數掃描」
%             (Kp_att/Ki_att/Kd_att/Kp_rate/Ki_rate/Kd_rate 任一個從 0
%             掃到該滑桿上限，其餘參數固定於目前滑桿值)，並疊加
%             等阻尼比(zeta)/等自然頻率(wn) 格線 (s-plane grid)
%           - Pole-Zero Map (外環閉環，標示目前工作點極點/零點，
%             同樣疊加 zeta/wn 格線)
%       3. 右下資訊欄會顯示穩定性、阻尼比、自然頻率、
%          以及「內外環頻寬分離比」是否符合講義建議(>=4~5倍)
%
%   需求：MATLAB Control System Toolbox
%
%   作者：CURIO 專案 / 零到飛課程教材
% -------------------------------------------------------------------

    %% ---------- 共用資料結構 (nested function 共享 workspace) ----------
    P = struct( ...
        'I',        1.5e-4, ...   % 轉動慣量 (kg*m^2)
        'Kp_rate',  0.08,   ...   % 內環比例增益
        'Ki_rate',  0.50,   ...   % 內環積分增益
        'Kd_rate',  0.0008, ...   % 內環微分增益 (與 I 同量級)
        'Kp_att',   6.0,    ...   % 外環比例增益
        'Ki_att',   0.0,    ...   % 外環積分增益
        'Kd_att',   0.0);         % 外環微分增益

    defaultP = P;   % 供 Reset 使用

    sliderH = struct();
    editH   = struct();
    ax      = struct();
    infoArea = [];
    sweepParam = 'Kp_att';   % Root Locus 分頁目前選定的掃描參數
    sweepDropdown = [];

    %% ---------- 建立主視窗 ----------
    fig = uifigure('Name','雙環 PID 姿態控制互動模擬器 (CURIO 教材)', ...
                    'Position',[80 60 1420 820]);

    mainGrid = uigridlayout(fig,[1 2]);
    mainGrid.ColumnWidth = {330,'1x'};
    mainGrid.RowHeight   = {'1x'};

    %% ---------- 左側：控制面板 ----------
    leftPanel = uipanel(mainGrid,'Title','控制參數 (Sliders)');
    leftGrid = uigridlayout(leftPanel,[1 1]);
    leftGrid.RowHeight = {'1x'};
    scrollGrid = uigridlayout(leftGrid,[16 1]);
    scrollGrid.RowHeight = repmat({'fit'},1,16);
    scrollGrid.Scrollable = 'on';

    paramDefs = {
        'I',       '轉動慣量 I (kg·m^2)',        5e-5,  1e-3
        'Kp_rate', '內環 Kp\_rate (角速率P)',      0,     0.5
        'Ki_rate', '內環 Ki\_rate (角速率I)',      0,     5
        'Kd_rate', '內環 Kd\_rate (角速率D)',      0,     0.01
        'Kp_att',  '外環 Kp\_att (姿態P)',         0,     50
        'Ki_att',  '外環 Ki\_att (姿態I)',         0,     20
        'Kd_att',  '外環 Kd\_att (姿態D)',         0,     5
        };

    row = 1;
    for k = 1:size(paramDefs,1)
        name = paramDefs{k,1};
        labelStr = paramDefs{k,2};
        lo = paramDefs{k,3};
        hi = paramDefs{k,4};
        val = P.(name);

        grp = uigridlayout(scrollGrid,[2 1]);
        grp.Layout.Row = row; row = row+1;
        grp.RowHeight = {'fit','fit'};
        grp.Padding = [0 4 0 4];

        lbl = uilabel(grp,'Text',sprintf('%s  = %.5g',labelStr,val));
        lbl.FontWeight = 'bold';

        sub = uigridlayout(grp,[1 2]);
        sub.ColumnWidth = {'1x',80};

        sld = uislider(sub,'Limits',[lo hi],'Value',val);
        sld.ValueChangedFcn = @(src,evt) onSliderChanged(name);

        ef = uieditfield(sub,'numeric','Limits',[lo hi],'Value',val, ...
                          'ValueDisplayFormat','%.5g');
        ef.ValueChangedFcn = @(src,evt) onEditChanged(name);

        sliderH.(name) = sld;
        editH.(name)   = ef;
        labelH.(name)  = lbl; %#ok<STRNU>
        labelHandles.(name) = lbl;
    end

    % Reset 按鈕
    btnGrid = uigridlayout(scrollGrid,[1 1]);
    btnGrid.Layout.Row = row; row = row+1;
    resetBtn = uibutton(btnGrid,'Text','重設為預設值 (Reset)', ...
                         'ButtonPushedFcn',@(src,evt) onReset());

    % 資訊欄
    infoGrid = uigridlayout(scrollGrid,[1 1]);
    infoGrid.Layout.Row = row; row = row+1; %#ok<NASGU>
    infoArea = uitextarea(infoGrid,'Editable','off', ...
                           'FontName','Consolas','FontSize',12);

    %% ---------- 右側：圖表區 (Tab) ----------
    rightPanel = uipanel(mainGrid,'Title','特徵曲線');
    rightGrid = uigridlayout(rightPanel,[1 1]);
    tg = uitabgroup(rightGrid);

    tab1 = uitab(tg,'Title','階躍響應 Step Response');
    g1 = uigridlayout(tab1,[1 1]);
    ax.step = uiaxes(g1);

    tab2 = uitab(tg,'Title','Bode 圖 (外環全系統)');
    g2 = uigridlayout(tab2,[2 1]);
    ax.bodeMag = uiaxes(g2);
    ax.bodePhase = uiaxes(g2);

    tab3 = uitab(tg,'Title','Root Locus (參數掃描)');
    g3 = uigridlayout(tab3,[2 1]);
    g3.RowHeight = {'fit','1x'};
    g3ctrl = uigridlayout(g3,[1 3]);
    g3ctrl.ColumnWidth = {'fit','fit','1x'};
    uilabel(g3ctrl,'Text','掃描參數 (0 -> 該滑桿上限):','FontWeight','bold');
    sweepDropdown = uidropdown(g3ctrl, ...
        'Items',{'Kp_att','Ki_att','Kd_att','Kp_rate','Ki_rate','Kd_rate'}, ...
        'Value',sweepParam, ...
        'ValueChangedFcn',@(src,evt) onSweepParamChanged());
    ax.rl = uiaxes(g3);
    ax.rl.Layout.Row = 2;

    tab4 = uitab(tg,'Title','Pole-Zero Map (外環閉環)');
    g4 = uigridlayout(tab4,[1 1]);
    ax.pz = uiaxes(g4);

    %% ---------- 初始繪圖 ----------
    updatePlots();

    %% =================== Nested Callback Functions ===================
    function onSliderChanged(name)
        val = sliderH.(name).Value;
        P.(name) = val;
        editH.(name).Value = val;
        refreshLabel(name,val);
        updatePlots();
    end

    function onEditChanged(name)
        val = editH.(name).Value;
        P.(name) = val;
        sliderH.(name).Value = val;
        refreshLabel(name,val);
        updatePlots();
    end

    function refreshLabel(name,val)
        idx = find(strcmp(paramDefs(:,1),name),1);
        labelHandles.(name).Text = sprintf('%s  = %.5g',paramDefs{idx,2},val);
    end

    function onReset()
        fn = fieldnames(defaultP);
        for i = 1:numel(fn)
            nm = fn{i};
            P.(nm) = defaultP.(nm);
            sliderH.(nm).Value = defaultP.(nm);
            editH.(nm).Value   = defaultP.(nm);
            refreshLabel(nm,defaultP.(nm));
        end
        updatePlots();
    end

    function onSweepParamChanged()
        sweepParam = sweepDropdown.Value;
        updatePlots();
    end

    %% ---- 共用：依給定參數結構組出 Tin/Ttotal/OpenOuter ----
    function [Tin,Ttotal,OpenOuter] = buildSystems(Pin)
        s = tf('s');
        Gw   = 1/(Pin.I*s);
        Cin  = Pin.Kp_rate + Pin.Ki_rate/s + Pin.Kd_rate*s;
        OpenInner = Cin*Gw;
        Tin  = feedback(OpenInner,1);

        Gtheta = 1/s;
        Cout   = Pin.Kp_att + Pin.Ki_att/s + Pin.Kd_att*s;
        OpenOuter = Cout*Tin*Gtheta;
        Ttotal = feedback(OpenOuter,1);
    end

    %% =================== 核心：重新計算並繪圖 ===================
    function updatePlots()
        [Tin,Ttotal,OpenOuter] = buildSystems(P); %#ok<ASGLU>

        %% ---- 1) 階躍響應 ----
        tVec = linspace(0,2,2000);
        cla(ax.step);
        try
            [yIn,tIn]   = step(Tin,tVec);
            [yOut,tOut] = step(Ttotal,tVec);
            plot(ax.step,tIn,yIn,'b-','LineWidth',1.5,'DisplayName','內環 Tin(s) 角速率響應'); hold(ax.step,'on');
            plot(ax.step,tOut,yOut,'r-','LineWidth',1.8,'DisplayName','外環 Ttotal(s) 姿態響應');
            yline(ax.step,1,'k--','HandleVisibility','off');
            hold(ax.step,'off');
            legend(ax.step,'Location','southeast');
        catch ME
            title(ax.step,['無法計算階躍響應: ' ME.message]);
        end
        grid(ax.step,'on');
        xlabel(ax.step,'時間 (s)'); ylabel(ax.step,'正規化輸出');
        title(ax.step,'階躍響應比較 (內環 vs 外環)');

        %% ---- 2) Bode 圖 (外環全系統) ----
        cla(ax.bodeMag); cla(ax.bodePhase);
        try
            w = logspace(-1,4,800);
            [mag,phase,wout] = bode(Ttotal,w);
            magdB = 20*log10(squeeze(mag));
            ph    = squeeze(phase);
            semilogx(ax.bodeMag,wout,magdB,'b-','LineWidth',1.5);
            grid(ax.bodeMag,'on');
            ylabel(ax.bodeMag,'幅值 (dB)');
            title(ax.bodeMag,'Bode Diagram - Ttotal(s)');

            semilogx(ax.bodePhase,wout,ph,'b-','LineWidth',1.5);
            grid(ax.bodePhase,'on');
            xlabel(ax.bodePhase,'頻率 (rad/s)'); ylabel(ax.bodePhase,'相位 (deg)');
        catch ME
            title(ax.bodeMag,['無法計算 Bode: ' ME.message]);
        end

        %% ---- 3) Root Locus (單一參數掃描, 其餘參數固定於目前滑桿值) ----
        cla(ax.rl);
        try
            idxSw = find(strcmp(paramDefs(:,1),sweepParam),1);
            loSw = paramDefs{idxSw,3};
            hiSw = paramDefs{idxSw,4};
            sweepVals = linspace(loSw,hiSw,150);

            allRe = []; allIm = []; allVal = [];
            for v = sweepVals
                Ptmp = P;
                Ptmp.(sweepParam) = v;
                [~,TtotalTmp] = buildSystems(Ptmp);
                pTmp = pole(TtotalTmp);
                allRe  = [allRe;  real(pTmp)];  %#ok<AGROW>
                allIm  = [allIm;  imag(pTmp)];  %#ok<AGROW>
                allVal = [allVal; repmat(v,numel(pTmp),1)]; %#ok<AGROW>
            end

            pCurrent = pole(Ttotal);
            rmax = max([abs([allRe(:)+1i*allIm(:); pCurrent(:)]); 5]) * 1.15;

            hold(ax.rl,'on');
            drawSGrid(ax.rl,rmax);
            scatter(ax.rl,allRe,allIm,14,allVal,'filled','MarkerFaceAlpha',0.65, ...
                    'HandleVisibility','off');
            colormap(ax.rl,'jet');
            cb = colorbar(ax.rl);
            cb.Label.String = sprintf('%s 值',sweepParam);

            plot(ax.rl,real(pCurrent),imag(pCurrent),'kp','MarkerSize',13, ...
                 'MarkerFaceColor','r','LineWidth',1.2,'DisplayName','目前工作點極點');
            xline(ax.rl,0,'k-','HandleVisibility','off');
            yline(ax.rl,0,'k-','HandleVisibility','off');
            hold(ax.rl,'off');
            legend(ax.rl,'Location','best');
            xlim(ax.rl,[-rmax*1.05, rmax*0.3]);
            ylim(ax.rl,[-rmax*1.05, rmax*1.05]);
        catch ME
            title(ax.rl,['無法計算 Root Locus: ' ME.message]);
        end
        grid(ax.rl,'on'); axis(ax.rl,'equal');
        xlabel(ax.rl,'Real Axis'); ylabel(ax.rl,'Imaginary Axis');
        title(ax.rl,sprintf('Root Locus - 掃描 %s (0->slider上限, 其餘參數固定)',sweepParam));

        %% ---- 4) Pole-Zero Map (外環閉環) ----
        cla(ax.pz);
        try
            p = pole(Ttotal);
            z = zero(Ttotal);
            rmaxPZ = max([abs(p(:)); 5]) * 1.3;
            hold(ax.pz,'on');
            drawSGrid(ax.pz,rmaxPZ);
            plot(ax.pz,real(p),imag(p),'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Poles');
            if ~isempty(z)
                plot(ax.pz,real(z),imag(z),'bo','MarkerSize',8,'LineWidth',1.5,'DisplayName','Zeros');
            end
            xline(ax.pz,0,'k-','HandleVisibility','off');
            yline(ax.pz,0,'k-','HandleVisibility','off');
            hold(ax.pz,'off');
            legend(ax.pz,'Location','best');
            xlim(ax.pz,[-rmaxPZ*1.05, rmaxPZ*0.3]);
            ylim(ax.pz,[-rmaxPZ*1.05, rmaxPZ*1.05]);
        catch ME
            title(ax.pz,['無法計算 Pole-Zero: ' ME.message]);
        end
        grid(ax.pz,'on'); axis(ax.pz,'equal');
        xlabel(ax.pz,'Real Axis'); ylabel(ax.pz,'Imaginary Axis');
        title(ax.pz,'Pole-Zero Map - 外環閉環 Ttotal(s)');

        %% ---- 5) 資訊欄：穩定性 / 阻尼 / 頻寬分離 ----
        lines = {};
        try
            p = pole(Ttotal);
            isStable = all(real(p) < 0);
            lines{end+1} = sprintf('外環系統階數: %d (應為 3 階)', order(Ttotal));
            lines{end+1} = sprintf('穩定性: %s', ternary(isStable,'穩定 (Stable)','不穩定 (Unstable)'));
            [wn,zeta] = damp(Ttotal);
            for i = 1:numel(wn)
                lines{end+1} = sprintf('  極點%d: wn=%.3f rad/s, zeta=%.3f', i, wn(i), zeta(i)); %#ok<AGROW>
            end
        catch
            lines{end+1} = '無法計算極點/阻尼資訊 (系統可能異常)';
        end

        try
            info = stepinfo(Ttotal);
            lines{end+1} = sprintf('Overshoot: %.1f %%', info.Overshoot);
            lines{end+1} = sprintf('Settling Time: %.3f s', info.SettlingTime);
        catch
            lines{end+1} = '無法計算 stepinfo (可能發散)';
        end

        try
            bwIn  = bandwidth(Tin);
            bwOut = bandwidth(Ttotal);
            ratio = bwIn/bwOut;
            lines{end+1} = sprintf('內環頻寬: %.2f rad/s', bwIn);
            lines{end+1} = sprintf('外環頻寬: %.2f rad/s', bwOut);
            lines{end+1} = sprintf('頻寬比 (內/外): %.2f 倍', ratio);
            if ratio >= 4
                lines{end+1} = '  -> 符合頻帶分離原則 (>=4倍), Tin(s)≈1 近似有效';
            else
                lines{end+1} = '  -> 未達建議頻帶分離(4~5倍), 外環近似二階可能失準';
            end
        catch
            lines{end+1} = '無法計算頻寬 (可能系統不穩定)';
        end

        infoArea.Value = lines;
    end

    function out = ternary(cond,a,b)
        if cond, out = a; else, out = b; end
    end

    %% ---- 手動繪製 s-plane 等阻尼比(zeta)/等自然頻率(wn)格線 ----
    % (uiaxes 對內建 sgrid() 相容性不佳，改用手動繪製以確保跨版本可用)
    function drawSGrid(axH,rmax)
        gridColor = [0.75 0.75 0.75];
        zetas = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
        for zz = zetas
            dirx = -zz;
            diry = sqrt(max(0,1-zz^2));
            plot(axH,[0 dirx*rmax],[0 diry*rmax],':','Color',gridColor,'HandleVisibility','off');
            plot(axH,[0 dirx*rmax],[0 -diry*rmax],':','Color',gridColor,'HandleVisibility','off');
            text(axH,dirx*rmax*1.03,diry*rmax*1.03, sprintf('\\zeta=%.1f',zz), ...
                 'FontSize',7,'Color',gridColor,'HorizontalAlignment','center');
        end
        wns = linspace(rmax/5, rmax, 5);
        th = linspace(pi/2, 3*pi/2, 100);
        for ww = wns
            plot(axH, ww*cos(th), ww*sin(th), ':','Color',gridColor,'HandleVisibility','off');
            text(axH, -ww*0.98, ww*0.12, sprintf('\\omega_n=%.2g',ww), ...
                 'FontSize',7,'Color',gridColor,'HorizontalAlignment','left');
        end
    end

end
