classdef OrigamiRobotProMLPAt < handle
    properties
        Fig, Ax3D, AxWS, AxTopView, AxMap
        RobotAssembly = {}  
        L = 30;             
        V_depth = 12;       
        T_pad = 4;          
        UI_TPad             
        UI_TSkel            
        UIList, SldAngle, LblSld, UITextOut, UIProgram
        UI_PressureEdit
        UIStartFromCurrent
        UIModelDetailsButton, UIPanelModelDetails, UIModelDetailsText
        UIModelDetailsExpanded = false
        MapLine = []; MapKnots = []; MapMarker = [];
        UI_TrackDropdown    
        UI_EE_X, UI_EE_Y, UI_EE_Z 
        UI_PaletteDropdown
        CurrentWSPoints = []; 
        CurrentTrace = []; 
        CurrentTraceStates = [];
        CurrentTraceBreakIdx = [];
        PaperPoseCount = 4;

        % ===== Real-time rendering: fixed view and reused graphics handles =====
        LivePatch = struct();
        LiveTip = [];
        LiveTrail = [];
        LiveFrame = struct();
        TopViewGraphics = struct();
        LiveGraphicsReady = false;
        ViewEnvelope = [];
        TopViewLimits = [];      % Fixed top-view X-Y bounds [xmin xmax ymin ymax]
        ViewEnvelopeDirty = true;
        AnimationFPS = 24;       % Recommended recording frame rate: 24 fps
        SimulationHz = 72;       % Internal trajectory sampling rate (render every 3 samples)
        RealTimePlayback = true; % Play motion using the durations specified in the script
    end
    
    methods
        function obj = OrigamiRobotProMLPAt()
            obj.Fig = uifigure('Name', 'Origami Soft Robotic Arm: Kinematics & Workspace', 'Position', [30 30 1760 900], ...
                'Color', [0.965 0.972 0.978]);
            
            % ================= 1. Assembly and actuation panel =================
            pnl = uipanel(obj.Fig, 'Title', 'Assembly & Actuation Control', 'Position', [10 10 390 880], ...
                'ForegroundColor', [0.15 0.15 0.15], 'BackgroundColor', [0.985 0.988 0.99], 'BorderType', 'line');
            
            uilabel(pnl, 'Text', '1. Module Definition (Limits & Theoretical Pressure Mapping):', 'Position', [10 790 370 22], 'FontWeight', 'bold');
            
            uilabel(pnl, 'Text', 'Module ID:', 'Position', [10 760 60 22]);
            efName = uieditfield(pnl, 'text', 'Position', [72 760 92 22], 'Value', 'A');
            
            uilabel(pnl, 'Text', 'Skeletons n:', 'Position', [168 760 65 22]);
            efN = uieditfield(pnl, 'text', 'Position', [235 760 35 22], 'Value', '3', 'UserData', 3);
            uilabel(pnl, 'Text', 'Seq.:', 'Position', [274 760 30 22]);
            efSeq = uieditfield(pnl, 'text', 'Position', [306 760 74 22], 'Value', '0');
            
            uilabel(pnl, 'Text', 'Initial θ (°):', 'Position', [10 730 78 22]);
            efA1 = uieditfield(pnl, 'text', 'Position', [90 730 45 22], 'Value', '45.000', 'UserData', 45, 'Enable', 'off');
            uilabel(pnl, 'Text', 'Initial P (kPa):', 'Position', [145 730 85 22]);
            efP1 = uieditfield(pnl, 'text', 'Position', [235 730 50 22], 'Value', '0.000', 'UserData', 0);
            
            uilabel(pnl, 'Text', 'Final θ (°):', 'Position', [10 700 78 22]);
            efA2 = uieditfield(pnl, 'text', 'Position', [90 700 45 22], 'Value', '9.083', 'UserData', 9.082671, 'Enable', 'off');
            uilabel(pnl, 'Text', 'Final P (kPa):', 'Position', [145 700 85 22]);
            efP2 = uieditfield(pnl, 'text', 'Position', [235 700 50 22], 'Value', '-70.000', 'UserData', -70);
            efP1.ValueChangedFcn = @(ef,e) obj.updateMappingPreview(efP1, efP2, efA1, efA2);
            efP2.ValueChangedFcn = @(ef,e) obj.updateMappingPreview(efP1, efP2, efA1, efA2);
            
            obj.UIModelDetailsButton = uibutton(pnl, 'Text', '▶ Show Theoretical Model', ...
                'Position', [10 670 370 22], 'BackgroundColor', [0.91 0.95 0.96], ...
                'FontColor', [0.16 0.48 0.51], 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(btn,e) obj.toggleModelDetails());

            uilabel(pnl, 'Text', 'Config. yaw:', 'Position', [10 640 72 22]);
            efInstRot = uieditfield(pnl, 'text', 'Position', [85 640 45 22], 'Value', '0', 'UserData', 0);
            uilabel(pnl, 'Text', 'Pitch:', 'Position', [145 640 42 22]);
            efInstPitch = uieditfield(pnl, 'text', 'Position', [190 640 45 22], 'Value', '0', 'UserData', 0);
            uilabel(pnl, 'Text', '(interface angle s_{i,0})', 'Position', [245 640 140 22], 'FontColor', [.5 .5 .5], 'FontSize', 10);

            uibutton(pnl, 'Text', 'Add Module Using Theoretical Mapping', 'Position', [10 600 370 32], 'BackgroundColor', [0.16 0.48 0.51], 'FontColor', 'w', ...
                'FontWeight', 'bold', 'ButtonPushedFcn', @(btn,e) obj.addModuleFromInputs(efName, efN, efSeq, efInstRot, efInstPitch, efA1, efA2, efP1, efP2));
            
            % --- 2. Robot assembly tree ---
            uilabel(pnl, 'Text', '2. Robot Assembly Sequence (Ctrl for multi-select):', 'Position', [10 540 360 22], 'FontWeight', 'bold');
            obj.UIList = uilistbox(pnl, 'Position', [10 445 370 90], 'Multiselect', 'on', 'ValueChangedFcn', @(lb,e) obj.onListSelect());
            
            uibutton(pnl, 'Text', 'Select All', 'Position', [10 410 100 28], 'BackgroundColor', [0.92 0.96 0.98], 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn,e) obj.selectAllModules());
            uibutton(pnl, 'Text', 'Delete Selected', 'Position', [120 410 110 28], 'BackgroundColor', [0.98 0.92 0.92], 'ButtonPushedFcn', @(btn,e) obj.deleteSelected());
            uibutton(pnl, 'Text', 'Clear All', 'Position', [240 410 140 28], 'ButtonPushedFcn', @(btn,e) obj.resetRobot());
            
            % --- 3. Pressure-driven motion sequence editor ---
            uilabel(pnl, 'Text', '3. Motion Sequence Generator', 'Position', [10 380 180 22], 'FontWeight', 'bold');
            uilabel(pnl, 'Text', 'Targets: P:0 / P:-30 / P:-70', 'Position', [185 380 195 22], 'FontSize', 9.2, 'FontColor', [0.5 0.5 0.5]);

            prog_default = sprintf(['%% Motion command: Module, P:<kPa>, duration_s\n' ...
                '%% Hold command: k+Module, P:<kPa>; use P:0, P:-30, or P:-70\n' ...
                '%% Repeated commands for one module run sequentially in listed order.\n' ...
                '%% Commands for different modules on the same line run concurrently.\n' ...
                'k+C, P:-70 ; k+A, P:-70;k+B, P:-70;\n' ...
                'k+C, P:-70 ; A, P:0,0.5;k+B, P:-70;\n' ...
                'k+C, P:-70 ; A, P:-70,0.5;k+B, P:-70;\n' ...
                'k+C, P:-70 ; k+A, P:-70;B, P:0,0.5;\n' ...
                'k+C, P:-70 ; k+A, P:-70;B, P:-70,0.5;']);
            obj.UIProgram = uitextarea(pnl, 'Position', [10 252 370 123], 'Value', prog_default, 'FontName', 'Consolas', 'FontSize', 11.5);
            obj.UIStartFromCurrent = uicheckbox(pnl, 'Text', 'Start from current pressure (uncheck to reset)', ...
                'Position', [10 232 370 18], 'Value', true, 'FontSize', 10, ...
                'Tooltip', 'Checked: continue from the current module pressures. Unchecked: reset all modules to the initial pressure before playback.');

            uibutton(pnl, 'Text', 'Run Sequence & Generate Tip Trajectory', 'Position', [10 198 370 32], 'BackgroundColor', [0.88 0.30 0.16], 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn,e) obj.runProgram());

            % Collapsible equation panel for a clean default interface.
            obj.UIPanelModelDetails = uipanel(pnl, 'Title', 'Theoretical Pressure-Angle / Axial-Length Model', ...
                'Position', [10 230 370 360], 'Visible', 'off', ...
                'BackgroundColor', [0.985 0.988 0.99], 'ForegroundColor', [0.16 0.35 0.38], ...
                'BorderType', 'line');
            model_details = { ...
                'SINGLE-SKELETON PRESSURE-FOLDING-ANGLE MODEL'; ...
                'x = theta_0 - theta  (radians)'; ...
                'U(x) = (a/2)*x^2 + [b/(c+1)]*x^(c+1)'; ...
                'dU/dx = a*x + b*x^c'; ...
                'P_g(theta) = -1e6*(a*x + b*x^c) / J_V(theta)'; ...
                'J_V(theta) = l*(2*w+w_top)/6 * [l*cos(theta) -'; ...
                'cos(theta/2)*(h_trap^2-2*l^2*sin(theta/2)^2) /'; ...
                'sqrt(h_trap^2-l^2*sin(theta/2)^2)]'; ...
                ''; ...
                'FINAL CALIBRATED PARAMETERS'; ...
                'a=0.2313, b=35.5672, c=10.5289'; ...
                'l=30 mm, w=30 mm, w_top=16 mm, h_trap=15 mm'; ...
                'theta_0=45 deg, h_g=0.8 mm, h_p=3.5 mm'; ...
                ''; ...
                'CONFIGURATION-DEPENDENT OUTPUT'; ...
                'B1: single-skeleton fold angle theta(P_g)'; ...
                'B3: total bending angle from the configured stack'; ...
                'Straight stack: L(P_g) = L0 - DeltaL(P_g)'; ...
                'DeltaL = n*[l/2*(sin(theta_0)-sin(theta))'; ...
                '+ h_g/2*(cos(theta)-cos(theta_0))]'; ...
                'L0 = 2*h_p + n*l/2*sin(theta_0) + (n-1)*h_g'; ...
                ''; ...
                'Equation angles are in radians.'; ...
                'The interface displays angles in degrees.'};
            obj.UIModelDetailsText = uitextarea(obj.UIPanelModelDetails, ...
                'Position', [8 8 354 324], 'Editable', 'off', ...
                'FontName', 'Consolas', 'FontSize', 9, 'Value', model_details);
            
            % --- Pressure-driven actuation monitor ---
            uilabel(pnl, 'Text', 'Pressure (kPa):', 'Position', [10 168 96 22], ...
                'FontWeight', 'bold', 'FontColor', [0.16 0.48 0.51]);
            obj.UI_PressureEdit = uieditfield(pnl, 'text', 'Position', [108 168 62 22], ...
                'Value', '0.000', 'UserData', 0, 'Enable', 'off', ...
                'ValueChangedFcn', @(ef,e) obj.onPressureTextChanged(ef));
            obj.LblSld = uilabel(pnl, 'Text', 'Angle: --° | State: --%', 'Position', [180 168 200 22], ...
                'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'FontColor', [0.16 0.48 0.51]);
            obj.SldAngle = uislider(pnl, 'Position', [10 158 370 3], 'Limits', [-101.3 0], ...
                'MajorTicks', [-100 -80 -60 -40 -20 0], 'MinorTicks', [], ...
                'ValueChangingFcn', @(sld,e) obj.onSliderChanging(e.Value));
            obj.SldAngle.Enable = 'off';
            
            uilabel(pnl, 'Text', '4. Global Parameters (Kinematics):', 'Position', [10 115 300 22], 'FontWeight', 'bold');
            
            uilabel(pnl, 'Text', 'Plate Hp:', 'Position', [10 85 50 22]);
            obj.UI_TPad = uieditfield(pnl, 'text', 'Position', [60 85 30 22], 'Value', '3.5', 'UserData', 3.5, 'ValueChangedFcn', @(ef,e) obj.onNumericKinematicsFieldChanged(ef, 'Plate thickness Hp', 0, Inf));
            
            uilabel(pnl, 'Text', 'Gap Hg:', 'Position', [95 85 50 22]);
            obj.UI_TSkel = uieditfield(pnl, 'text', 'Position', [150 85 35 22], 'Value', '0.8', 'UserData', 0.8, 'ValueChangedFcn', @(ef,e) obj.onNumericKinematicsFieldChanged(ef, 'Layer gap Hg', 0, Inf));
            
            uilabel(pnl, 'Text', 'Track:', 'Position', [190 85 40 22]);
            obj.UI_TrackDropdown = uidropdown(pnl, 'Position', [232 85 148 22], 'Items', {'Global Tip (Default)'}, 'BackgroundColor', [0.96 0.96 0.96], 'ValueChangedFcn', @(dd,e) obj.onKinematicsChanged());
            
            uilabel(pnl, 'Text', 'Offset (X,Y,Z):', 'Position', [10 55 90 22]);
            obj.UI_EE_X = uieditfield(pnl, 'text', 'Position', [100 55 40 22], 'Value', '0', 'UserData', 0, 'ValueChangedFcn', @(ef,e) obj.onNumericKinematicsFieldChanged(ef, 'Offset X', -Inf, Inf));
            obj.UI_EE_Y = uieditfield(pnl, 'text', 'Position', [145 55 40 22], 'Value', '0', 'UserData', 0, 'ValueChangedFcn', @(ef,e) obj.onNumericKinematicsFieldChanged(ef, 'Offset Y', -Inf, Inf));
            obj.UI_EE_Z = uieditfield(pnl, 'text', 'Position', [190 55 40 22], 'Value', '0', 'UserData', 0, 'ValueChangedFcn', @(ef,e) obj.onNumericKinematicsFieldChanged(ef, 'Offset Z', -Inf, Inf));
            uilabel(pnl, 'Text', 'Theme:', 'Position', [240 55 45 22]);
            obj.UI_PaletteDropdown = uidropdown(pnl, 'Position', [285 55 95 22], ...
                'Items', {'Teal', 'Blue', 'Gray'}, 'Value', 'Teal', ...
                'ValueChangedFcn', @(dd,e) obj.renderAndResize());
            
            % ================= Workspace generation and export controls =================
            uibutton(pnl, 'Text', 'Workspace', 'Position', [10 15 80 28], 'BackgroundColor', [0.16 0.48 0.51], 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn,e) obj.calcWorkspaceMC());
            uibutton(pnl, 'Text', 'Clear', 'Position', [95 15 45 28], 'BackgroundColor', [0.98 0.92 0.92], 'ButtonPushedFcn', @(btn,e) obj.clearTrace());
            uibutton(pnl, 'Text', 'Paper Fig.', 'Position', [145 15 55 28], 'BackgroundColor', [0.92 0.96 0.98], 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn,e) obj.exportPaperFigure());
            uibutton(pnl, 'Text', 'Plot', 'Position', [205 15 45 28], 'BackgroundColor', [0.98 0.96 0.94], 'ButtonPushedFcn', @(btn,e) obj.popOutAxes(obj.AxWS, 'Workspace / Trajectory', false));
            uibutton(pnl, 'Text', 'Save/Load', 'Position', [255 15 70 28], 'BackgroundColor', [0.94 0.96 0.94], 'ButtonPushedFcn', @(btn,e) obj.saveLoadMenu());
            uibutton(pnl, 'Text', 'Export', 'Position', [330 15 50 28], 'BackgroundColor', [0.98 0.96 0.90], 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn,e) obj.exportData());
            
            % ================= 2. Real-time 3D model =================
            pnl3D = uipanel(obj.Fig, 'Title', 'Real-Time Robot Model', 'Position', [410 10 840 880], ...
                'ForegroundColor', [0.15 0.15 0.15], 'BackgroundColor', [0.985 0.988 0.99], 'BorderType', 'line');
            obj.Ax3D = uiaxes(pnl3D, 'Position', [8 8 824 840]);
            obj.init3DAxes(obj.Ax3D);
            enableDefaultInteractivity(obj.Ax3D);

            % Add a top-view inset to inspect lateral motion
            obj.AxTopView = uiaxes(pnl3D, 'Position', [525 18 285 235]);
            obj.initTopViewAxes();
            
            % ================= 3. Workspace and data report =================
            pnlWS = uipanel(obj.Fig, 'Title', 'Decoupled Tip Analysis (Trajectory & Workspace)', 'Position', [1260 10 490 880], ...
                'ForegroundColor', [0.15 0.15 0.15], 'BackgroundColor', [0.985 0.985 0.98], 'BorderType', 'line');
            obj.AxWS = uiaxes(pnlWS, 'Position', [20 350 440 370]);

            % Pressure-angle mapping is kept visible during operation so the
            % supplementary video directly shows the actuation input-output law.
            pnlMap = uipanel(pnlWS, 'Title', 'Pressure–Angle / Axial-Length Mapping', 'Position', [10 70 470 280], ...
                'ForegroundColor', [0.15 0.15 0.15], 'BackgroundColor', [0.992 0.992 0.988], 'BorderType', 'line');
            obj.AxMap = uiaxes(pnlMap, 'Position', [45 18 410 225]);
            obj.initPressureMapAxes();

            pnlData = uipanel(pnlWS, 'Title', 'Physical Status & Kinematic Limits', 'Position', [10 10 470 60], 'ForegroundColor', [0.15 0.15 0.15], 'BorderType', 'line');
            obj.UITextOut = uitextarea(pnlData, 'Position', [5 4 460 35], 'Editable', 'off', ...
                'FontName', 'Consolas', 'FontSize', 9.5, 'Value', 'Ready. Add modules and run the sequence.');
            obj.showEmptyWorkspaceState();
            obj.showEmptyPressureMap();
        end
        
        function init3DAxes(obj, ax)
            grid(ax, 'off'); view(ax, 38, 20); 
            ax.DataAspectRatio = [1 1 1];
            xlabel(ax, 'X (mm)', 'FontWeight', 'normal'); ylabel(ax, 'Y (mm)', 'FontWeight', 'normal'); zlabel(ax, 'Z (mm)', 'FontWeight', 'normal');
            set(ax, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.8, 'Box', 'off', 'Projection', 'orthographic');
            ax.Color = [1 1 1]; 
            camup(ax, [0 0 1]);
        end

        function initTopViewAxes(obj)
            cla(obj.AxTopView, 'reset');
            hold(obj.AxTopView, 'on');
            view(obj.AxTopView, 2);
            grid(obj.AxTopView, 'on');
            axis(obj.AxTopView, 'equal');
            set(obj.AxTopView, 'FontName', 'Arial', 'FontSize', 9.5, 'LineWidth', 0.75, ...
                'Box', 'on', 'Color', [1 1 1]);
            obj.AxTopView.GridColor = [0.80 0.83 0.86];
            obj.AxTopView.GridAlpha = 0.22;
            xlabel(obj.AxTopView, 'X (mm)', 'FontWeight', 'bold');
            ylabel(obj.AxTopView, 'Y (mm)', 'FontWeight', 'bold');
            title(obj.AxTopView, 'Top View (X-Y)', 'FontName', 'Arial', 'FontSize', 10.5);
            hold(obj.AxTopView, 'off');
        end


        function initPressureMapAxes(obj)
            cla(obj.AxMap, 'reset');
            hold(obj.AxMap, 'on');
            grid(obj.AxMap, 'on'); box(obj.AxMap, 'on');
            set(obj.AxMap, 'FontName', 'Arial', 'FontSize', 8.5, 'LineWidth', 0.7, 'Color', 'w');
            obj.AxMap.GridColor = [0.82 0.84 0.86];
            obj.AxMap.GridAlpha = 0.22;
            xlabel(obj.AxMap, 'Pressure (kPa)', 'FontWeight', 'bold');
            ylabel(obj.AxMap, 'Fold angle θ (deg)', 'FontWeight', 'bold');
            obj.AxMap.XDir = 'reverse';
            obj.AxMap.XTick = [-101.3, -100:10:0];
            hold(obj.AxMap, 'off');
        end

        function showEmptyPressureMap(obj)
            if isempty(obj.AxMap) || ~isvalid(obj.AxMap), return; end
            obj.initPressureMapAxes();
            text(obj.AxMap, 0.5, 0.52, 'Select a module to display the theoretical pressure mapping', ...
                'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                'FontName', 'Arial', 'FontSize', 9, 'Color', [0.48 0.51 0.53]);
            obj.MapLine = []; obj.MapKnots = []; obj.MapMarker = [];
        end

        function showPressureMap(obj, mod, curr_p, ang)
            if isempty(obj.AxMap) || ~isvalid(obj.AxMap), return; end
            obj.initPressureMapAxes();
            hold(obj.AxMap, 'on');

            p_lo = max(-101.3, min([mod.p_start, mod.p_end]));
            p_hi = min(0, max([mod.p_start, mod.p_end]));
            if p_hi - p_lo < 1e-9
                p_lo = max(-101.3, p_lo - 0.5);
                p_hi = min(0, p_hi + 0.5);
            end
            Pq = linspace(p_lo, p_hi, 240);
            [theta_q, ~, stroke_q] = obj.pressureToB1B3L8(Pq);
            [Yq, y_label, mode_label] = obj.getMappingOutput(mod, Pq, theta_q, stroke_q);
            [y_now, ~, ~] = obj.getMappingOutput(mod, curr_p);

            style = obj.getPaperPalette();
            obj.MapLine = plot(obj.AxMap, Pq, Yq, '-', 'Color', style.side, 'LineWidth', 2.2);
            Pmark = linspace(p_lo, p_hi, 8);
            [theta_mark, ~, stroke_mark] = obj.pressureToB1B3L8(Pmark);
            Ymark = obj.getMappingOutput(mod, Pmark, theta_mark, stroke_mark);
            obj.MapKnots = plot(obj.AxMap, Pmark, Ymark, 'o', ...
                'MarkerSize', 4.2, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', style.edge, 'LineWidth', 0.9);
            obj.MapMarker = plot(obj.AxMap, curr_p, y_now, 'o', ...
                'MarkerSize', 8, 'MarkerFaceColor', style.trajectory, ...
                'MarkerEdgeColor', style.edge, 'LineWidth', 1.0);

            if abs(p_hi-p_lo) < 1e-9
                xlim(obj.AxMap, [p_lo-1 p_hi+1]);
            else
                xlim(obj.AxMap, [p_lo p_hi]);
            end
            obj.AxMap.XDir = 'reverse';
            ticks = -100:10:0;
            ticks = ticks(ticks >= p_lo-1e-9 & ticks <= p_hi+1e-9);
            ticks = unique([ticks, p_lo, p_hi]);
            ticks = ticks(ticks >= p_lo-1e-9 & ticks <= p_hi+1e-9);
            if ~isempty(ticks), obj.AxMap.XTick = sort(ticks); end
            % Fit the vertical range to the complete configured output, including
            % high-angle stacks (e.g., 90-degree yaw increments) and the live marker.
            y_all = [Yq(:); y_now(:); Ymark(:)];
            y_all = y_all(isfinite(y_all));
            if isempty(y_all)
                y_min = 0; y_max = 1;
            else
                y_min = min(y_all);
                y_max = max(y_all);
            end
            y_span = max(y_max-y_min, 1);
            y_pad = max([0.14*y_span, 0.035*max(abs([y_min y_max])), 1.5]);
            ylim(obj.AxMap, [y_min-y_pad y_max+y_pad]);
            obj.AxMap.YLimMode = 'manual';
            ylabel(obj.AxMap, y_label, 'FontWeight', 'bold');
            title(obj.AxMap, sprintf('Module %s: Pressure → %s', mod.name, mode_label), ...
                'FontName', 'Arial', 'FontSize', 10.5);
            hold(obj.AxMap, 'off');
        end

        function updatePressureMapMarker(obj, mod, curr_p, ang)
            if isempty(obj.MapMarker) || ~all(isgraphics(obj.MapMarker))
                obj.showPressureMap(mod, curr_p, ang);
                return;
            end
            [y_now, y_label, mode_label] = obj.getMappingOutput(mod, curr_p);
            set(obj.MapMarker, 'XData', curr_p, 'YData', y_now);
            ylabel(obj.AxMap, y_label, 'FontWeight', 'bold');
            title(obj.AxMap, sprintf('Module %s: P = %.1f kPa → %s = %.2f', ...
                mod.name, curr_p, mode_label, y_now), 'FontName', 'Arial', 'FontSize', 10.5);
        end

        % =================================================================================
        % Kinematic feature extraction using homogeneous transforms (length and overall bend)
        % =================================================================================
        function [L, Theta] = getModuleMetrics(obj, curr_mod, pct)
            % Evaluate the current module pose with the theory-derived fold angle.
            T_mod = obj.transz(obj.readNumericFieldValue(obj.UI_TPad));
            [ang, ~] = obj.getInterpolatedAngle(curr_mod, pct);
            tskel = obj.readNumericFieldValue(obj.UI_TSkel);
            for i = 1:curr_mod.n
                if i > 1, T_mod = T_mod * obj.rotz(curr_mod.seq(i-1)); end
                T_mod = T_mod * obj.getUnitTransform(ang, tskel);
            end
            T_mod = T_mod * obj.transz(obj.readNumericFieldValue(obj.UI_TPad));

            % Straight contraction modules use the paper's analytical axial length.
            % Other configurations retain FK-based length and report the configured bend.
            if obj.isStraightContractionModule(curr_mod)
                L = obj.getModuleLengthAtAngle(curr_mod, ang);
                Theta = 0;
            else
                L = norm(T_mod(1:3, 4));
                Theta = obj.getConfiguredBendAngle(curr_mod, ang);
            end
        end
        
        function report_str = generateModuleBoundsReport(obj)
            if isempty(obj.RobotAssembly)
                report_str = ''; return;
            end
            report_str = sprintf('\n[ Module Kinematic Limits (0%% fully extended -> 100%% fully contracted) ]\n');
            for m = 1:length(obj.RobotAssembly)
                curr_mod = obj.RobotAssembly{m};
                [L0, Th0] = obj.getModuleMetrics(curr_mod, 0);
                [L1, Th1] = obj.getModuleMetrics(curr_mod, 100);
                dTh = abs(Th1 - Th0);
                dL = abs(L1 - L0);
                
                report_str = [report_str, sprintf('Module %s (%d skeletons):\n', curr_mod.name, curr_mod.n)];
                report_str = [report_str, sprintf('  - Overall bending: %5.1f° -> %5.1f°  (span: %5.1f°)\n', Th0, Th1, dTh)];
                report_str = [report_str, sprintf('  - Axial length: %5.1f -> %5.1f mm (stroke: %5.1f mm)\n', L0, L1, dL)];
            end
        end
        
        % =================================================================================
        % Manual save/load engine
        % =================================================================================
        function saveLoadMenu(obj)
            choice = uiconfirm(obj.Fig, 'Select a data management action:', 'Save / Load', 'Options', {'Save to Folder', 'Load from File', 'Cancel'}, 'DefaultOption', 1);
            if strcmp(choice, 'Save to Folder')
                [f, p] = uiputfile('*.mat', 'Save Robot Assembly', 'OrigamiArm.mat'); 
                if ~isequal(f,0)
                    data.RobotAssembly = obj.RobotAssembly; 
                    data.TPad = obj.readNumericFieldValue(obj.UI_TPad); 
                    data.TSkel = obj.readNumericFieldValue(obj.UI_TSkel); 
                    data.Program = obj.UIProgram.Value;
                    save(fullfile(p, f), 'data'); 
                    uialert(obj.Fig, sprintf('Assembly and motion script saved successfully to:\n%s', fullfile(p, f)), 'Saved');
                end
            elseif strcmp(choice, 'Load from File')
                [f, p] = uigetfile('*.mat', 'Load Robot Assembly'); 
                if ~isequal(f,0)
                    load(fullfile(p, f), 'data'); 
                    if isfield(data, 'RobotAssembly'), obj.RobotAssembly = data.RobotAssembly; else, obj.RobotAssembly = data; end
                    if isfield(data, 'TPad'), obj.UI_TPad.Value = sprintf('%.12g', data.TPad); obj.UI_TPad.UserData = data.TPad; end
                    if isfield(data, 'TSkel'), obj.UI_TSkel.Value = sprintf('%.12g', data.TSkel); obj.UI_TSkel.UserData = data.TSkel; end
                    if isfield(data, 'Program'), obj.UIProgram.Value = data.Program; end
                    obj.invalidateDerivedResults();
                    obj.ViewEnvelopeDirty = true;
                    obj.resetLiveGraphics();
                    obj.updateList(); obj.onListSelect(); obj.renderAndResize(); 
                    uialert(obj.Fig, 'Assembly and parameters loaded successfully.', 'Loaded');
                end
            end
        end
        
        function exportData(obj)
            if isempty(obj.CurrentWSPoints) && isempty(obj.CurrentTrace)
                uialert(obj.Fig, 'No trajectory or workspace data are available. Run the sequence or calculate the workspace first.', 'Notice'); 
                return; 
            end

            if ~isempty(obj.CurrentWSPoints) && ~isempty(obj.CurrentTrace)
                choice = uiconfirm(obj.Fig, 'Select the data to export.', 'Export Data', ...
                    'Options', {'Tip Trajectory', 'Theoretical Workspace', 'Cancel'}, 'DefaultOption', 1, 'CancelOption', 3);
                if strcmp(choice, 'Cancel'), return; end
                if strcmp(choice, 'Tip Trajectory'), data_to_export = obj.CurrentTrace; default_name = 'Trajectory_Points.csv';
                else, data_to_export = obj.CurrentWSPoints; default_name = 'Workspace_Points.csv'; end
            elseif ~isempty(obj.CurrentTrace)
                data_to_export = obj.CurrentTrace; default_name = 'Trajectory_Points.csv';
            else
                data_to_export = obj.CurrentWSPoints; default_name = 'Workspace_Points.csv';
            end
            [file, path] = uiputfile({'*.csv', 'CSV comma-separated file (*.csv)'; '*.xlsx', 'Excel workbook (*.xlsx)'}, 'Export X,Y,Z Data', default_name);
            if ~isequal(file, 0)
                try
                    writematrix(data_to_export, fullfile(path, file)); 
                    uialert(obj.Fig, sprintf('Successfully exported %d 3D coordinate points.\n\nThe file can be imported directly into Origin or Python for plotting and comparison.', size(data_to_export, 1)), 'Export Complete');
                catch ME
                    uialert(obj.Fig, ['Export failed. Check whether the file is in use: ', ME.message], 'Error');
                end
            end
        end
        
        function clearTrace(obj)
            obj.invalidateDerivedResults(); obj.renderAndResize();
            obj.UITextOut.Value = 'Trajectory/workspace data cleared.';
        end

        function invalidateDerivedResults(obj)
            obj.CurrentTrace = [];
            obj.CurrentTraceStates = [];
            obj.CurrentTraceBreakIdx = [];
            obj.CurrentWSPoints = [];
            obj.closePaperFigure();
            obj.closePlotFigure();
            obj.closeIntegratedFigure();
            if ~isempty(obj.AxWS) && isvalid(obj.AxWS), obj.showEmptyWorkspaceState(); end
            if ~isempty(obj.UITextOut) && isvalid(obj.UITextOut)
                obj.UITextOut.Value = {'The model structure or kinematic parameters have been updated.'; ...
                    'Previous workspace and dynamic trajectory data were cleared. Recalculate the workspace or rerun the sequence.'};
            end
        end

        function closePaperFigure(obj)
            old_fig = findall(0, 'Type', 'figure', 'Tag', 'OrigamiRobotPaperFigure');
            if ~isempty(old_fig), delete(old_fig); end
        end

        function closePlotFigure(obj)
            old_fig = findall(0, 'Type', 'figure', 'Tag', 'OrigamiRobotPlotFigure');
            if ~isempty(old_fig), delete(old_fig); end
        end

        function closeIntegratedFigure(obj)
            old_fig = findall(0, 'Type', 'figure', 'Tag', 'OrigamiRobotIntegratedFigure');
            if ~isempty(old_fig), delete(old_fig); end
        end

        function resetAllModuleStates(obj)
            for m = 1:length(obj.RobotAssembly)
                obj.RobotAssembly{m}.contraction_pct = 0;
            end
        end

        function onKinematicsChanged(obj)
            obj.invalidateDerivedResults();
            obj.ViewEnvelopeDirty = true;
            obj.renderAndResize();
            % Refresh the pressure-to-angle/length curve if geometry parameters change.
            if ~isempty(obj.RobotAssembly)
                obj.onListSelect();
            end
        end

        function showEmptyWorkspaceState(obj)
            colorbar(obj.AxWS, 'off');
            obj.AxWS.UserData = [];
            cla(obj.AxWS, 'reset');
            hold(obj.AxWS, 'on'); view(obj.AxWS, 2); axis(obj.AxWS, 'off');
            xlim(obj.AxWS, [0 1]); ylim(obj.AxWS, [0 1]);
            zlim(obj.AxWS, [0 1]);
            obj.AxWS.Color = [0.992 0.992 0.988];
            text(obj.AxWS, 0.5, 0.54, 'Workspace / Trajectory', ...
                'HorizontalAlignment', 'center', 'FontName', 'Arial', ...
                'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.34 0.38 0.40]);
            text(obj.AxWS, 0.5, 0.46, 'Calculate the theoretical workspace or run a tip trajectory', ...
                'HorizontalAlignment', 'center', 'FontName', 'Arial', ...
                'FontSize', 11, 'Color', [0.55 0.58 0.60]);
            hold(obj.AxWS, 'off');
        end
        
        function renderAndResize(obj), obj.render3D(); drawnow; end
        function selectAllModules(obj), if isempty(obj.RobotAssembly), return; end, obj.UIList.Value = obj.UIList.Items; obj.onListSelect(); end
        
        function [ang, curr_p] = getInterpolatedAngle(obj, mod, pct_val)
            % Preserve the old 0-100 sequence interface, but use the calibrated
            % pressure-to-angle equation rather than a fitted polynomial.
            curr_p = mod.p_start + (pct_val / 100) * (mod.p_end - mod.p_start);
            curr_p = max(-101.3, min(0, curr_p));
            [ang, ~, ~] = obj.pressureToB1B3L8(curr_p);
        end
        
        function calcWorkspaceMC(obj)
            M = length(obj.RobotAssembly); if M == 0, uialert(obj.Fig, 'Add at least one module first.', 'Notice'); return; end
            obj.closePaperFigure();
            obj.closePlotFigure();
            obj.closeIntegratedFigure();
            delete(obj.AxWS);
            pnlWS = findobj(obj.Fig, 'Title', 'Decoupled Tip Analysis (Trajectory & Workspace)'); 
            obj.AxWS = uiaxes(pnlWS, 'Position', [20 350 440 370]); hold(obj.AxWS, 'on');
            
            track_val = obj.UI_TrackDropdown.Value; track_m = M; track_is_top = true;
            if ~strcmp(track_val, 'Global Tip (Default)')
                tokens = regexp(track_val, 'Module(\d+) - (.*) Plate', 'tokens');
                if ~isempty(tokens), track_m = str2double(tokens{1}{1}); track_is_top = strcmp(tokens{1}{2}, 'Top'); end
            end
            if track_is_top, M_eff = track_m; else, M_eff = track_m - 1; end
            
            if M_eff == 0
                uialert(obj.Fig, 'No movable module exists above the selected tracking node.', 'Notice'); return;
            end
            
            obj.UITextOut.Value = 'Running full-arm Monte Carlo sampling (50,000 points)...\nComputing FK matrices and voxel-based integration...'; drawnow;
            
            tpad = obj.readNumericFieldValue(obj.UI_TPad); tskel = obj.readNumericFieldValue(obj.UI_TSkel); 
            res = 100; 
            LUT = cell(1, max(1, M_eff));
            
            for j = 1:M_eff
                mod = obj.RobotAssembly{j};
                pcts = linspace(0, 100, res); 
                s_count = length(pcts);
                LUT{j} = zeros(4, 4, s_count);
                for s = 1:s_count
                    ang = obj.getInterpolatedAngle(mod, pcts(s)); 
                    T_mod = obj.rotz(mod.inst_rot) * obj.rotx(mod.inst_pitch) * obj.transz(tpad);
                    for k = 1:mod.n, if k > 1, T_mod = T_mod * obj.rotz(mod.seq(k-1)); end; T_mod = T_mod * obj.getUnitTransform(ang, tskel); end
                    T_mod = T_mod * obj.transz(tpad); 
                    LUT{j}(:,:,s) = T_mod; 
                end
            end
            
            if ~track_is_top && track_m > 0, mod_target = obj.RobotAssembly{track_m}; T_base_offset = obj.rotz(mod_target.inst_rot) * obj.rotx(mod_target.inst_pitch) * obj.transz(tpad); else, T_base_offset = eye(4); end
            T_local_offset = obj.transx(obj.readNumericFieldValue(obj.UI_EE_X)) * obj.transy(obj.readNumericFieldValue(obj.UI_EE_Y)) * obj.transz(obj.readNumericFieldValue(obj.UI_EE_Z));
            
            N_samples = 50000;
            end_points = zeros(N_samples, 3);
            
            for i = 1:N_samples
                T_curr = eye(4); 
                for j = 1:M_eff
                    idx = randi(res); 
                    T_curr = T_curr * LUT{j}(:, :, idx); 
                end
                T_curr = T_curr * T_base_offset * T_local_offset; 
                end_points(i, :) = T_curr(1:3, 4)'; 
            end
            
            obj.CurrentWSPoints = end_points; 
            
            min_x = min(end_points(:,1)); max_x = max(end_points(:,1)); range_x = max_x - min_x;
            min_y = min(end_points(:,2)); max_y = max(end_points(:,2)); range_y = max_y - min_y;
            min_z = min(end_points(:,3)); max_z = max(end_points(:,3)); range_z = max_z - min_z;
            
            tol = 1e-2;
            is_x_const = range_x < tol; is_y_const = range_y < tol; is_z_const = range_z < tol; 
            is_2d_plot = is_x_const || is_y_const || is_z_const;
            
            if is_2d_plot
                voxel_size = 2.0; 
                if is_x_const, voxels = floor(end_points(:, 2:3) / voxel_size);
                elseif is_y_const, voxels = floor(end_points(:, [1,3]) / voxel_size);
                else, voxels = floor(end_points(:, 1:2) / voxel_size); end
                unique_voxels = unique(voxels, 'rows');
                mc_volume = size(unique_voxels, 1) * (voxel_size^2);
                vol_unit = 'mm² (planar area)';
            else
                voxel_size = 3.0; 
                voxels = floor(end_points / voxel_size);
                unique_voxels = unique(voxels, 'rows');
                mc_volume = size(unique_voxels, 1) * (voxel_size^3);
                vol_unit = 'mm³ (spatial volume)';
            end
            
            style = obj.getPaperPalette();
            global_max_span = max([range_x, range_y, range_z, 20]); half_span = global_max_span / 2 + global_max_span * 0.15; 
            
            if is_2d_plot
                if is_x_const, X_data = end_points(:,2); Y_data = end_points(:,3); strX = 'Y-axis (mm)'; strY = 'Z-axis (mm)'; dim_name = 'Y-Z';
                elseif is_y_const, X_data = end_points(:,1); Y_data = end_points(:,3); strX = 'X-axis (mm)'; strY = 'Z-axis (mm)'; dim_name = 'X-Z';
                else, X_data = end_points(:,1); Y_data = end_points(:,2); strX = 'X-axis (mm)'; strY = 'Y-axis (mm)'; dim_name = 'X-Y'; end
                
                scatter(obj.AxWS, X_data, Y_data, 7, end_points(:,3), 'filled', ...
                    'MarkerFaceAlpha', 0.34, 'MarkerEdgeColor', 'none'); 
                
                if ~isempty(obj.CurrentTrace)
                    if is_x_const
                        trace_x = obj.CurrentTrace(:,2); trace_y = obj.CurrentTrace(:,3);
                    elseif is_y_const
                        trace_x = obj.CurrentTrace(:,1); trace_y = obj.CurrentTrace(:,3);
                    else
                        trace_x = obj.CurrentTrace(:,1); trace_y = obj.CurrentTrace(:,2);
                    end
                    obj.plotHeightGradientLine2D(obj.AxWS, trace_x, trace_y, ...
                        obj.CurrentTrace(:,3), 3.0);
                    scatter(obj.AxWS, trace_x(1), trace_y(1), 60, 'k', 'filled', ...
                        'MarkerEdgeColor', 'w', 'LineWidth', 0.7);
                    scatter(obj.AxWS, trace_x(end), trace_y(end), 100, ...
                        obj.CurrentTrace(end,3), 'filled', 'Marker', 'p', ...
                        'MarkerEdgeColor', style.edge, 'LineWidth', 0.8);
                end
                
                grid(obj.AxWS, 'on'); view(obj.AxWS, 2); xlabel(obj.AxWS, strX, 'FontWeight', 'bold'); ylabel(obj.AxWS, strY, 'FontWeight', 'bold'); obj.AxWS.ZAxis.Visible = 'off';
                cx = (max(X_data) + min(X_data)) / 2; cy = (max(Y_data) + min(Y_data)) / 2; 
                xlim(obj.AxWS, [cx - half_span, cx + half_span]); ylim(obj.AxWS, [cy - half_span, cy + half_span]); pbaspect(obj.AxWS, [1 1 1]); 
                set(obj.AxWS, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.8, 'Box', 'on', 'Color', [1 1 1]);
                obj.AxWS.GridColor = [0.76 0.80 0.84]; obj.AxWS.GridAlpha = 0.22;
                title(obj.AxWS, sprintf('Monte Carlo Workspace and Trajectory (Auto-projected: %s plane)', dim_name), 'FontName', 'Arial', 'FontSize', 13);
                color_values = end_points(:,3);
                if ~isempty(obj.CurrentTrace), color_values = [color_values; obj.CurrentTrace(:,3)]; end
                obj.applyHeightColorScale(obj.AxWS, color_values);
                disableDefaultInteractivity(obj.AxWS); axtoolbar(obj.AxWS, {'pan', 'zoomin', 'zoomout', 'restoreview'});
            else
                obj.AxWS.ZAxis.Visible = 'on'; 
                scatter3(obj.AxWS, end_points(:,1), end_points(:,2), end_points(:,3), ...
                    7, end_points(:,3), 'filled', 'MarkerFaceAlpha', 0.30, ...
                    'MarkerEdgeColor', 'none'); 
                
                if ~isempty(obj.CurrentTrace)
                    obj.plotSegmentedHeightLine3D(obj.AxWS, obj.CurrentTrace, 3.5);
                    scatter3(obj.AxWS, obj.CurrentTrace(1,1), obj.CurrentTrace(1,2), ...
                        obj.CurrentTrace(1,3), 60, 'k', 'filled', ...
                        'MarkerEdgeColor', 'w', 'LineWidth', 0.7);
                    scatter3(obj.AxWS, obj.CurrentTrace(end,1), obj.CurrentTrace(end,2), ...
                        obj.CurrentTrace(end,3), 100, obj.CurrentTrace(end,3), ...
                        'filled', 'Marker', 'p', 'MarkerEdgeColor', style.edge, 'LineWidth', 0.8);
                end
                
                view(obj.AxWS, 38, 20); grid(obj.AxWS, 'on'); xlabel(obj.AxWS, 'X (mm)'); ylabel(obj.AxWS, 'Y (mm)'); zlabel(obj.AxWS, 'Z (mm)');
                cx = (max_x + min_x)/2; cy = (max_y + min_y)/2; cz = (max_z + min_z)/2; 
                xlim(obj.AxWS, [cx - half_span, cx + half_span]); ylim(obj.AxWS, [cy - half_span, cy + half_span]); zlim(obj.AxWS, [cz - half_span, cz + half_span]); pbaspect(obj.AxWS, [1 1 1]); 
                set(obj.AxWS, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.8, 'Box', 'on', 'Projection', 'orthographic', 'Color', [1 1 1]);
                obj.AxWS.GridColor = [0.76 0.80 0.84]; obj.AxWS.GridAlpha = 0.22;
                title(obj.AxWS, sprintf('3D Workspace and Dynamic Trajectory (Samples: %d)', N_samples), 'FontName', 'Arial', 'FontSize', 13);
                color_values = end_points(:,3);
                if ~isempty(obj.CurrentTrace), color_values = [color_values; obj.CurrentTrace(:,3)]; end
                obj.applyHeightColorScale(obj.AxWS, color_values);
                enableDefaultInteractivity(obj.AxWS);
            end
            
            out_str = sprintf('Monte Carlo full-arm exploration completed.\n\n');
            out_str = [out_str, sprintf('[ Workspace Bounds ]\n X: %.2f ~ %.2f mm\n Y: %.2f ~ %.2f mm\n Z: %.2f ~ %.2f mm\n\n', min_x, max_x, min_y, max_y, min_z, max_z)];
            out_str = [out_str, sprintf('[ Voxel Integration ]\n Samples: %d\n Workspace measure: %.1f %s\n', N_samples, mc_volume, vol_unit)];
            
            % Append the kinematic feature report
            out_str = [out_str, obj.generateModuleBoundsReport()];
            obj.UITextOut.Value = out_str;
        end

        function runProgram(obj)
            if isempty(obj.RobotAssembly), uialert(obj.Fig, 'The robot assembly is empty. Add at least one module first.', 'Notice'); return; end
            
            lines = obj.UIProgram.Value; if ischar(lines), lines = cellstr(lines); end
            num_cycles = 1; valid_lines = {};
            for i = 1:length(lines)
                str = strtrim(lines{i});
                if isempty(str) || startsWith(str, '%') || startsWith(str, '#'), continue; end
                if startsWith(lower(str), 'cycle')
                    tok = regexp(str, 'cycle\s*(\d+)', 'tokens', 'ignorecase');
                    if ~isempty(tok), num_cycles = str2double(tok{1}{1}); end
                    continue; 
                end
                valid_lines{end+1} = str;
            end
            
            obj.closePaperFigure();
            obj.closePlotFigure();
            obj.closeIntegratedFigure();
            if isempty(obj.UIStartFromCurrent) || ~isvalid(obj.UIStartFromCurrent) || ~obj.UIStartFromCurrent.Value
                obj.resetAllModuleStates();
            end
            obj.CurrentTrace = [];
            obj.CurrentTraceStates = [];
            obj.CurrentTraceBreakIdx = [];
            obj.CurrentTrace = obj.getTrackedPosition()';
            obj.CurrentTraceStates = obj.getCurrentState();
            obj.UITextOut.Value = 'Executing the motion sequence and recording the tip trajectory...';
            
            for cyc = 1:num_cycles
                for i = 1:length(valid_lines)
                    str = valid_lines{i}; cmds = strsplit(str, ';'); tasks = {}; max_time = 0;
                    for c = 1:length(cmds)
                        cmdStr = strtrim(cmds{c}); if isempty(cmdStr), continue; end
                        parts = strsplit(cmdStr, ',');
                        
                        t_name_raw = strtrim(parts{1});
                        is_hold = startsWith(t_name_raw, 'k+', 'IgnoreCase', true);

                        if is_hold
                            t_name = strtrim(t_name_raw(3:end));
                            if length(parts) < 2
                                uialert(obj.Fig, sprintf('Syntax error: "%s". Use k+module,P:-30 for a pressure hold.', cmdStr), 'Error'); return;
                            end
                            t_target_token = strtrim(parts{2});
                            t_time = 0;
                        else
                            t_name = t_name_raw;
                            if length(parts) ~= 3
                                uialert(obj.Fig, sprintf('Syntax error: "%s". Use module,P:-30,time_s (or legacy module,state,time).', cmdStr), 'Error'); return;
                            end
                            t_target_token = strtrim(parts{2});
                            t_time = str2double(strtrim(parts{3}));
                            if ~isfinite(t_time) || t_time < 0
                                uialert(obj.Fig, sprintf('Invalid duration in command: "%s". Duration must be nonnegative.', cmdStr), 'Error'); return;
                            end
                        end

                        found = false;
                        for m = 1:length(obj.RobotAssembly)
                            if strcmp(obj.RobotAssembly{m}.name, t_name)
                                curr_mod = obj.RobotAssembly{m};
                                [target_pct, target_pressure, target_ok, target_message] = ...
                                    obj.parseProgramTarget(curr_mod, t_target_token);
                                if ~target_ok
                                    uialert(obj.Fig, sprintf('Invalid target in "%s": %s', cmdStr, target_message), 'Error');
                                    return;
                                end
                                task.idx = m;
                                task.target_pct = target_pct;
                                task.target_pressure = target_pressure;
                                task.time = t_time;
                                tasks{end+1} = task;
                                found = true;
                                break;
                            end
                        end
                        if ~found, uialert(obj.Fig, sprintf('Module "%s" was not found.', t_name), 'Error'); return; end
                    end
                    
                    if ~isempty(tasks)
                        % Group commands by module while preserving their written order.
                        % Repeated commands for one module form a sequential pressure ramp;
                        % different modules still execute concurrently.
                        groups = {};
                        for k = 1:numel(tasks)
                            tk = tasks{k};
                            gidx = [];
                            for gg = 1:numel(groups)
                                if groups{gg}.idx == tk.idx
                                    gidx = gg;
                                    break;
                                end
                            end
                            if isempty(gidx)
                                mod_now = obj.RobotAssembly{tk.idx};
                                start_pressure = mod_now.p_start + ...
                                    (mod_now.contraction_pct/100) * (mod_now.p_end-mod_now.p_start);
                                new_group = struct('idx', tk.idx, 'start_pressure', start_pressure, ...
                                    'commands', {{}}, 'total_time', 0);
                                groups{end+1} = new_group; %#ok<AGROW>
                                gidx = numel(groups);
                            end
                            groups{gidx}.commands{end+1} = tk;
                            groups{gidx}.total_time = groups{gidx}.total_time + max(0, tk.time);
                        end

                        max_time = 0;
                        for gg = 1:numel(groups)
                            max_time = max(max_time, groups{gg}.total_time);
                        end

                        % Apply only zero-time commands at their scheduled point (usually t=0).
                        % Any instantaneous configuration change is separated in the plotted trace.
                        had_instant_jump = false;
                        for gg = 1:numel(groups)
                            idx_mod = groups{gg}.idx;
                            old_pct = obj.RobotAssembly{idx_mod}.contraction_pct;
                            p_at_zero = obj.evaluatePressureChain(groups{gg}.start_pressure, ...
                                groups{gg}.commands, 0);
                            new_pct = obj.pressureToPct(obj.RobotAssembly{idx_mod}, p_at_zero);
                            obj.RobotAssembly{idx_mod}.contraction_pct = new_pct;
                            if abs(new_pct-old_pct) > 1e-9
                                had_instant_jump = true;
                            end
                        end

                        if had_instant_jump
                            curr_tip = obj.getTrackedPosition();
                            obj.CurrentTrace(end+1, :) = curr_tip';
                            obj.CurrentTraceStates(end+1, :) = obj.getCurrentState();
                            obj.CurrentTraceBreakIdx(end+1) = size(obj.CurrentTrace,1);
                            obj.render3D();
                            obj.syncSlider();
                            drawnow limitrate nocallbacks;
                        end

                        if max_time <= 0
                            continue;
                        end

                        steps = max(2, round(max_time * obj.SimulationHz));
                        render_interval = max(1, round(obj.SimulationHz / obj.AnimationFPS));
                        line_clock = tic;
                        for s = 1:steps
                            prog = s / steps;
                            elapsed_time = prog * max_time;
                            for gg = 1:numel(groups)
                                idx_mod = groups{gg}.idx;
                                curr_mod = obj.RobotAssembly{idx_mod};
                                p_now = obj.evaluatePressureChain(groups{gg}.start_pressure, ...
                                    groups{gg}.commands, elapsed_time);
                                obj.RobotAssembly{idx_mod}.contraction_pct = ...
                                    obj.pressureToPct(curr_mod, p_now);
                            end

                            curr_tip = obj.getTrackedPosition();
                            obj.CurrentTrace(end+1, :) = curr_tip';
                            obj.CurrentTraceStates(end+1, :) = obj.getCurrentState();

                            if mod(s, render_interval) == 0 || s == steps
                                obj.render3D();
                                obj.syncSlider();
                                drawnow limitrate nocallbacks;

                                % Preserve the requested real-time duration for recording.
                                if obj.RealTimePlayback && max_time > 0
                                    target_elapsed = prog * max_time;
                                    remain = target_elapsed - toc(line_clock);
                                    if remain > 0.002
                                        pause(remain);
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            obj.plotTrajectoryToWS(obj.CurrentTrace);
        end
        
        function tip_p = getTrackedPosition(obj)
            T_curr = eye(4); tpad = obj.readNumericFieldValue(obj.UI_TPad); tskel = obj.readNumericFieldValue(obj.UI_TSkel);
            T_track = eye(4); track_val = obj.UI_TrackDropdown.Value;
            for m = 1:length(obj.RobotAssembly)
                curr_mod = obj.RobotAssembly{m};
                T_curr = T_curr * obj.rotz(curr_mod.inst_rot) * obj.rotx(curr_mod.inst_pitch) * obj.transz(tpad); 
                if strcmp(track_val, sprintf('Module%d - Base Plate', m)), T_track = T_curr; end
                ang = obj.getInterpolatedAngle(curr_mod, curr_mod.contraction_pct);
                for i = 1:curr_mod.n
                    if i > 1, T_curr = T_curr * obj.rotz(curr_mod.seq(i-1)); end
                    T_curr = T_curr * obj.getUnitTransform(ang, tskel); 
                end
                T_curr = T_curr * obj.transz(tpad);
                if strcmp(track_val, sprintf('Module%d - Top Plate', m)), T_track = T_curr; end
            end
            if strcmp(track_val, 'Global Tip (Default)') || isempty(obj.RobotAssembly), T_track = T_curr; end
            T_local_offset = obj.transx(obj.readNumericFieldValue(obj.UI_EE_X)) * obj.transy(obj.readNumericFieldValue(obj.UI_EE_Y)) * obj.transz(obj.readNumericFieldValue(obj.UI_EE_Z));
            T_track = T_track * T_local_offset; tip_p = T_track(1:3,4); 
        end

        function [seg_start, seg_end] = getTraceSegments(obj, n_pts)
            if nargin < 2, n_pts = size(obj.CurrentTrace,1); end
            breaks = unique(obj.CurrentTraceBreakIdx(:)');
            breaks = breaks(breaks >= 2 & breaks <= n_pts);
            seg_start = [1, breaks];
            seg_end = [breaks-1, n_pts];
            valid = seg_end >= seg_start;
            seg_start = seg_start(valid);
            seg_end = seg_end(valid);
        end

        function [xv, yv, zv] = buildSegmentedTraceVectors(obj, pts)
            if isempty(pts)
                xv = []; yv = []; zv = [];
                return;
            end
            [seg_start, seg_end] = obj.getTraceSegments(size(pts,1));
            xv = []; yv = []; zv = [];
            for ii = 1:numel(seg_start)
                idx = seg_start(ii):seg_end(ii);
                xv = [xv; pts(idx,1); NaN]; %#ok<AGROW>
                yv = [yv; pts(idx,2); NaN]; %#ok<AGROW>
                zv = [zv; pts(idx,3); NaN]; %#ok<AGROW>
            end
            if ~isempty(xv)
                xv(end) = []; yv(end) = []; zv(end) = [];
            end
        end

        function plotSegmentedHeightLine2D(obj, Ax, pts3, x_data, y_data, line_width)
            [seg_start, seg_end] = obj.getTraceSegments(size(pts3,1));
            for ii = 1:numel(seg_start)
                idx = seg_start(ii):seg_end(ii);
                if numel(idx) >= 2
                    obj.plotHeightGradientLine2D(Ax, x_data(idx), y_data(idx), pts3(idx,3), line_width);
                elseif numel(idx) == 1
                    scatter(Ax, x_data(idx), y_data(idx), 18, pts3(idx,3), 'filled', 'HandleVisibility', 'off');
                end
            end
        end

        function plotSegmentedHeightLine3D(obj, Ax, pts3, line_width)
            [seg_start, seg_end] = obj.getTraceSegments(size(pts3,1));
            for ii = 1:numel(seg_start)
                idx = seg_start(ii):seg_end(ii);
                if numel(idx) >= 2
                    obj.plotHeightGradientLine3D(Ax, pts3(idx,1), pts3(idx,2), pts3(idx,3), line_width);
                elseif numel(idx) == 1
                    scatter3(Ax, pts3(idx,1), pts3(idx,2), pts3(idx,3), 18, pts3(idx,3), 'filled', 'HandleVisibility', 'off');
                end
            end
        end
        
        function plotTrajectoryToWS(obj, pts)
            colorbar(obj.AxWS, 'off');
            obj.AxWS.UserData = [];
            cla(obj.AxWS); axis(obj.AxWS, 'on'); hold(obj.AxWS, 'on');
            if isempty(pts), return; end

            pts_valid = pts(all(isfinite(pts),2), :);
            if isempty(pts_valid), return; end
            
            min_x = min(pts_valid(:,1)); max_x = max(pts_valid(:,1)); range_x = max_x - min_x;
            min_y = min(pts_valid(:,2)); max_y = max(pts_valid(:,2)); range_y = max_y - min_y;
            min_z = min(pts_valid(:,3)); max_z = max(pts_valid(:,3)); range_z = max_z - min_z;
            
            tol = 1e-3;
            is_x_const = range_x < tol; is_y_const = range_y < tol; is_z_const = range_z < tol; 
            is_2d_plot = is_x_const || is_y_const || is_z_const;
            
            style = obj.getPaperPalette();
            
            if is_2d_plot
                if is_x_const, X_data = pts_valid(:,2); Y_data = pts_valid(:,3); strX = 'Y-axis (mm)'; strY = 'Z-axis (mm)'; dim_name = 'Y-Z';
                elseif is_y_const, X_data = pts_valid(:,1); Y_data = pts_valid(:,3); strX = 'X-axis (mm)'; strY = 'Z-axis (mm)'; dim_name = 'X-Z';
                else, X_data = pts_valid(:,1); Y_data = pts_valid(:,2); strX = 'X-axis (mm)'; strY = 'Y-axis (mm)'; dim_name = 'X-Y'; end
                
                obj.plotSegmentedHeightLine2D(obj.AxWS, pts_valid, X_data, Y_data, 2.8);
                scatter(obj.AxWS, X_data(1), Y_data(1), 60, 'k', 'filled', ...
                    'MarkerEdgeColor', 'w', 'LineWidth', 0.7); 
                scatter(obj.AxWS, X_data(end), Y_data(end), 100, pts_valid(end,3), ...
                    'filled', 'Marker', 'p', 'MarkerEdgeColor', style.edge, 'LineWidth', 0.8); 
                
                grid(obj.AxWS, 'on'); view(obj.AxWS, 2); 
                xlabel(obj.AxWS, strX, 'FontWeight', 'bold'); ylabel(obj.AxWS, strY, 'FontWeight', 'bold'); obj.AxWS.ZAxis.Visible = 'off';
                
                span_x = max(X_data) - min(X_data); span_y = max(Y_data) - min(Y_data);
                max_span = max([span_x, span_y, 20]); half_span = max_span * 0.5 + max_span * 0.15;
                cx = (max(X_data)+min(X_data))/2; cy = (max(Y_data)+min(Y_data))/2;
                
                xlim(obj.AxWS, [cx - half_span, cx + half_span]); ylim(obj.AxWS, [cy - half_span, cy + half_span]); 
                pbaspect(obj.AxWS, [1 1 1]);
                set(obj.AxWS, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.8, 'Box', 'on', 'Color', [1 1 1]);
                obj.AxWS.GridColor = [0.76 0.80 0.84]; obj.AxWS.GridAlpha = 0.22;
                title(obj.AxWS, sprintf('Tip Kinematic Trajectory (Auto-projected: %s plane)', dim_name), 'FontName', 'Arial', 'FontSize', 13);
                obj.applyHeightColorScale(obj.AxWS, pts_valid(:,3));
                
                disableDefaultInteractivity(obj.AxWS); axtoolbar(obj.AxWS, {'pan', 'zoomin', 'zoomout', 'restoreview'});
                disp_type = '2D planar projection';
            else
                obj.AxWS.ZAxis.Visible = 'on';
                obj.plotSegmentedHeightLine3D(obj.AxWS, pts_valid, 2.8);
                scatter3(obj.AxWS, pts_valid(1,1), pts_valid(1,2), pts_valid(1,3), 60, 'k', 'filled', ...
                    'MarkerEdgeColor', 'w', 'LineWidth', 0.7); 
                scatter3(obj.AxWS, pts_valid(end,1), pts_valid(end,2), pts_valid(end,3), 100, pts_valid(end,3), ...
                    'filled', 'Marker', 'p', 'MarkerEdgeColor', style.edge, 'LineWidth', 0.8); 
                
                minW = min(pts_valid); maxW = max(pts_valid); span = max(maxW - minW); half_span = max(span * 0.6, 20);
                cx = (maxW(1)+minW(1))/2; cy = (maxW(2)+minW(2))/2; cz = (maxW(3)+minW(3))/2;
                
                grid(obj.AxWS, 'on'); view(obj.AxWS, 38, 20); 
                xlabel(obj.AxWS, 'X (mm)'); ylabel(obj.AxWS, 'Y (mm)'); zlabel(obj.AxWS, 'Z (mm)');
                xlim(obj.AxWS, [cx - half_span, cx + half_span]); ylim(obj.AxWS, [cy - half_span, cy + half_span]); zlim(obj.AxWS, [cz - half_span, cz + half_span]); 
                pbaspect(obj.AxWS, [1 1 1]);
                set(obj.AxWS, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.8, 'Box', 'on', 'Projection', 'orthographic', 'Color', [1 1 1]);
                obj.AxWS.GridColor = [0.76 0.80 0.84]; obj.AxWS.GridAlpha = 0.22;
                title(obj.AxWS, 'Continuous Tip Kinematic Trajectory (3D)', 'FontName', 'Arial', 'FontSize', 13);
                obj.applyHeightColorScale(obj.AxWS, pts_valid(:,3));
                enableDefaultInteractivity(obj.AxWS);
                disp_type = '3D spatial trajectory';
            end
            
            out_str = sprintf('Tip kinematic trajectory generated. [%s]\nInterpolated points: %d\nFinal position: X=%.2f, Y=%.2f, Z=%.2f\n', disp_type, size(pts,1), pts(end,1), pts(end,2), pts(end,3));
            
            % Append the kinematic feature report
            out_str = [out_str, obj.generateModuleBoundsReport()];
            obj.UITextOut.Value = out_str;
        end

        function cmap = getHeightColormap(obj, color_count)
            if nargin < 2, color_count = 256; end
            anchors = [ ...
                0.10 0.25 0.29; ...  % lower range: deep teal
                0.08 0.47 0.48; ...
                0.27 0.67 0.53; ...
                0.80 0.77 0.36; ...
                0.91 0.43 0.20];     % upper range: warm orange
            anchor_pos = linspace(0, 1, size(anchors, 1));
            query_pos = linspace(0, 1, color_count);
            cmap = interp1(anchor_pos, anchors, query_pos, 'linear');
            cmap = max(0, min(1, cmap));
        end

        function applyHeightColorScale(obj, Ax, color_values)
            valid_values = color_values(isfinite(color_values));
            if isempty(valid_values), return; end
            z_min = min(valid_values); z_max = max(valid_values);
            if abs(z_max - z_min) < 1e-9
                pad = max(1, abs(z_min) * 0.02);
                z_min = z_min - pad; z_max = z_max + pad;
            end
            colormap(Ax, obj.getHeightColormap(256));
            caxis(Ax, [z_min z_max]);
            cb = colorbar(Ax, 'eastoutside');
            cb.Label.String = 'Z height (mm)';
            cb.Label.FontName = 'Arial'; cb.Label.FontSize = 10;
            cb.FontName = 'Arial'; cb.FontSize = 9;
            cb.Color = [0.22 0.25 0.27]; cb.Box = 'off';
            Ax.UserData = struct('GradientColorbar', true, ...
                'ColorbarLabel', 'Z height (mm)');
        end

        function plotHeightGradientLine2D(obj, Ax, x_data, y_data, z_color, line_width)
            x_data = x_data(:); y_data = y_data(:); z_color = z_color(:);
            if numel(x_data) < 2
                scatter(Ax, x_data, y_data, 26, z_color, 'filled'); return;
            end
            surface(Ax, [x_data x_data], [y_data y_data], zeros(numel(x_data), 2), ...
                [z_color z_color], 'FaceColor', 'none', 'EdgeColor', 'interp', ...
                'LineWidth', line_width, 'HandleVisibility', 'off');
        end

        function plotHeightGradientLine3D(obj, Ax, x_data, y_data, z_data, line_width)
            x_data = x_data(:); y_data = y_data(:); z_data = z_data(:);
            if numel(x_data) < 2
                scatter3(Ax, x_data, y_data, z_data, 26, z_data, 'filled'); return;
            end
            surface(Ax, [x_data x_data], [y_data y_data], [z_data z_data], ...
                [z_data z_data], 'FaceColor', 'none', 'EdgeColor', 'interp', ...
                'LineWidth', line_width, 'HandleVisibility', 'off');
        end

        function syncSlider(obj)
            sel = obj.UIList.Value;
            if isempty(obj.RobotAssembly) || isempty(sel) || (ischar(sel) && strcmp(sel, '(Empty)')), return; end
            if ~iscell(sel), sel = {sel}; end
            idx = find(strcmp(obj.UIList.Items, sel{1}), 1);
            if isempty(idx), return; end

            curr_mod = obj.RobotAssembly{idx};
            pct = curr_mod.contraction_pct;
            [ang, curr_p] = obj.getInterpolatedAngle(curr_mod, pct);
            obj.configurePressureControls(curr_mod, curr_p, ang, pct);
        end

        function toggleModelDetails(obj)
            obj.UIModelDetailsExpanded = ~obj.UIModelDetailsExpanded;
            if obj.UIModelDetailsExpanded
                obj.UIPanelModelDetails.Visible = 'on';
                obj.UIModelDetailsButton.Text = '▼ Hide Theoretical Model';
            else
                obj.UIPanelModelDetails.Visible = 'off';
                obj.UIModelDetailsButton.Text = '▶ Show Theoretical Model';
            end
            drawnow limitrate;
        end

        function [target_pct, target_pressure, ok, message] = parseProgramTarget(obj, mod, token)
            % Preferred syntax is P:-30 or -30 kPa. Legacy state syntax remains supported.
            token = strtrim(char(token));
            token_low = lower(strtrim(token));
            target_pct = NaN;
            target_pressure = NaN;
            ok = false;
            message = '';

            is_pressure = false;
            pressure_text = '';
            if startsWith(token_low, 'p:') || startsWith(token_low, 'p=')
                pressure_text = strtrim(token(3:end));
                is_pressure = true;
            else
                tok = regexp(token_low, '^(-?(?:\d+\.?\d*|\.\d+))\s*kpa$', 'tokens', 'once');
                if ~isempty(tok)
                    pressure_text = tok{1};
                    is_pressure = true;
                end
            end

            if is_pressure
                p_cmd = str2double(pressure_text);
                if ~isfinite(p_cmd)
                    message = 'Pressure target must be numeric, e.g., P:-30.';
                    return;
                end
                p_min = min(mod.p_start, mod.p_end);
                p_max = max(mod.p_start, mod.p_end);
                if p_cmd < p_min-1e-9 || p_cmd > p_max+1e-9
                    message = sprintf('Pressure %.3f kPa is outside this module range [%.3f, %.3f] kPa.', ...
                        p_cmd, p_min, p_max);
                    return;
                end
                target_pressure = p_cmd;
                target_pct = obj.pressureToPct(mod, p_cmd);
                ok = true;
                return;
            end

            % Optional explicit percentage syntax: S:50 or 50%.
            if startsWith(token_low, 's:')
                pct_value = str2double(strtrim(token(3:end)));
            elseif endsWith(token_low, '%')
                pct_value = str2double(strtrim(token(1:end-1)));
            else
                state_value = str2double(token);
                if ~isfinite(state_value)
                    message = 'Use P:<kPa> (e.g., P:-30), S:50, or a legacy numeric state 0/1.';
                    return;
                end
                if state_value >= 0 && state_value <= 1
                    pct_value = 100*state_value;
                elseif state_value > 1 && state_value <= 100
                    pct_value = state_value;
                else
                    message = 'State must be between 0 and 1, or a percentage between 0 and 100.';
                    return;
                end
            end

            if ~isfinite(pct_value) || pct_value < 0 || pct_value > 100
                message = 'State percentage must be between 0 and 100.';
                return;
            end
            target_pct = pct_value;
            target_pressure = mod.p_start + (target_pct/100)*(mod.p_end-mod.p_start);
            ok = true;
        end

        function pct = pressureToPct(obj, mod, pressure_val)
            den = mod.p_end - mod.p_start;
            if abs(den) < 1e-12
                pct = 0;
            else
                pct = 100 * (pressure_val - mod.p_start) / den;
            end
            pct = max(0, min(100, pct));
        end

        function pressure_now = evaluatePressureChain(obj, start_pressure, commands, elapsed_time)
            % Evaluate a module's ordered pressure commands at a common line-clock time.
            % Positive-duration targets use a monotonic cubic smoothstep in pressure space;
            % zero-duration hold commands are applied at their position in the command chain.
            pressure_now = start_pressure;
            cursor_time = 0;
            for jj = 1:numel(commands)
                cmd = commands{jj};
                dt = max(0, cmd.time);
                target_p = cmd.target_pressure;
                if dt <= 0
                    if elapsed_time + 1e-10 >= cursor_time
                        pressure_now = target_p;
                    else
                        return;
                    end
                else
                    if elapsed_time < cursor_time
                        return;
                    elseif elapsed_time < cursor_time + dt
                        tau = max(0, min(1, (elapsed_time-cursor_time)/dt));
                        % Cubic smoothstep gives zero velocity at both ends without overshoot.
                        alpha = tau*tau*(3 - 2*tau);
                        pressure_now = pressure_now + (target_p-pressure_now)*alpha;
                        return;
                    else
                        pressure_now = target_p;
                        cursor_time = cursor_time + dt;
                    end
                end
            end
        end

        function [y, y_label, mode_label, theta_deg, stroke_mm, actual_length_mm] = getMappingOutput(obj, mod, pressure_kpa, theta_in, stroke_in)
            % One calibrated pressure-angle law drives every module. The configured
            % stack sequence determines whether the output is bend angle or axial length.
            if nargin < 4 || isempty(theta_in)
                [theta_deg, ~, ~] = obj.pressureToB1B3L8(pressure_kpa);
            else
                theta_deg = theta_in;
            end
            theta_deg = double(theta_deg);
            actual_length_mm = NaN(size(theta_deg));
            stroke_mm = NaN(size(theta_deg));

            if obj.isStraightContractionModule(mod)
                actual_length_mm = obj.getModuleLengthAtAngle(mod, theta_deg);
                L0 = obj.getModuleLengthAtAngle(mod, 45 .* ones(size(theta_deg)));
                stroke_mm = L0 - actual_length_mm;
                y = actual_length_mm;
                y_label = 'Actual axial length (mm)';
                mode_label = 'axial length (mm)';
            else
                y = zeros(size(theta_deg));
                for ii = 1:numel(theta_deg)
                    y(ii) = obj.getConfiguredBendAngle(mod, theta_deg(ii));
                end
                stroke_mm = zeros(size(theta_deg));
                if mod.n == 1
                    y_label = 'Fold angle θ (deg)';
                    mode_label = 'fold angle (deg)';
                else
                    y_label = 'Total bending angle Θ (deg)';
                    mode_label = 'total bending angle (deg)';
                end
            end
        end

        function updateMappingPreview(obj, efP1, efP2, efA1, efA2)
            % Endpoint angles are derived from the calibrated model.
            % Text inputs avoid MATLAB's locale-dependent native numeric-validation tooltip.
            [p1, ok1] = obj.validateNumericTextField(efP1, 'Initial pressure', -101.3, 0, true, '%.3f');
            [p2, ok2] = obj.validateNumericTextField(efP2, 'Final pressure', -101.3, 0, true, '%.3f');
            if ~(ok1 && ok2), return; end
            p1 = round(p1 * 1000) / 1000;
            p2 = round(p2 * 1000) / 1000;
            efP1.Value = sprintf('%.3f', p1); efP1.UserData = p1;
            efP2.Value = sprintf('%.3f', p2); efP2.UserData = p2;
            [a1, ~, ~] = obj.pressureToB1B3L8(p1);
            [a2, ~, ~] = obj.pressureToB1B3L8(p2);
            efA1.Value = sprintf('%.3f', a1);
            efA1.UserData = a1;
            efA2.Value = sprintf('%.3f', a2);
            efA2.UserData = a2;
        end

        function params = getPressureModelParams(obj)
            % Final calibrated physical parameters supplied for the paper model.
            % They remain independent of the GUI's display-only dimensions.
            params = struct('a', 0.2313, 'b', 35.5672, 'c', 10.5289, ...
                'l', 30, 'w', 30, 'w_top', 16, 'h_trap', 15, ...
                'theta0_deg', 45, 'h_g', 0.8, 'h_p', 3.5);
        end

        function [theta_deg, b3_total_deg, l8_stroke_mm, params] = pressureToB1B3L8(obj, pressure_kpa)
            % Numerically invert the final nonlinear pressure-folding-angle model.
            % No polynomial fitting or piecewise-linear replacement is used.
            params = obj.getPressureModelParams();
            % Pressure is gauge pressure. At sea level, negative gauge pressure is bounded
            % by approximately -101.3 kPa (absolute pressure cannot be negative).
            % Invert only on a strictly positive fold-angle branch.
            P = max(-101.3, min(0, double(pressure_kpa)));
            theta_deg = zeros(size(P));
            theta0 = deg2rad(params.theta0_deg);
            theta_min = deg2rad(0.1); % strict positive-angle floor for numerical robustness
            for ii = 1:numel(P)
                lo = theta_min;
                hi = theta0;
                targetP = P(ii);
                for jj = 1:60
                    mid = 0.5 * (lo + hi);
                    p_mid = obj.pressureFromThetaRad(mid, params);
                    if p_mid < targetP
                        lo = mid;
                    else
                        hi = mid;
                    end
                end
                theta_deg(ii) = rad2deg(0.5 * (lo + hi));
            end
            theta = deg2rad(theta_deg);
            l8_stroke_mm = 8 .* ((params.l/2) .* (sin(theta0)-sin(theta)) + ...
                (params.h_g/2) .* (cos(theta)-cos(theta0)));
            b3_total_deg = 3 .* theta_deg;
        end

        function PkPa = pressureFromThetaRad(obj, theta, params)
            theta0 = deg2rad(params.theta0_deg);
            x = max(0, theta0 - theta);
            radicand = params.h_trap^2 - params.l^2 .* sin(theta/2).^2;
            radicand = max(radicand, eps);
            Jv = params.l .* (2*params.w + params.w_top) ./ 6 .* ...
                (params.l .* cos(theta) - cos(theta/2) .* ...
                (params.h_trap^2 - 2*params.l^2 .* sin(theta/2).^2) ./ sqrt(radicand));
            if abs(Jv) < eps
                PkPa = 0;
            else
                PkPa = -1e6 .* (params.a .* x + params.b .* x.^params.c) ./ Jv;
            end
        end

        function L = getModuleLengthAtAngle(obj, curr_mod, ang)
            % Actual axial length from the paper's analytical model, generalized to n layers.
            % L0 = 2*hp + n*l/2*sin(theta0) + (n-1)*hg; L = L0 - DeltaL.
            params = obj.getPressureModelParams();
            theta = deg2rad(double(ang));
            theta0 = deg2rad(params.theta0_deg);
            n = curr_mod.n;
            L0 = 2*params.h_p + n*(params.l/2)*sin(theta0) + (n-1)*params.h_g;
            stroke = n .* ((params.l/2).*(sin(theta0)-sin(theta)) + ...
                (params.h_g/2).*(cos(theta)-cos(theta0)));
            L = L0 - stroke;
        end

        function tf = isStraightContractionModule(obj, curr_mod)
            % Axial-contraction output is selected from the module configuration,
            % not merely from its number of layers.
            tf = false;
            if curr_mod.n < 2 || numel(curr_mod.seq) ~= curr_mod.n-1
                return;
            end
            q = mod(double(curr_mod.seq(:)'), 360);
            q(abs(q-360) < 1e-6) = 0;
            patternA = mod(180*(1:numel(q)), 360); % 180,0,180,0,...
            patternB = mod(180*(0:numel(q)-1), 360); % 0,180,0,180,...
            errA = min(abs(q-patternA), 360-abs(q-patternA));
            errB = min(abs(q-patternB), 360-abs(q-patternB));
            tf = all(errA < 1e-4) || all(errB < 1e-4);
        end

        function theta_total = getConfiguredBendAngle(obj, curr_mod, fold_angle_deg)
            % Derive the overall bend from the configured skeleton sequence.
            % Same-direction stacks retain cumulative angle (e.g., B3 = 3*theta).
            seq = double(curr_mod.seq(:)');
            if isempty(seq) || all(min(abs(mod(seq,360)), abs(mod(seq,360)-360)) < 1e-6)
                theta_total = curr_mod.n * fold_angle_deg;
                return;
            end
            Trot = eye(4);
            for ii = 1:curr_mod.n
                if ii > 1
                    Trot = Trot * obj.rotz(seq(ii-1));
                end
                Trot = Trot * obj.rotx(-fold_angle_deg);
            end
            R = Trot(1:3,1:3);
            cval = max(-1, min(1, (trace(R)-1)/2));
            theta_total = acos(cval) * 180/pi;
        end

        function configurePressureControls(obj, mod, curr_p, ang, pct)
            % Use a stable 0.001 kPa pressure resolution for the UI and model input.
            curr_p = round(double(curr_p) * 1000) / 1000;
            pmin = min(mod.p_start, mod.p_end);
            pmax = max(mod.p_start, mod.p_end);
            if abs(pmax-pmin) < 1e-9, pmax = pmin + 1; end

            obj.SldAngle.Enable = 'on';
            obj.UI_PressureEdit.Enable = 'on';
            obj.SldAngle.Limits = [pmin pmax];
            ticks = linspace(pmin, pmax, 5);
            if pmin < 0 && pmax >= 0
                ticks = unique([ticks 0]);
            end
            obj.SldAngle.MajorTicks = ticks;
            obj.SldAngle.MinorTicks = [];
            p_display = round(max(pmin, min(pmax, curr_p)) * 1000) / 1000;
            obj.SldAngle.Value = p_display;
            obj.UI_PressureEdit.Value = sprintf('%.3f', p_display);
            obj.UI_PressureEdit.UserData = p_display;
            [metric_value, ~, ~] = obj.getMappingOutput(mod, curr_p);
            if obj.isStraightContractionModule(mod)
                obj.LblSld.Text = sprintf('Length: %.1f mm | State: %d%%', metric_value, round(pct));
            else
                obj.LblSld.Text = sprintf('Angle: %.1f° | State: %d%%', metric_value, round(pct));
            end
            obj.updatePressureMapMarker(mod, curr_p, ang);
        end

        function onPressureEditChanged(obj, pressure_val)
            obj.applyPressureCommand(pressure_val);
        end

        function onPressureTextChanged(obj, field)
            % Validate pressure text with an English message, then apply the command.
            sel = obj.UIList.Value;
            if isempty(obj.RobotAssembly) || isempty(sel) || (ischar(sel) && strcmp(sel, '(Empty)'))
                return;
            end
            if ~iscell(sel), sel = {sel}; end
            idx = find(strcmp(obj.UIList.Items, sel{1}), 1);
            if isempty(idx), return; end
            mod = obj.RobotAssembly{idx};
            pmin = min(mod.p_start, mod.p_end);
            pmax = max(mod.p_start, mod.p_end);
            [pressure_val, ok] = obj.validateNumericTextField(field, 'Pressure', pmin, pmax, true, '%.3f');
            if ok
                obj.applyPressureCommand(pressure_val);
            end
        end

        function onNumericKinematicsFieldChanged(obj, field, label, min_value, max_value)
            [~, ok] = obj.validateNumericTextField(field, label, min_value, max_value, true, '%.12g');
            if ok
                obj.onKinematicsChanged();
            end
        end

        function [value, ok] = validateNumericTextField(obj, field, label, min_value, max_value, show_alert, display_format)
            % Shared English-only validation for editable numeric text fields.
            if nargin < 6 || isempty(show_alert), show_alert = true; end
            if nargin < 7 || isempty(display_format), display_format = '%.12g'; end
            raw_value = str2double(strtrim(char(field.Value)));
            ok = isfinite(raw_value) && raw_value >= min_value && raw_value <= max_value;
            if ~ok
                fallback = field.UserData;
                if isempty(fallback) || ~isnumeric(fallback) || ~isscalar(fallback) || ~isfinite(fallback)
                    fallback = 0;
                end
                value = double(fallback);
                field.Value = sprintf(display_format, value);
                field.UserData = value;
                if show_alert
                    if isfinite(min_value) && isfinite(max_value)
                        msg = sprintf('%s must be a number between %.3f and %.3f.', label, min_value, max_value);
                    elseif isfinite(min_value)
                        msg = sprintf('%s must be a number greater than or equal to %.3f.', label, min_value);
                    elseif isfinite(max_value)
                        msg = sprintf('%s must be a number less than or equal to %.3f.', label, max_value);
                    else
                        msg = sprintf('%s must be a valid finite number.', label);
                    end
                    uialert(obj.Fig, msg, 'Invalid Input');
                end
                return;
            end
            value = double(raw_value);
            field.UserData = value;
            field.Value = sprintf(display_format, value);
        end

        function value = readNumericFieldValue(obj, field)
            % Return the current numeric text value, falling back to the last valid value.
            value = str2double(strtrim(char(field.Value)));
            if ~isfinite(value)
                value = field.UserData;
            end
            if isempty(value) || ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
                value = 0;
            end
            value = double(value);
        end

        function addModuleFromInputs(obj, efName, efN, efSeq, efInstRot, efInstPitch, efA1, efA2, efP1, efP2)
            % Read and validate the module-definition inputs without native localized tooltips.
            previous_n = efN.UserData;
            [n, okN] = obj.validateNumericTextField(efN, 'Skeleton count', 1, Inf, true, '%.0f');
            if ~okN, return; end
            if abs(n-round(n)) > 1e-9
                efN.UserData = previous_n;
                efN.Value = sprintf('%.0f', previous_n);
                uialert(obj.Fig, 'Skeleton count must be a positive integer.', 'Invalid Input');
                return;
            end
            [inst_rot, okR] = obj.validateNumericTextField(efInstRot, 'Configuration yaw', -Inf, Inf, true, '%.12g');
            [inst_pitch, okP] = obj.validateNumericTextField(efInstPitch, 'Pitch', -Inf, Inf, true, '%.12g');
            [p1, ok1] = obj.validateNumericTextField(efP1, 'Initial pressure', -101.3, 0, true, '%.3f');
            [p2, ok2] = obj.validateNumericTextField(efP2, 'Final pressure', -101.3, 0, true, '%.3f');
            if ~(okR && okP && ok1 && ok2), return; end
            initial_angle = obj.readNumericFieldValue(efA1);
            final_angle = obj.readNumericFieldValue(efA2);
            obj.addModule(efName.Value, round(n), efSeq.Value, inst_rot, inst_pitch, ...
                initial_angle, final_angle, p1, p2);
        end

        function applyPressureCommand(obj, pressure_val)
            % Quantize pressure before mapping; do not carry slider floating-point noise.
            pressure_val = round(double(pressure_val) * 1000) / 1000;
            sel = obj.UIList.Value;
            if isempty(obj.RobotAssembly) || isempty(sel) || (ischar(sel) && strcmp(sel, '(Empty)')), return; end
            if ~iscell(sel), sel = {sel}; end

            first_mod = [];
            first_pct = 0; first_ang = 0; first_p = pressure_val;
            for k = 1:length(sel)
                idx = find(strcmp(obj.UIList.Items, sel{k}), 1);
                if isempty(idx), continue; end
                mod = obj.RobotAssembly{idx};
                pmin = min(mod.p_start, mod.p_end); pmax = max(mod.p_start, mod.p_end);
                p_cmd = round(max(pmin, min(pmax, pressure_val)) * 1000) / 1000;
                p_cmd = max(pmin, min(pmax, p_cmd));
                pct = obj.pressureToPct(mod, p_cmd);
                obj.RobotAssembly{idx}.contraction_pct = pct;
                if isempty(first_mod)
                    first_mod = obj.RobotAssembly{idx};
                    first_pct = pct;
                    first_p = p_cmd;
                    % Compute angle directly from the exact commanded pressure.
                    [first_ang, ~, ~] = obj.pressureToB1B3L8(first_p);
                end
            end
            if isempty(first_mod), return; end

            % Manual pressure actuation invalidates only trajectory/workspace data;
            % all robot/rendering functions remain unchanged.
            if ~isempty(obj.CurrentTrace) || ~isempty(obj.CurrentWSPoints)
                obj.CurrentTrace = [];
                obj.CurrentTraceStates = [];
                obj.CurrentTraceBreakIdx = [];
                obj.CurrentWSPoints = [];
                obj.showEmptyWorkspaceState();
                obj.UITextOut.Value = {'Pressure command applied manually.'; 'Trajectory/workspace data cleared.'};
            end

            obj.configurePressureControls(first_mod, first_p, first_ang, first_pct);
            [metric_value, ~, ~, theta_now, stroke_now, length_now] = obj.getMappingOutput(first_mod, first_p);
            if obj.isStraightContractionModule(first_mod)
                obj.UITextOut.Value = { ...
                    sprintf('Theoretical pressure-driven actuation: Module %s (straight contraction)', first_mod.name); ...
                    sprintf('Command: %.1f kPa -> single-skeleton fold angle: %.3f deg', first_p, theta_now); ...
                    sprintf('Actual axial length: %.3f mm | contraction: %.3f mm', length_now, stroke_now); ...
                    sprintf('Normalized state: %.1f%%', first_pct)};
            else
                if first_mod.n == 1
                    metric_text = sprintf('Fold angle: %.3f deg', metric_value);
                else
                    metric_text = sprintf('Configured total bending angle: %.3f deg', metric_value);
                end
                obj.UITextOut.Value = { ...
                    sprintf('Theoretical pressure-driven actuation: Module %s', first_mod.name); ...
                    sprintf('Command: %.1f kPa -> single-skeleton fold angle: %.3f deg', first_p, theta_now); ...
                    metric_text; ...
                    sprintf('Normalized state: %.1f%%', first_pct)};
            end
            obj.render3D();
            drawnow limitrate nocallbacks;
        end

        function addModule(obj, name, n, seqStr, inst_rot, inst_pitch, a1, a2, p1, p2)
            base_seq = str2num(seqStr); if isempty(base_seq), base_seq = 0; end %#ok<ST2NM>
            n = max(1, round(n));
            full_seq = zeros(1, max(0, n-1));
            for i = 1:(n-1), full_seq(i) = base_seq(mod(i-1, length(base_seq)) + 1); end

            % Use the final calibrated pressure-angle model as the sole source of angle data.
            % Use the theoretical negative-gauge-pressure range rather than the previous
            % hardware-specific -70 kPa endpoint.
            p1 = max(-101.3, min(0, p1));
            p2 = max(-101.3, min(0, p2));
            [a1, ~, ~] = obj.pressureToB1B3L8(p1);
            [a2, ~, ~] = obj.pressureToB1B3L8(p2);
            if abs(p2-p1) < 1e-9
                P_sorted = p1;
                A_sorted = a1;
            else
                nSamples = max(2, round(abs(p2-p1)) + 1);
                P_sorted = linspace(min(p1,p2), max(p1,p2), nSamples);
                [A_sorted, ~, ~] = obj.pressureToB1B3L8(P_sorted);
            end

            mod_struct = struct('name', name, 'n', n, 'seq', full_seq, ...
                'inst_rot', inst_rot, 'inst_pitch', inst_pitch, ...
                'P_array', P_sorted, 'A_array', A_sorted, ...
                'p_start', p1, 'p_end', p2, 'a_init', a1, 'a_min', a2, ...
                'contraction_pct', 0);
            obj.RobotAssembly{end+1} = mod_struct;
            obj.invalidateDerivedResults();
            obj.ViewEnvelopeDirty = true;
            obj.resetLiveGraphics();
            obj.updateList(); obj.UIList.Value = obj.UIList.Items{end}; obj.onListSelect(); obj.renderAndResize();

            % Keep the existing assembly-preview auto-fit behavior.
            obj.fitAssemblyPreviewView();
            drawnow limitrate;
        end
        
        function updateList(obj)
            items = {}; track_items = {'Global Tip (Default)'};
            for i = 1:length(obj.RobotAssembly)
                curr_mod = obj.RobotAssembly{i};
                [a_start, ~] = obj.getInterpolatedAngle(curr_mod, 0);
                [a_end, ~] = obj.getInterpolatedAngle(curr_mod, 100);
                items{end+1} = sprintf('[%d] %s: %d skeletons | P:[%g~%g] | Angle:[%.3f~%.3f]', ...
                    i, curr_mod.name, curr_mod.n, curr_mod.p_start, curr_mod.p_end, a_start, a_end);
                track_items{end+1} = sprintf('Module%d - Base Plate', i);
                track_items{end+1} = sprintf('Module%d - Top Plate', i);
            end
            if isempty(items), items = {'(Empty)'}; end; obj.UIList.Items = items;
            old_val = obj.UI_TrackDropdown.Value; obj.UI_TrackDropdown.Items = track_items;
            if any(strcmp(track_items, old_val)), obj.UI_TrackDropdown.Value = old_val; else, obj.UI_TrackDropdown.Value = track_items{1}; end
        end
        
        function deleteSelected(obj)
            sel = obj.UIList.Value;
            if isempty(obj.RobotAssembly) || isempty(sel), return; end
            if ischar(sel) && strcmp(sel, '(Empty)'), return; end
            if ~iscell(sel), sel = {sel}; end
            to_del = [];
            for k = 1:length(sel)
                idx = find(strcmp(obj.UIList.Items, sel{k}), 1);
                if ~isempty(idx), to_del(end+1) = idx; end
            end
            to_del = unique(to_del);
            if isempty(to_del), return; end
            obj.RobotAssembly(to_del) = [];
            obj.invalidateDerivedResults();
            obj.ViewEnvelopeDirty = true;
            obj.resetLiveGraphics();
            obj.updateList();
            if ~isempty(obj.RobotAssembly)
                obj.UIList.Value = obj.UIList.Items{min(to_del(1), length(obj.UIList.Items))};
                obj.onListSelect();
            else
                obj.SldAngle.Enable = 'off';
                obj.UI_PressureEdit.Enable = 'off';
                obj.LblSld.Text = 'Output: -- | State: --%';
                obj.showEmptyPressureMap();
            end
            obj.renderAndResize();
        end
        
        function onListSelect(obj)
            if isempty(obj.RobotAssembly), obj.showEmptyPressureMap(); return; end
            sel = obj.UIList.Value;
            if isempty(sel) || (ischar(sel) && strcmp(sel, '(Empty)')), return; end
            if ~iscell(sel), sel = {sel}; end
            idx = find(strcmp(obj.UIList.Items, sel{1}), 1);
            if isempty(idx), return; end

            curr_mod = obj.RobotAssembly{idx};
            pct = curr_mod.contraction_pct;
            [ang, curr_p] = obj.getInterpolatedAngle(curr_mod, pct);
            obj.showPressureMap(curr_mod, curr_p, ang);
            obj.configurePressureControls(curr_mod, curr_p, ang, pct);
        end

        function onSliderChanging(obj, pressure_val)
            % Slider is a direct pressure command; quantize to 0.001 kPa.
            obj.applyPressureCommand(round(double(pressure_val) * 1000) / 1000);
        end

        function resetRobot(obj)
            obj.RobotAssembly = {};
            obj.invalidateDerivedResults();
            obj.ViewEnvelope = [];
            obj.TopViewLimits = [];
            obj.ViewEnvelopeDirty = true;
            obj.resetLiveGraphics();
            obj.updateList();
            obj.SldAngle.Enable = 'off';
            obj.UI_PressureEdit.Enable = 'off';
            obj.UI_PressureEdit.Value = '0.000';
            obj.UI_PressureEdit.UserData = 0;
            obj.LblSld.Text = 'Output: -- | State: --%';
            obj.showEmptyPressureMap();
            obj.renderAndResize();
        end
        
        function popOutAxes(obj, targetAx, titleStr, isModel)
            if ~isModel && isempty(obj.CurrentWSPoints) && isempty(obj.CurrentTrace)
                uialert(obj.Fig, 'Calculate the workspace or run a dynamic trajectory first.', 'Notice'); return;
            end
            if isempty(targetAx.Children), return; end
            obj.closePlotFigure();
            f = figure('Name', titleStr, 'Color', 'w', 'Position', [200, 120, 900, 720], ...
                'Renderer', 'opengl', 'Tag', 'OrigamiRobotPlotFigure', 'NumberTitle', 'off');
            plot_panel = uipanel('Parent', f, 'Units', 'normalized', ...
                'Position', [0 0.12 1 0.88], 'BorderType', 'none', 'BackgroundColor', 'w');
            ax_new = axes(plot_panel, 'Position', [0.10 0.10 0.78 0.84]);
            copyobj(targetAx.Children, ax_new);
            ax_new.XLim = targetAx.XLim; ax_new.YLim = targetAx.YLim; ax_new.ZLim = targetAx.ZLim; ax_new.View = targetAx.View;
            ax_new.DataAspectRatio = targetAx.DataAspectRatio;
            ax_new.PlotBoxAspectRatio = targetAx.PlotBoxAspectRatio;
            ax_new.CameraPosition = targetAx.CameraPosition;
            ax_new.CameraTarget = targetAx.CameraTarget;
            ax_new.CameraUpVector = targetAx.CameraUpVector;
            ax_new.CameraViewAngle = targetAx.CameraViewAngle;
            xlabel(ax_new, targetAx.XLabel.String); ylabel(ax_new, targetAx.YLabel.String); zlabel(ax_new, targetAx.ZLabel.String);
            title(ax_new, targetAx.Title.String, 'FontName', 'Arial', 'FontSize', 13);
            set(ax_new, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.8, ...
                'Box', ~isModel, 'Projection', targetAx.Projection, 'Color', 'w');
            ax_new.XGrid = targetAx.XGrid; ax_new.YGrid = targetAx.YGrid; ax_new.ZGrid = targetAx.ZGrid;
            ax_new.GridColor = targetAx.GridColor; ax_new.GridAlpha = targetAx.GridAlpha;
            ax_new.ZAxis.Visible = targetAx.ZAxis.Visible;
            source_userdata = targetAx.UserData;
            if isstruct(source_userdata) && isfield(source_userdata, 'GradientColorbar') && ...
                    source_userdata.GradientColorbar
                colormap(ax_new, colormap(targetAx));
                caxis(ax_new, targetAx.CLim);
                cb = colorbar(ax_new, 'eastoutside');
                cb.Label.String = source_userdata.ColorbarLabel;
                cb.Label.FontName = 'Arial'; cb.Label.FontSize = 10;
                cb.FontName = 'Arial'; cb.FontSize = 9; cb.Box = 'off';
            end
            
            if isModel
                grid(ax_new, 'off'); axis(ax_new, 'off'); obj.addLightRig(ax_new);
            else
                grid(ax_new, 'on'); ax_new.GridColor = [0.82 0.85 0.88]; ax_new.GridAlpha = 0.28;
            end

            tool_panel = uipanel('Parent', f, 'Units', 'normalized', ...
                'Position', [0.012 0.012 0.976 0.095], 'BorderType', 'etchedin', ...
                'BackgroundColor', [0.965 0.972 0.976]);
            is_3d = abs(ax_new.View(2) - 90) > 1e-6;
            if is_3d
                tip_text = 'Drag to rotate, use the mouse wheel to zoom; export preserves the current view';
            else
                tip_text = 'Zoom or pan the plot, then select an export format';
            end
            status_text = uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.018 0.08 0.43 0.80], 'String', tip_text, ...
                'HorizontalAlignment', 'left', 'FontName', 'Arial', ...
                'FontSize', 9.5, 'ForegroundColor', [0.20 0.28 0.31], ...
                'BackgroundColor', [0.965 0.972 0.976]);
            obj.createPaperButton(tool_panel, [0.475 0.17 0.13 0.66], '600 dpi PNG', ...
                @(src,event)obj.exportStandaloneAxesFromButton(ax_new, 'png', status_text));
            obj.createPaperButton(tool_panel, [0.615 0.17 0.12 0.66], 'Vector PDF', ...
                @(src,event)obj.exportStandaloneAxesFromButton(ax_new, 'pdf', status_text));
            camera_state = obj.capturePaperCamera(ax_new);
            obj.createPaperButton(tool_panel, [0.745 0.17 0.11 0.66], 'Restore View', ...
                @(src,event)obj.restoreStandaloneView(ax_new, camera_state, status_text));
            obj.createPaperButton(tool_panel, [0.865 0.17 0.11 0.66], 'Close', ...
                @(src,event)delete(f));

            if is_3d, rotate3d(f, 'on'); else, enableDefaultInteractivity(ax_new); end
            figure(f);
        end

        function exportStandaloneAxesFromButton(obj, Ax, file_type, status_text)
            if ~ishghandle(Ax), return; end
            try
                if strcmpi(file_type, 'png')
                    [ok, out_file] = obj.saveAxesAsHighResPNG(Ax, ...
                        'Save Workspace / Trajectory Figure', 'Workspace_Trajectory.png');
                    format_label = '600 dpi PNG';
                else
                    [ok, out_file] = obj.saveAxesAsVectorPDF(Ax, ...
                        'Save Workspace / Trajectory Figure', 'Workspace_Trajectory.pdf');
                    format_label = 'Vector PDF';
                end
                if ok
                    obj.setPaperStatus(status_text, ...
                        sprintf('Saved as %s: %s', format_label, out_file), false);
                else
                    obj.setPaperStatus(status_text, 'Save canceled.', false);
                end
            catch ME
                obj.setPaperStatus(status_text, ['Export failed: ' ME.message], true);
            end
        end

        function restoreStandaloneView(obj, Ax, camera_state, status_text)
            obj.applyPaperCamera(Ax, camera_state);
            obj.setPaperStatus(status_text, 'Recommended view restored.', false);
        end

        function exportPaperFigure(obj)
            if isempty(obj.RobotAssembly)
                uialert(obj.Fig, 'Build the robot assembly first.', 'Notice'); return;
            end

            obj.closePaperFigure();
            f = figure('Name', 'Publication Figure: Robot and Global Workspace', ...
                'Color', 'w', 'Units', 'centimeters', 'Position', [2 2 18.0 9.6], ...
                'Renderer', 'opengl', 'Tag', 'OrigamiRobotPaperFigure', 'NumberTitle', 'off');
            % Panel A shows the structure; panel B shows the theoretical workspace. A 1:3 layout reduces unused space.
            tl = tiledlayout(f, 1, 8, 'TileSpacing', 'compact', 'Padding', 'compact');
            tl.Units = 'normalized';
            tl.OuterPosition = [0.002 0.165 0.996 0.825];
            style = obj.getPaperPalette();
            state_now = obj.getCurrentState();

            ax_model = nexttile(tl, 1, [1 2]); hold(ax_model, 'on');
            obj.drawRobotGeometry(ax_model, state_now, style, 1.0, 0.90, true);
            model_pts = obj.getArmPointsAtState(state_now);
            geometry_margin = 0.52 * obj.L;
            model_envelope = [min(model_pts, [], 1) - geometry_margin; ...
                max(model_pts, [], 1) + geometry_margin];
            obj.fitModelAxesToPoints(ax_model, model_envelope, 0.015);
            obj.applyPaperAxes(ax_model, false);
            axis(ax_model, 'off');
            text(ax_model, 0.02, 0.96, '(a)', 'Units', 'normalized', ...
                'FontName', 'Arial', 'FontSize', 10, 'FontWeight', 'bold');
            obj.addLightRig(ax_model);

            ax_space = nexttile(tl, 3, [1 6]); hold(ax_space, 'on');
            obj.drawPaperWorkspace(ax_space);
            text(ax_space, 0.02, 0.96, '(b)', 'Units', 'normalized', ...
                'FontName', 'Arial', 'FontSize', 10, 'FontWeight', 'bold');
            drawnow;

            % The Paper Fig. window has an integrated toolbar for view adjustment and export.
            tool_panel = uipanel('Parent', f, 'Units', 'normalized', ...
                'Position', [0.010 0.010 0.980 0.135], 'BorderType', 'etchedin', ...
                'BackgroundColor', [0.965 0.972 0.976]);
            if isempty(obj.CurrentWSPoints)
                paper_tip = sprintf('Panel B has no data\nPlease recalculate');
                tip_color = [0.72 0.30 0.12];
            else
                paper_tip = 'Drag to rotate | Wheel to zoom';
                tip_color = [0.20 0.28 0.31];
            end
            status_text = uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.012 0.08 0.125 0.82], ...
                'String', paper_tip, ...
                'HorizontalAlignment', 'left', 'FontName', 'Arial', ...
                'FontSize', 8.5, 'ForegroundColor', tip_color, ...
                'BackgroundColor', [0.965 0.972 0.976]);

            % Row 1: adjust font size and axes/tick widths directly in the Paper Fig. window.
            uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.145 0.59 0.070 0.25], 'String', 'Font size (pt)', ...
                'HorizontalAlignment', 'right', 'FontName', 'Arial', ...
                'FontSize', 8.5, 'BackgroundColor', [0.965 0.972 0.976]);
            font_edit = uicontrol(tool_panel, 'Style', 'edit', 'Units', 'normalized', ...
                'Position', [0.220 0.57 0.047 0.31], 'String', '9.5', ...
                'FontName', 'Arial', 'FontSize', 9, 'BackgroundColor', 'w');
            uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.275 0.59 0.082 0.25], 'String', 'Axes/ticks (pt)', ...
                'HorizontalAlignment', 'right', 'FontName', 'Arial', ...
                'FontSize', 8.5, 'BackgroundColor', [0.965 0.972 0.976]);
            line_edit = uicontrol(tool_panel, 'Style', 'edit', 'Units', 'normalized', ...
                'Position', [0.362 0.57 0.047 0.31], 'String', '0.90', ...
                'FontName', 'Arial', 'FontSize', 9, 'BackgroundColor', 'w');
            obj.createPaperButton(tool_panel, [0.420 0.54 0.088 0.36], 'Apply Style', ...
                @(src,event)obj.applyPaperFigureStyle(ax_model, ax_space, font_edit, line_edit, status_text));
            obj.createPaperButton(tool_panel, [0.518 0.54 0.092 0.36], 'Integrated Fig.', ...
                @(src,event)obj.openIntegratedPaperFigure());
            obj.createPaperButton(tool_panel, [0.620 0.54 0.100 0.36], 'Export Layout', ...
                @(src,event)obj.exportPaperLayoutFromButton(tl, status_text));

            % Row 2: save panels A and B separately using the integrated controls.
            obj.createPaperButton(tool_panel, [0.145 0.08 0.095 0.34], 'Panel A PNG', ...
                @(src,event)obj.exportPaperAxesFromButton(ax_model, 'png', status_text, '(a)'));
            obj.createPaperButton(tool_panel, [0.250 0.08 0.095 0.34], 'Panel A PDF', ...
                @(src,event)obj.exportPaperAxesFromButton(ax_model, 'pdf', status_text, '(a)'));
            obj.createPaperButton(tool_panel, [0.355 0.08 0.095 0.34], 'Panel B PNG', ...
                @(src,event)obj.exportPaperAxesFromButton(ax_space, 'png', status_text, '(b)'));
            obj.createPaperButton(tool_panel, [0.460 0.08 0.095 0.34], 'Panel B PDF', ...
                @(src,event)obj.exportPaperAxesFromButton(ax_space, 'pdf', status_text, '(b)'));

            obj.applyPaperFigureStyle(ax_model, ax_space, font_edit, line_edit, status_text);
            obj.setPaperStatus(status_text, paper_tip, ...
                isempty(obj.CurrentWSPoints));
            model_camera = obj.capturePaperCamera(ax_model);
            space_camera = obj.capturePaperCamera(ax_space);
            obj.createPaperButton(tool_panel, [0.742 0.54 0.100 0.36], 'Restore View', ...
                @(src,event)obj.restorePaperViews(ax_model, ax_space, model_camera, space_camera, status_text));
            obj.createPaperButton(tool_panel, [0.890 0.08 0.090 0.82], 'Close', ...
                @(src,event)delete(f));

            font_edit.Callback = @(src,event)obj.applyPaperFigureStyle( ...
                ax_model, ax_space, font_edit, line_edit, status_text);
            line_edit.Callback = @(src,event)obj.applyPaperFigureStyle( ...
                ax_model, ax_space, font_edit, line_edit, status_text);
            rotate3d(f, 'on');
            figure(f);
        end

        function createPaperButton(obj, parent_panel, position, label_text, callback)
            uicontrol(parent_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
                'Position', position, 'String', label_text, ...
                'FontName', 'Arial', 'FontSize', 9, ...
                'ForegroundColor', [0.16 0.23 0.26], ...
                'BackgroundColor', [0.91 0.94 0.95], 'Callback', callback);
        end

        function openIntegratedPaperFigure(obj)
            if isempty(obj.RobotAssembly)
                uialert(obj.Fig, 'Build the robot assembly first.', 'Notice'); return;
            end
            if isempty(obj.CurrentWSPoints) && isempty(obj.CurrentTrace)
                uialert(obj.Fig, ...
                    'Calculate the workspace or run a sequence before generating the integrated robot-workspace figure.', ...
                    'Missing Workspace / Trajectory Data');
                return;
            end

            obj.closeIntegratedFigure();
            f = figure('Name', 'Integrated Robot and Workspace Figure', ...
                'Color', 'w', 'Units', 'centimeters', 'Position', [4 2 16.2 12.2], ...
                'Renderer', 'opengl', 'Tag', 'OrigamiRobotIntegratedFigure', ...
                'NumberTitle', 'off');
            plot_panel = uipanel('Parent', f, 'Units', 'normalized', ...
                'Position', [0 0.14 1 0.86], 'BorderType', 'none', 'BackgroundColor', 'w');
            ax = axes(plot_panel, 'Position', [0.075 0.075 0.78 0.88]);
            obj.drawIntegratedRobotWorkspace(ax);
            drawnow;

            tool_panel = uipanel('Parent', f, 'Units', 'normalized', ...
                'Position', [0.012 0.012 0.976 0.105], 'BorderType', 'etchedin', ...
                'BackgroundColor', [0.965 0.972 0.976]);
            status_text = uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.012 0.08 0.17 0.80], ...
                'String', sprintf('Drag to rotate\nWheel to zoom'), ...
                'HorizontalAlignment', 'left', 'FontName', 'Arial', ...
                'FontSize', 9, 'ForegroundColor', [0.20 0.28 0.31], ...
                'BackgroundColor', [0.965 0.972 0.976]);
            obj.createPaperButton(tool_panel, [0.190 0.17 0.095 0.66], '600 dpi PNG', ...
                @(src,event)obj.exportIntegratedAxesFromButton(ax, 'png', status_text));
            obj.createPaperButton(tool_panel, [0.295 0.17 0.085 0.66], 'Vector PDF', ...
                @(src,event)obj.exportIntegratedAxesFromButton(ax, 'pdf', status_text));

            uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.395 0.16 0.070 0.58], 'String', 'Font (pt)', ...
                'HorizontalAlignment', 'right', 'FontName', 'Arial', ...
                'FontSize', 9, 'BackgroundColor', [0.965 0.972 0.976]);
            font_edit = uicontrol(tool_panel, 'Style', 'edit', 'Units', 'normalized', ...
                'Position', [0.470 0.20 0.050 0.60], 'String', '9', ...
                'FontName', 'Arial', 'FontSize', 9, 'BackgroundColor', 'w');
            uicontrol(tool_panel, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.530 0.16 0.085 0.58], 'String', 'Axes/ticks', ...
                'HorizontalAlignment', 'right', 'FontName', 'Arial', ...
                'FontSize', 9, 'BackgroundColor', [0.965 0.972 0.976]);
            line_edit = uicontrol(tool_panel, 'Style', 'edit', 'Units', 'normalized', ...
                'Position', [0.620 0.20 0.050 0.60], 'String', '0.9', ...
                'FontName', 'Arial', 'FontSize', 9, 'BackgroundColor', 'w');
            obj.createPaperButton(tool_panel, [0.685 0.17 0.085 0.66], 'Apply Style', ...
                @(src,event)obj.applyPublicationAxisStyle(ax, font_edit, line_edit, status_text));

            obj.applyPublicationAxisStyle(ax, font_edit, line_edit, status_text);
            obj.setPaperStatus(status_text, sprintf('Drag to rotate\nWheel to zoom'), false);
            camera_state = obj.capturePaperCamera(ax);
            obj.createPaperButton(tool_panel, [0.785 0.17 0.095 0.66], 'Restore View', ...
                @(src,event)obj.restoreStandaloneView(ax, camera_state, status_text));
            obj.createPaperButton(tool_panel, [0.895 0.17 0.085 0.66], 'Close', ...
                @(src,event)delete(f));

            font_edit.Callback = @(src,event)obj.applyPublicationAxisStyle(ax, font_edit, line_edit, status_text);
            line_edit.Callback = @(src,event)obj.applyPublicationAxisStyle(ax, font_edit, line_edit, status_text);
            rotate3d(f, 'on');
            figure(f);
        end

        function drawIntegratedRobotWorkspace(obj, Ax)
            cla(Ax, 'reset'); hold(Ax, 'on');
            style = obj.getPaperPalette();
            primary_state = obj.getCurrentState();
            if ~isempty(obj.CurrentTraceStates) && ...
                    size(obj.CurrentTraceStates,2) == length(obj.RobotAssembly)
                primary_state = obj.CurrentTraceStates(end,:);
            end
            all_pts = obj.getArmPointsAtState(primary_state);
            color_values = [];

            if ~isempty(obj.CurrentWSPoints)
                point_count = size(obj.CurrentWSPoints, 1);
                idx = unique(round(linspace(1, point_count, min(point_count, 9000))));
                ws = obj.CurrentWSPoints(idx, :);
                scatter3(Ax, ws(:,1), ws(:,2), ws(:,3), 7, ws(:,3), ...
                    'filled', 'MarkerFaceAlpha', 0.62, 'MarkerEdgeColor', 'none', ...
                    'HandleVisibility', 'off');
                all_pts = [all_pts; obj.CurrentWSPoints];
                color_values = [color_values; obj.CurrentWSPoints(:,3)];
            end

            if ~isempty(obj.CurrentTrace)
                obj.plotSegmentedHeightLine3D(Ax, obj.CurrentTrace, 2.4);
                scatter3(Ax, obj.CurrentTrace(1,1), obj.CurrentTrace(1,2), ...
                    obj.CurrentTrace(1,3), 40, 'k', 'filled', ...
                    'MarkerEdgeColor', 'w', 'LineWidth', 0.7, 'HandleVisibility', 'off');
                scatter3(Ax, obj.CurrentTrace(end,1), obj.CurrentTrace(end,2), ...
                    obj.CurrentTrace(end,3), 70, obj.CurrentTrace(end,3), ...
                    'filled', 'Marker', 'p', 'MarkerEdgeColor', style.edge, ...
                    'LineWidth', 0.8, 'HandleVisibility', 'off');
                all_pts = [all_pts; obj.CurrentTrace];
                color_values = [color_values; obj.CurrentTrace(:,3)];
            end

            tip_p = obj.drawRobotGeometry(Ax, primary_state, style, 1.0, 0.92, false);
            scatter3(Ax, tip_p(1), tip_p(2), tip_p(3), 30, style.trajectory, ...
                'filled', 'MarkerEdgeColor', style.edge, 'LineWidth', 0.6, ...
                'HandleVisibility', 'off');
            frame_pts = obj.drawEndFrame(Ax, tip_p, all_pts);
            all_pts = [all_pts; frame_pts];

            obj.fitAxesToPoints(Ax, all_pts, 0.04);
            obj.applyPaperAxes(Ax, true);
            Ax.GridColor = [0.78 0.81 0.83]; Ax.GridAlpha = 0.18;
            Ax.TickDir = 'in'; Ax.TickLength = [0.014 0.025];
            title(Ax, '');
            obj.addLightRig(Ax);
            obj.applyHeightColorScale(Ax, color_values);
        end

        function applyPublicationAxisStyle(obj, Ax, font_edit, line_edit, status_text)
            if ~ishghandle(Ax), return; end
            font_size = str2double(font_edit.String);
            line_width = str2double(line_edit.String);
            if ~isfinite(font_size), font_size = 9; end
            if ~isfinite(line_width), line_width = 0.9; end
            font_size = max(6, min(18, font_size));
            line_width = max(0.4, min(2.5, line_width));
            font_edit.String = sprintf('%.1f', font_size);
            line_edit.String = sprintf('%.2f', line_width);

            Ax.FontName = 'Arial'; Ax.FontSize = font_size;
            Ax.FontSizeMode = 'manual'; Ax.LineWidth = line_width;
            Ax.TickDir = 'in'; Ax.TickLength = [0.014 0.025];
            axis_text = findall(Ax, 'Type', 'text');
            for k = 1:numel(axis_text)
                axis_text(k).FontName = 'Arial';
                axis_text(k).FontSize = font_size;
            end
            Ax.XLabel.FontName = 'Arial'; Ax.XLabel.FontSize = font_size + 0.5;
            Ax.YLabel.FontName = 'Arial'; Ax.YLabel.FontSize = font_size + 0.5;
            Ax.ZLabel.FontName = 'Arial'; Ax.ZLabel.FontSize = font_size + 0.5;
            Ax.Title.FontName = 'Arial'; Ax.Title.FontSize = font_size + 1;
            fig = ancestor(Ax, 'figure');
            colorbars = findall(fig, 'Type', 'colorbar');
            for k = 1:numel(colorbars)
                colorbars(k).FontName = 'Arial';
                colorbars(k).FontSize = max(6, font_size - 0.5);
                colorbars(k).LineWidth = line_width;
                colorbars(k).Label.FontName = 'Arial';
                colorbars(k).Label.FontSize = font_size + 0.5;
            end
            obj.setPaperStatus(status_text, ...
                sprintf('Style applied: font %.1f pt, axes/ticks %.2f pt', ...
                font_size, line_width), false);
            drawnow;
        end

        function applyPaperFigureStyle(obj, ax_model, ax_space, font_edit, line_edit, status_text)
            if ~ishghandle(ax_model) || ~ishghandle(ax_space), return; end
            obj.applyPublicationAxisStyle(ax_model, font_edit, line_edit, status_text);
            obj.applyPublicationAxisStyle(ax_space, font_edit, line_edit, status_text);
            font_size = str2double(font_edit.String);
            line_width = str2double(line_edit.String);
            obj.setPaperStatus(status_text, ...
                sprintf('Panel A/B style updated: font %.1f pt, axes/ticks %.2f pt', ...
                font_size, line_width), false);
        end

        function exportIntegratedAxesFromButton(obj, Ax, file_type, status_text)
            if ~ishghandle(Ax), return; end
            try
                if strcmpi(file_type, 'png')
                    [ok, out_file] = obj.saveAxesAsHighResPNG(Ax, ...
                        'Save Integrated Robot-Workspace Figure', ...
                        'Integrated_Robot_Workspace.png');
                    format_label = '600 dpi PNG';
                else
                    [ok, out_file] = obj.saveAxesAsVectorPDF(Ax, ...
                        'Save Integrated Robot-Workspace Figure', ...
                        'Integrated_Robot_Workspace.pdf');
                    format_label = 'Vector PDF';
                end
                if ok
                    obj.setPaperStatus(status_text, ...
                        sprintf('Integrated figure saved as %s: %s', format_label, out_file), false);
                else
                    obj.setPaperStatus(status_text, 'Save canceled.', false);
                end
            catch ME
                obj.setPaperStatus(status_text, ['Export failed: ' ME.message], true);
            end
        end

        function exportPaperAxesFromButton(obj, Ax, file_type, status_text, panel_label)
            if ~ishghandle(Ax), return; end
            try
                if strcmpi(file_type, 'png')
                    if strcmp(panel_label, '(a)')
                        default_name = 'Figure_a_Robot_Model.png';
                    else
                        default_name = 'Figure_b_Global_Workspace.png';
                    end
                    [ok, out_file] = obj.saveAxesAsHighResPNG(Ax, ...
                        sprintf('Save panel %s as high-resolution PNG', panel_label), default_name);
                    format_label = '600 dpi PNG';
                else
                    if strcmp(panel_label, '(a)')
                        default_name = 'Figure_a_Robot_Model.pdf';
                    else
                        default_name = 'Figure_b_Global_Workspace.pdf';
                    end
                    [ok, out_file] = obj.saveAxesAsVectorPDF(Ax, ...
                        sprintf('Save panel %s as vector PDF', panel_label), default_name);
                    format_label = 'Vector PDF';
                end
                if ok
                    obj.setPaperStatus(status_text, ...
                        sprintf('%s saved as %s: %s', panel_label, format_label, out_file), false);
                else
                    obj.setPaperStatus(status_text, 'Save canceled.', false);
                end
            catch ME
                obj.setPaperStatus(status_text, ['Export failed: ' ME.message], true);
            end
        end

        function exportPaperLayoutFromButton(obj, layout_handle, status_text)
            try
                [file, path] = uiputfile({ ...
                    '*.pdf', 'Vector PDF (*.pdf)'; ...
                    '*.png', '600 dpi PNG (*.png)'; ...
                    '*.tif', '600 dpi TIFF (*.tif)'}, ...
                    'Export Publication Figure Layout', 'Robot_Workspace_Paper.pdf');
                if isequal(file, 0)
                    obj.setPaperStatus(status_text, 'Save canceled.', false); return;
                end
                out_file = fullfile(path, file);
                [~, ~, ext] = fileparts(out_file);
                if strcmpi(ext, '.pdf')
                    exportgraphics(layout_handle, out_file, ...
                        'ContentType', 'vector', 'BackgroundColor', 'white');
                else
                    exportgraphics(layout_handle, out_file, ...
                        'Resolution', 600, 'BackgroundColor', 'white');
                end
                obj.setPaperStatus(status_text, ['Figure layout saved: ' out_file], false);
            catch ME
                obj.setPaperStatus(status_text, ['Export failed: ' ME.message], true);
            end
        end

        function camera_state = capturePaperCamera(obj, Ax)
            camera_state = struct( ...
                'CameraPosition', Ax.CameraPosition, ...
                'CameraTarget', Ax.CameraTarget, ...
                'CameraUpVector', Ax.CameraUpVector, ...
                'CameraViewAngle', Ax.CameraViewAngle, ...
                'XLim', Ax.XLim, 'YLim', Ax.YLim, 'ZLim', Ax.ZLim);
        end

        function restorePaperViews(obj, ax_model, ax_space, model_camera, space_camera, status_text)
            obj.applyPaperCamera(ax_model, model_camera);
            obj.applyPaperCamera(ax_space, space_camera);
            obj.setPaperStatus(status_text, 'Recommended views for panels A/B restored.', false);
        end

        function applyPaperCamera(obj, Ax, camera_state)
            if ~ishghandle(Ax), return; end
            Ax.CameraPosition = camera_state.CameraPosition;
            Ax.CameraTarget = camera_state.CameraTarget;
            Ax.CameraUpVector = camera_state.CameraUpVector;
            Ax.CameraViewAngle = camera_state.CameraViewAngle;
            Ax.XLim = camera_state.XLim; Ax.YLim = camera_state.YLim; Ax.ZLim = camera_state.ZLim;
        end

        function setPaperStatus(obj, status_text, message_text, is_error)
            if ~ishghandle(status_text), return; end
            status_text.String = message_text;
            if is_error
                status_text.ForegroundColor = [0.72 0.18 0.14];
            else
                status_text.ForegroundColor = [0.16 0.38 0.41];
            end
        end

        function [ok, out_file] = saveAxesAsVectorPDF(obj, Ax, dialog_title, default_name)
            ok = false;
            out_file = '';
            [file, path] = uiputfile({'*.pdf', 'Vector PDF (*.pdf)'}, dialog_title, default_name);
            if isequal(file, 0), return; end
            out_file = fullfile(path, file);
            exportgraphics(Ax, out_file, 'ContentType', 'vector', 'BackgroundColor', 'white');
            ok = true;
        end

        function [ok, out_file] = saveAxesAsHighResPNG(obj, Ax, dialog_title, default_name)
            ok = false;
            out_file = '';
            [file, path] = uiputfile({'*.png', '600 dpi PNG (*.png)'}, dialog_title, default_name);
            if isequal(file, 0), return; end
            out_file = fullfile(path, file);
            exportgraphics(Ax, out_file, 'Resolution', 600, 'BackgroundColor', 'white');
            ok = true;
        end

        function drawPaperWorkspace(obj, Ax)
            % Panel B shows only the theoretical workspace, without the robot, dynamic trajectory, or tip frame.
            if isempty(obj.CurrentWSPoints)
                obj.applyPaperAxes(Ax, true);
                text(Ax, 0.5, 0.5, 'Please calculate the theoretical workspace first', ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                    'FontName', 'Arial', 'FontSize', 9, 'Color', [0.45 0.48 0.50]);
                return;
            end

            n = size(obj.CurrentWSPoints, 1);
            idx = unique(round(linspace(1, n, min(n, 6500))));
            ws = obj.CurrentWSPoints(idx, :);
            scatter3(Ax, ws(:,1), ws(:,2), ws(:,3), 10, ws(:,3), ...
                'filled', 'MarkerFaceAlpha', 0.88, 'MarkerEdgeColor', 'none', ...
                'HandleVisibility', 'off');

            obj.fitAxesToPoints(Ax, obj.CurrentWSPoints, 0.025);
            obj.applyPaperAxes(Ax, true);
            Ax.GridAlpha = 0.14;
            obj.applyHeightColorScale(Ax, obj.CurrentWSPoints(:,3));
        end

        function frame_pts = drawEndFrame(obj, Ax, origin, pts)
            if isempty(pts), scale_len = 22; else, scale_len = max(max(pts)-min(pts)) * 0.105; end
            scale_len = max(14, min(scale_len, 28));
            ox = origin(1); oy = origin(2); oz = origin(3);
            frame_pts = [origin'; origin' + [1.18*scale_len 0 0]; ...
                origin' + [0 1.18*scale_len 0]; origin' + [0 0 1.18*scale_len]];
            quiver3(Ax, ox,oy,oz, scale_len,0,0, 0, 'Color', [0.78 0.22 0.18], 'LineWidth', 1.15, 'MaxHeadSize', 0.38, 'HandleVisibility', 'off');
            quiver3(Ax, ox,oy,oz, 0,scale_len,0, 0, 'Color', [0.18 0.55 0.32], 'LineWidth', 1.15, 'MaxHeadSize', 0.38, 'HandleVisibility', 'off');
            quiver3(Ax, ox,oy,oz, 0,0,scale_len, 0, 'Color', [0.16 0.36 0.76], 'LineWidth', 1.15, 'MaxHeadSize', 0.38, 'HandleVisibility', 'off');
            text(Ax, ox+1.10*scale_len,oy,oz,'X_G', 'FontName','Arial','FontSize',8,'Color',[0.60 0.14 0.12]);
            text(Ax, ox,oy+1.10*scale_len,oz,'Y_G', 'FontName','Arial','FontSize',8,'Color',[0.12 0.43 0.23]);
            text(Ax, ox,oy,oz+1.10*scale_len,'Z_G', 'FontName','Arial','FontSize',8,'Color',[0.11 0.28 0.62]);
        end

        function applyPaperAxes(obj, Ax, show_axes)
            view(Ax, 38, 20); camup(Ax, [0 0 1]);
            daspect(Ax, [1 1 1]);
            set(Ax, 'FontName', 'Arial', 'FontSize', 8, 'LineWidth', 0.65, ...
                'Projection', 'orthographic', 'Color', 'w');
            if show_axes
                pbaspect(Ax, [1 1 1]);
                grid(Ax, 'on'); box(Ax, 'on');
                Ax.GridColor = [0.76 0.80 0.84]; Ax.GridAlpha = 0.20;
                xlabel(Ax, 'X (mm)'); ylabel(Ax, 'Y (mm)'); zlabel(Ax, 'Z (mm)');
            else
                Ax.PlotBoxAspectRatioMode = 'auto';
                grid(Ax, 'off'); box(Ax, 'off'); axis(Ax, 'off');
            end
        end

        function addLightRig(obj, Ax)
            delete(findall(Ax, 'Type', 'light'));
            light(Ax, 'Position', [1 -1 1.5], 'Style', 'infinite', 'Color', [1 1 1]);
            light(Ax, 'Position', [-1 0.5 0.4], 'Style', 'infinite', 'Color', [0.86 0.90 0.94]);
            lighting(Ax, 'gouraud'); material(Ax, 'dull');
        end

        function fitAxesToPoints(obj, Ax, pts, pad_frac)
            if isempty(pts), pts = [0 0 0; 1 1 1]; end
            minW = min(pts, [], 1); maxW = max(pts, [], 1);
            ctr = (minW + maxW) / 2;
            span = max(maxW - minW);
            radius = max(span * (0.5 + pad_frac), obj.L * 0.75);
            xlim(Ax, ctr(1) + [-radius radius]);
            ylim(Ax, ctr(2) + [-radius radius]);
            zlim(Ax, ctr(3) + [-radius radius]);
            camtarget(Ax, ctr);
        end

        function fitAssemblyPreviewView(obj)
            % Assembly-preview framing. Only the Ax3D camera and limits are adjusted here;
            % robot state, kinematic parameters, trajectory data, and animation logic remain unchanged.
            if isempty(obj.RobotAssembly) || isempty(obj.Ax3D) || ~isvalid(obj.Ax3D)
                return;
            end

            state_now = obj.getCurrentState();
            [mesh, tip_p] = obj.buildLiveMesh(state_now);

            V = [mesh.pad.V; mesh.outer.V; mesh.inner.V; mesh.hinge.V; mesh.side.V];
            V = V(all(isfinite(V), 2), :);
            if isempty(V)
                return;
            end

            % Include the tip frame in the bounds to prevent clipping during assembly.
            if isempty(obj.ViewEnvelope)
                axis_len = 18;
            else
                axis_len = max(12, min(26, 0.11 * obj.ViewEnvelope.radius));
            end
            extra_pts = [ ...
                tip_p(:)'; ...
                tip_p(:)' + [1.25*axis_len, 0, 0]; ...
                tip_p(:)' + [0, 1.25*axis_len, 0]; ...
                tip_p(:)' + [0, 0, 1.25*axis_len]];
            V = [V; extra_pts];

            minW = min(V, [], 1);
            maxW = max(V, [], 1);
            ctr = (minW + maxW) / 2;
            span_xyz = maxW - minW;
            max_span = max(span_xyz);

            % About 14%% safety margin keeps the assembly visible without making it too small.
            half_span = max(0.57 * max_span, 1.35 * obj.L);

            xlim(obj.Ax3D, ctr(1) + [-half_span, half_span]);
            ylim(obj.Ax3D, ctr(2) + [-half_span, half_span]);
            zlim(obj.Ax3D, ctr(3) + [-half_span, half_span]);
            obj.Ax3D.XLimMode = 'manual';
            obj.Ax3D.YLimMode = 'manual';
            obj.Ax3D.ZLimMode = 'manual';
            obj.Ax3D.DataAspectRatio = [1 1 1];
            obj.Ax3D.DataAspectRatioMode = 'manual';
            camtarget(obj.Ax3D, ctr);
        end

        function fitModelAxesToPoints(obj, Ax, pts, pad_frac)
            if isempty(pts), pts = [0 0 0; 1 1 1]; end
            minW = min(pts, [], 1); maxW = max(pts, [], 1);
            ctr = (minW + maxW) / 2;
            span = maxW - minW;
            span = max(span, [1.25*obj.L, 1.25*obj.L, 1.25*obj.L]);
            half_span = 0.5 * span .* (1 + 2*pad_frac);
            xlim(Ax, ctr(1) + [-half_span(1) half_span(1)]);
            ylim(Ax, ctr(2) + [-half_span(2) half_span(2)]);
            zlim(Ax, ctr(3) + [-half_span(3) half_span(3)]);
            camtarget(Ax, ctr);
        end
        
        function updateSmartAxesLimits(obj)
            % Keep the view fixed during recording; recompute the envelope only when geometry changes.
            % Never autoscale during motion, so the robot retains a constant on-screen size.
            if isempty(obj.RobotAssembly)
                obj.ViewEnvelope = struct('center', [0 0 20], 'radius', 45);
                obj.ViewEnvelopeDirty = false;
            elseif obj.ViewEnvelopeDirty || isempty(obj.ViewEnvelope)
                obj.computeFixedViewEnvelope();
            end

            ctr = obj.ViewEnvelope.center;
            r = obj.ViewEnvelope.radius;

            % Recording composition: shift the model slightly left and enlarge it.
            disp_ctr = ctr + [0.16*r, 0.08*r, 0.00*r];
            zoom_r = 0.80 * r;
            xlim(obj.Ax3D, disp_ctr(1) + [-0.92*zoom_r, 1.00*zoom_r]);
            ylim(obj.Ax3D, disp_ctr(2) + [-1.00*zoom_r, 0.92*zoom_r]);
            zlim(obj.Ax3D, disp_ctr(3) + [-0.98*zoom_r, 0.98*zoom_r]);
            obj.Ax3D.XLimMode = 'manual';
            obj.Ax3D.YLimMode = 'manual';
            obj.Ax3D.ZLimMode = 'manual';
            obj.Ax3D.DataAspectRatio = [1 1 1];
            obj.Ax3D.DataAspectRatioMode = 'manual';
            camtarget(obj.Ax3D, disp_ctr);
        end

        function computeFixedViewEnvelope(obj)
            M = length(obj.RobotAssembly);
            if M == 0
                obj.ViewEnvelope = struct('center', [0 0 20], 'radius', 45);
                obj.TopViewLimits = [-60 60 -60 60];
                obj.ViewEnvelopeDirty = false;
                return;
            end

            % Sample endpoint states once to estimate the motion envelope. Enumerate binary states for M<=10;
            % use a fixed random seed for larger assemblies to avoid combinatorial explosion.
            if M <= 10
                bits = dec2bin(0:(2^M-1)) - '0';
                states = 100 * double(bits);
            else
                old_rng = rng;
                rng(7, 'twister');
                states = 100 * double(rand(256, M) > 0.5);
                rng(old_rng);
            end

            states = [states; zeros(1,M); 50*ones(1,M); 100*ones(1,M)];
            pts_per_state = 1 + sum(cellfun(@(r) r.n + 2, obj.RobotAssembly));
            all_pts = zeros(size(states,1) * pts_per_state, 3);
            p0 = 1;
            for s = 1:size(states,1)
                pts = obj.getArmPointsAtState(states(s,:));
                p1 = p0 + size(pts,1) - 1;
                all_pts(p0:p1,:) = pts;
                p0 = p1 + 1;
            end
            all_pts = all_pts(1:p0-1,:);

            % Add structural half-width and recording margin around the centerline envelope.
            minW = min(all_pts, [], 1) - 0.85*obj.L;
            maxW = max(all_pts, [], 1) + 0.85*obj.L;
            ctr = (minW + maxW) / 2;
            span = max(maxW - minW);
            radius = max(0.44 * span, 1.50 * obj.L);

            obj.ViewEnvelope = struct('center', ctr, 'radius', radius);

            % Determine fixed top-view limits from the full motion envelope.
            % Use equal X/Y spans to prevent resizing or aspect-ratio changes during animation.
            xy_min = min(all_pts(:,1:2), [], 1) - 0.70*obj.L;
            xy_max = max(all_pts(:,1:2), [], 1) + 0.70*obj.L;
            xy_ctr = (xy_min + xy_max) / 2;
            xy_span = max(xy_max - xy_min);
            xy_half = max(0.57 * xy_span, 2.0 * obj.L);
            obj.TopViewLimits = [xy_ctr(1)-xy_half, xy_ctr(1)+xy_half, ...
                                 xy_ctr(2)-xy_half, xy_ctr(2)+xy_half];

            obj.ViewEnvelopeDirty = false;
        end

        function pts = getArmPointsAtPct(obj, pct)
            state = pct * ones(1, length(obj.RobotAssembly));
            pts = obj.getArmPointsAtState(state);
        end

        function state = getCurrentState(obj)
            if isempty(obj.RobotAssembly), state = []; return; end
            state = cellfun(@(r) r.contraction_pct, obj.RobotAssembly);
        end

        function pts = getArmPointsAtState(obj, state)
            % Preallocate arrays to avoid repeated growth during animation and envelope calculations.
            n_pts = 1 + sum(cellfun(@(r) r.n + 2, obj.RobotAssembly));
            pts = zeros(n_pts, 3);
            idx_pt = 1;
            pts(idx_pt,:) = [0 0 0];

            T_curr = eye(4);
            tpad = obj.readNumericFieldValue(obj.UI_TPad);
            tskel = obj.readNumericFieldValue(obj.UI_TSkel);
            for m = 1:length(obj.RobotAssembly)
                curr_mod = obj.RobotAssembly{m};
                T_curr = T_curr * obj.rotz(curr_mod.inst_rot) * ...
                    obj.rotx(curr_mod.inst_pitch) * obj.transz(tpad);
                idx_pt = idx_pt + 1;
                pts(idx_pt,:) = T_curr(1:3,4)';

                ang = obj.getInterpolatedAngle(curr_mod, state(m));
                for i = 1:curr_mod.n
                    if i > 1
                        T_curr = T_curr * obj.rotz(curr_mod.seq(i-1));
                    end
                    T_curr = T_curr * obj.getUnitTransform(ang, tskel);
                    idx_pt = idx_pt + 1;
                    pts(idx_pt,:) = T_curr(1:3,4)';
                end

                T_curr = T_curr * obj.transz(tpad);
                idx_pt = idx_pt + 1;
                pts(idx_pt,:) = T_curr(1:3,4)';
            end
        end

        function resetLiveGraphics(obj)
            obj.LivePatch = struct();
            obj.LiveTip = [];
            obj.LiveTrail = [];
            obj.LiveFrame = struct();
            obj.TopViewGraphics = struct();
            obj.LiveGraphicsReady = false;
            if ~isempty(obj.AxTopView) && isvalid(obj.AxTopView)
                obj.initTopViewAxes();
            end
        end

        function render3D(obj)
            % High-performance rendering: avoid clearing and rebuilding dozens of patches;
            % reuse five patch handles and update only Vertices/Faces each frame.
            if isempty(obj.RobotAssembly)
                cla(obj.Ax3D, 'reset');
                obj.init3DAxes(obj.Ax3D);
                obj.updateSmartAxesLimits();
                obj.applyPaperAxes(obj.Ax3D, false);
                obj.resetLiveGraphics();
                return;
            end

            style = obj.getPaperPalette();
            state_now = obj.getCurrentState();
            [mesh, tip_p] = obj.buildLiveMesh(state_now);

            if ~obj.LiveGraphicsReady || ~obj.areLiveGraphicsValid()
                cla(obj.Ax3D, 'reset');
                obj.init3DAxes(obj.Ax3D);
                hold(obj.Ax3D, 'on');

                obj.LivePatch.pad = obj.createLivePatch(mesh.pad, style.pad, style.edge);
                obj.LivePatch.outer = obj.createLivePatch(mesh.outer, style.outer, style.edge);
                obj.LivePatch.inner = obj.createLivePatch(mesh.inner, style.inner, style.edge);
                obj.LivePatch.hinge = obj.createLivePatch(mesh.hinge, style.hinge, style.edge);
                obj.LivePatch.side = obj.createLivePatch(mesh.side, style.side, style.edge);

                obj.LiveTip = plot3(obj.Ax3D, tip_p(1), tip_p(2), tip_p(3), 'o', ...
                    'Color', style.trajectory, 'MarkerFaceColor', style.trajectory, ...
                    'MarkerSize', 5, 'HandleVisibility', 'off');

                obj.LiveTrail = plot3(obj.Ax3D, NaN, NaN, NaN, '-', ...
                    'Color', style.trajectory, 'LineWidth', 1.7, 'HandleVisibility', 'off');

                obj.updateSmartAxesLimits();
                obj.createLiveEndFrame(tip_p);
                obj.applyPaperAxes(obj.Ax3D, false);
                obj.addLightRig(obj.Ax3D);
                enableDefaultInteractivity(obj.Ax3D);
                obj.LiveGraphicsReady = true;
            else
                obj.updateLivePatch(obj.LivePatch.pad, mesh.pad, style.pad, style.edge);
                obj.updateLivePatch(obj.LivePatch.outer, mesh.outer, style.outer, style.edge);
                obj.updateLivePatch(obj.LivePatch.inner, mesh.inner, style.inner, style.edge);
                obj.updateLivePatch(obj.LivePatch.hinge, mesh.hinge, style.hinge, style.edge);
                obj.updateLivePatch(obj.LivePatch.side, mesh.side, style.side, style.edge);
                set(obj.LiveTip, 'XData', tip_p(1), 'YData', tip_p(2), 'ZData', tip_p(3), ...
                    'Color', style.trajectory, 'MarkerFaceColor', style.trajectory);
                obj.updateSmartAxesLimits();
                obj.updateLiveEndFrame(tip_p);
            end

            obj.updateLiveTipTrail();
            obj.updateTopViewInset(state_now);
        end

        function updateLiveTipTrail(obj)
            if isempty(obj.LiveTrail) || ~all(isgraphics(obj.LiveTrail))
                return;
            end
            if isempty(obj.CurrentTrace)
                set(obj.LiveTrail, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                return;
            end
            [xv, yv, zv] = obj.buildSegmentedTraceVectors(obj.CurrentTrace);
            set(obj.LiveTrail, 'XData', xv, 'YData', yv, 'ZData', zv);
        end

        function updateTopViewInset(obj, state_now)
            if isempty(obj.AxTopView) || ~isvalid(obj.AxTopView)
                return;
            end

            pts = obj.getArmPointsAtState(state_now);
            x = pts(:,1); y = pts(:,2);
            style = obj.getPaperPalette();

            % Include the tip trajectory in the top view and use the centerline to match the main model's motion.
            [tx, ty, ~] = obj.buildSegmentedTraceVectors(obj.CurrentTrace);

            arm_ok = isfield(obj.TopViewGraphics, 'arm') && ...
                ~isempty(obj.TopViewGraphics.arm) && all(isgraphics(obj.TopViewGraphics.arm));
            if ~arm_ok
                cla(obj.AxTopView);
                hold(obj.AxTopView, 'on');
                obj.TopViewGraphics.trail = plot(obj.AxTopView, tx, ty, '-', 'Color', style.trajectory, ...
                    'LineWidth', 1.4, 'HandleVisibility', 'off');
                obj.TopViewGraphics.arm = plot(obj.AxTopView, x, y, '-', 'Color', style.side, 'LineWidth', 2.4);
                obj.TopViewGraphics.nodes = plot(obj.AxTopView, x, y, 'o', 'Color', style.edge, ...
                    'MarkerSize', 3.2, 'MarkerFaceColor', style.outer);
                obj.TopViewGraphics.tip = plot(obj.AxTopView, x(end), y(end), 'p', 'Color', style.edge, ...
                    'MarkerSize', 9, 'MarkerFaceColor', style.trajectory);
                obj.TopViewGraphics.base = plot(obj.AxTopView, x(1), y(1), 'o', 'Color', 'k', ...
                    'MarkerSize', 5.5, 'MarkerFaceColor', 'k');
                hold(obj.AxTopView, 'off');
            else
                trail_ok = isfield(obj.TopViewGraphics, 'trail') && ...
                    ~isempty(obj.TopViewGraphics.trail) && all(isgraphics(obj.TopViewGraphics.trail));
                if trail_ok
                    set(obj.TopViewGraphics.trail, 'XData', tx, 'YData', ty, 'Color', style.trajectory);
                end
                set(obj.TopViewGraphics.arm, 'XData', x, 'YData', y, 'Color', style.side);
                set(obj.TopViewGraphics.nodes, 'XData', x, 'YData', y, 'Color', style.edge, 'MarkerFaceColor', style.outer);
                set(obj.TopViewGraphics.tip, 'XData', x(end), 'YData', y(end), 'Color', style.edge, 'MarkerFaceColor', style.trajectory);
                set(obj.TopViewGraphics.base, 'XData', x(1), 'YData', y(1));
            end

            % Keep the top-view window fixed; recompute its bounds only when structure or parameters change;
            % do not change its bounds with the current pose or accumulated trajectory.
            if obj.ViewEnvelopeDirty || isempty(obj.TopViewLimits)
                obj.computeFixedViewEnvelope();
            end
            lim = obj.TopViewLimits;
            xlim(obj.AxTopView, lim(1:2));
            ylim(obj.AxTopView, lim(3:4));
            obj.AxTopView.XLimMode = 'manual';
            obj.AxTopView.YLimMode = 'manual';
            obj.AxTopView.DataAspectRatio = [1 1 1];
            obj.AxTopView.DataAspectRatioMode = 'manual';
            view(obj.AxTopView, 2);
            grid(obj.AxTopView, 'on');
            obj.AxTopView.GridColor = [0.80 0.83 0.86];
            obj.AxTopView.GridAlpha = 0.22;
            obj.AxTopView.Box = 'on';
            obj.AxTopView.Color = [1 1 1];
            xlabel(obj.AxTopView, 'X (mm)', 'FontWeight', 'bold');
            ylabel(obj.AxTopView, 'Y (mm)', 'FontWeight', 'bold');
            title(obj.AxTopView, 'Top View (X-Y)', 'FontName', 'Arial', 'FontSize', 10.5);
        end

        function tf = areLiveGraphicsValid(obj)
            tf = isfield(obj.LivePatch, 'pad') && ~isempty(obj.LivePatch.pad) && all(isgraphics(obj.LivePatch.pad)) && ...
                isfield(obj.LivePatch, 'outer') && ~isempty(obj.LivePatch.outer) && all(isgraphics(obj.LivePatch.outer)) && ...
                isfield(obj.LivePatch, 'inner') && ~isempty(obj.LivePatch.inner) && all(isgraphics(obj.LivePatch.inner)) && ...
                isfield(obj.LivePatch, 'hinge') && ~isempty(obj.LivePatch.hinge) && all(isgraphics(obj.LivePatch.hinge)) && ...
                isfield(obj.LivePatch, 'side') && ~isempty(obj.LivePatch.side) && all(isgraphics(obj.LivePatch.side)) && ...
                ~isempty(obj.LiveTip) && all(isgraphics(obj.LiveTip)) && ...
                ~isempty(obj.LiveTrail) && all(isgraphics(obj.LiveTrail));
        end

        function h = createLivePatch(obj, group, face_col, edge_col)
            h = patch(obj.Ax3D, 'Vertices', group.V, 'Faces', group.F, ...
                'FaceColor', face_col, 'FaceAlpha', 1.0, ...
                'EdgeColor', edge_col, 'EdgeAlpha', 0.78, 'LineWidth', 0.55, ...
                'FaceLighting', 'gouraud', 'AmbientStrength', 0.68, ...
                'DiffuseStrength', 0.48, 'SpecularStrength', 0.06, ...
                'HandleVisibility', 'off');
        end

        function updateLivePatch(obj, h, group, face_col, edge_col)
            set(h, 'Vertices', group.V, 'Faces', group.F, ...
                'FaceColor', face_col, 'EdgeColor', edge_col);
        end

        function [mesh, tip_p] = buildLiveMesh(obj, state)
            mesh.pad = obj.emptyMeshGroup();
            mesh.outer = obj.emptyMeshGroup();
            mesh.inner = obj.emptyMeshGroup();
            mesh.hinge = obj.emptyMeshGroup();
            mesh.side = obj.emptyMeshGroup();

            T_curr = eye(4);
            tpad = obj.readNumericFieldValue(obj.UI_TPad);
            tskel = obj.readNumericFieldValue(obj.UI_TSkel);

            p1=[-15; -15; 0; 1]; p2=[15; -15; 0; 1];
            p3=[15; 15; 0; 1];   p4=[-15; 15; 0; 1];

            for m = 1:length(obj.RobotAssembly)
                curr_mod = obj.RobotAssembly{m};
                T_curr = T_curr * obj.rotz(curr_mod.inst_rot) * obj.rotx(curr_mod.inst_pitch);
                mesh.pad = obj.appendPadMesh(mesh.pad, T_curr, tpad);
                T_curr = T_curr * obj.transz(tpad);

                ang = obj.getInterpolatedAngle(curr_mod, state(m));
                for i = 1:curr_mod.n
                    if i > 1
                        T_curr = T_curr * obj.rotz(curr_mod.seq(i-1));
                    end

                    T_top = T_curr * obj.getUnitTransform(ang, tskel);
                    T_mid = T_curr * obj.getMidTransform(ang, tskel);

                    e_b_L = T_curr*p1; e_b_R = T_curr*p2;
                    h_b_R = T_curr*p3; h_b_L = T_curr*p4;
                    e_t_L = T_top*p1;  e_t_R = T_top*p2;
                    h_t_R = T_top*p3;  h_t_L = T_top*p4;

                    cy = -15 + obj.V_depth;
                    c_L = T_mid*[-7.5; cy; 0; 1];
                    c_R = T_mid*[ 7.5; cy; 0; 1];
                    m_h_R = T_mid*p3;
                    m_h_L = T_mid*p4;

                    mesh.outer = obj.appendQuadMesh(mesh.outer, e_b_L, e_b_R, h_b_R, h_b_L);
                    mesh.outer = obj.appendQuadMesh(mesh.outer, e_t_L, e_t_R, h_t_R, h_t_L);

                    mesh.inner = obj.appendQuadMesh(mesh.inner, e_b_L, e_b_R, c_R, c_L);
                    mesh.inner = obj.appendQuadMesh(mesh.inner, c_L, c_R, e_t_R, e_t_L);

                    mesh.hinge = obj.appendQuadMesh(mesh.hinge, h_b_L, h_b_R, m_h_R, m_h_L);
                    mesh.hinge = obj.appendQuadMesh(mesh.hinge, m_h_L, m_h_R, h_t_R, h_t_L);

                    mesh.side = obj.appendQuadMesh(mesh.side, e_b_R, h_b_R, m_h_R, c_R);
                    mesh.side = obj.appendQuadMesh(mesh.side, c_R, m_h_R, h_t_R, e_t_R);
                    mesh.side = obj.appendQuadMesh(mesh.side, h_b_L, e_b_L, c_L, m_h_L);
                    mesh.side = obj.appendQuadMesh(mesh.side, m_h_L, c_L, e_t_L, h_t_L);

                    T_curr = T_top;
                end

                mesh.pad = obj.appendPadMesh(mesh.pad, T_curr, tpad);
                T_curr = T_curr * obj.transz(tpad);
            end

            tip_p = T_curr(1:3,4);
        end

        function group = emptyMeshGroup(obj)
            group = struct('V', zeros(0,3), 'F', zeros(0,4));
        end

        function group = appendPadMesh(obj, group, T, h)
            x = obj.L/2; y = obj.L/2;
            v = [-x -y 0; x -y 0; x y 0; -x y 0; ...
                 -x -y h; x -y h; x y h; -x y h];
            pts = (T * [v, ones(8,1)]')';
            base = size(group.V,1);
            group.V(end+1:end+8,:) = pts(:,1:3);
            localF = [1 2 3 4; 5 6 7 8; 1 2 6 5; ...
                      2 3 7 6; 3 4 8 7; 4 1 5 8];
            group.F(end+1:end+6,:) = base + localF;
        end

        function group = appendQuadMesh(obj, group, p1, p2, p3, p4)
            P = [p1'; p2'; p3'; p4'];
            base = size(group.V,1);
            group.V(end+1:end+4,:) = P(:,1:3);
            group.F(end+1,:) = base + (1:4);
        end

        function createLiveEndFrame(obj, origin)
            if isempty(obj.ViewEnvelope)
                axis_len = 18;
            else
                axis_len = max(12, min(26, 0.11 * obj.ViewEnvelope.radius));
            end

            ox=origin(1); oy=origin(2); oz=origin(3);
            obj.LiveFrame.qx = quiver3(obj.Ax3D, ox,oy,oz, axis_len,0,0, 0, ...
                'Color', [0.78 0.22 0.18], 'LineWidth', 1.0, 'MaxHeadSize', 0.38, 'HandleVisibility', 'off');
            obj.LiveFrame.qy = quiver3(obj.Ax3D, ox,oy,oz, 0,axis_len,0, 0, ...
                'Color', [0.18 0.55 0.32], 'LineWidth', 1.0, 'MaxHeadSize', 0.38, 'HandleVisibility', 'off');
            obj.LiveFrame.qz = quiver3(obj.Ax3D, ox,oy,oz, 0,0,axis_len, 0, ...
                'Color', [0.16 0.36 0.76], 'LineWidth', 1.0, 'MaxHeadSize', 0.38, 'HandleVisibility', 'off');
            obj.LiveFrame.tx = text(obj.Ax3D, ox+1.08*axis_len,oy,oz,'X_G', ...
                'FontName','Arial','FontSize',8,'Color',[0.60 0.14 0.12], 'HandleVisibility','off');
            obj.LiveFrame.ty = text(obj.Ax3D, ox,oy+1.08*axis_len,oz,'Y_G', ...
                'FontName','Arial','FontSize',8,'Color',[0.12 0.43 0.23], 'HandleVisibility','off');
            obj.LiveFrame.tz = text(obj.Ax3D, ox,oy,oz+1.08*axis_len,'Z_G', ...
                'FontName','Arial','FontSize',8,'Color',[0.11 0.28 0.62], 'HandleVisibility','off');
        end

        function updateLiveEndFrame(obj, origin)
            qx_ok = isfield(obj.LiveFrame,'qx') && ~isempty(obj.LiveFrame.qx) && all(isgraphics(obj.LiveFrame.qx));
            if ~qx_ok
                obj.createLiveEndFrame(origin);
                return;
            end

            if isempty(obj.ViewEnvelope)
                axis_len = 18;
            else
                axis_len = max(12, min(26, 0.11 * obj.ViewEnvelope.radius));
            end

            ox=origin(1); oy=origin(2); oz=origin(3);
            set(obj.LiveFrame.qx, 'XData',ox,'YData',oy,'ZData',oz,'UData',axis_len,'VData',0,'WData',0);
            set(obj.LiveFrame.qy, 'XData',ox,'YData',oy,'ZData',oz,'UData',0,'VData',axis_len,'WData',0);
            set(obj.LiveFrame.qz, 'XData',ox,'YData',oy,'ZData',oz,'UData',0,'VData',0,'WData',axis_len);
            obj.LiveFrame.tx.Position = [ox+1.08*axis_len, oy, oz];
            obj.LiveFrame.ty.Position = [ox, oy+1.08*axis_len, oz];
            obj.LiveFrame.tz.Position = [ox, oy, oz+1.08*axis_len];
        end

        function tip_p = drawRobotGeometry(obj, Ax, state, style, face_alpha, edge_alpha, mark_tip)
            if nargin < 7, mark_tip = false; end
            T_curr = eye(4); tpad = obj.readNumericFieldValue(obj.UI_TPad); tskel = obj.readNumericFieldValue(obj.UI_TSkel);
            for m = 1:length(obj.RobotAssembly)
                curr_mod = obj.RobotAssembly{m};
                T_curr = T_curr * obj.rotz(curr_mod.inst_rot) * obj.rotx(curr_mod.inst_pitch);
                obj.drawSolidPad(Ax, T_curr, style.pad, tpad, style.edge, face_alpha, edge_alpha);
                T_curr = T_curr * obj.transz(tpad);

                ang = obj.getInterpolatedAngle(curr_mod, state(m));
                for i = 1:curr_mod.n
                    if i > 1, T_curr = T_curr * obj.rotz(curr_mod.seq(i-1)); end
                    T_top = T_curr * obj.getUnitTransform(ang, tskel);
                    T_mid = T_curr * obj.getMidTransform(ang, tskel);
                    obj.drawOrigamiUnit(Ax, T_curr, T_top, T_mid, style, face_alpha, edge_alpha);
                    T_curr = T_top;
                end

                obj.drawSolidPad(Ax, T_curr, style.pad, tpad, style.edge, face_alpha, edge_alpha);
                T_curr = T_curr * obj.transz(tpad);
            end
            tip_p = T_curr(1:3,4);
            if mark_tip
                plot3(Ax, tip_p(1), tip_p(2), tip_p(3), 'o', ...
                    'MarkerSize', 5, 'MarkerEdgeColor', style.trajectory, ...
                    'MarkerFaceColor', style.trajectory, 'HandleVisibility', 'off');
            end
        end

        function style = getPaperPalette(obj)
            theme = 'Teal';
            if ~isempty(obj.UI_PaletteDropdown), theme = obj.UI_PaletteDropdown.Value; end
            switch theme
                case 'Blue'
                    style.pad = [0.54 0.61 0.69];
                    style.outer = [0.96 0.97 0.98];
                    style.inner = [0.73 0.82 0.90];
                    style.hinge = [0.85 0.88 0.91];
                    style.side = [0.30 0.54 0.73];
                    style.edge = [0.17 0.21 0.26];
                    style.workspace = [0.38 0.63 0.80];
                    style.trajectory = [0.91 0.38 0.18];
                    style.start = [0.22 0.25 0.29];
                    style.pose = [0.45 0.53 0.62];
                case 'Gray'
                    style.pad = [0.58 0.60 0.61];
                    style.outer = [0.97 0.97 0.965];
                    style.inner = [0.84 0.85 0.85];
                    style.hinge = [0.90 0.90 0.89];
                    style.side = [0.68 0.70 0.70];
                    style.edge = [0.18 0.18 0.18];
                    style.workspace = [0.62 0.65 0.66];
                    style.trajectory = [0.22 0.22 0.22];
                    style.start = [0.10 0.10 0.10];
                    style.pose = [0.48 0.49 0.49];
                otherwise
                    style.pad = [0.58 0.64 0.68];
                    style.outer = [0.965 0.958 0.94];
                    style.inner = [0.72 0.84 0.84];
                    style.hinge = [0.86 0.88 0.86];
                    style.side = [0.24 0.62 0.63];
                    style.edge = [0.18 0.22 0.23];
                    style.workspace = [0.36 0.66 0.78];
                    style.trajectory = [0.90 0.36 0.18];
                    style.start = [0.22 0.25 0.26];
                    style.pose = [0.45 0.54 0.57];
            end
        end
        
        function drawSolidPad(obj, Ax, T, col, h, edge_col, face_alpha, edge_alpha)
            x=obj.L/2; y=obj.L/2; v=[-x -y 0; x -y 0; x y 0; -x y 0; -x -y h; x -y h; x y h; -x y h];
            pts = (T * [v, ones(8,1)]')'; 
            patch(Ax, 'Vertices', pts(:,1:3), 'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8], ...
                'FaceColor', col, 'FaceAlpha', face_alpha, 'EdgeColor', edge_col, ...
                'EdgeAlpha', edge_alpha, 'LineWidth', 0.60, 'FaceLighting', 'gouraud', ...
                'AmbientStrength', 0.62, 'DiffuseStrength', 0.52, 'SpecularStrength', 0.10, ...
                'HandleVisibility', 'off');
        end
        
        function drawOrigamiUnit(obj, Ax, T_base, T_top, T_mid, style, face_alpha, edge_alpha)
            p1=[-15; -15; 0; 1]; p2=[15; -15; 0; 1]; p3=[15; 15; 0; 1]; p4=[-15; 15; 0; 1];
            e_b_L = T_base*p1; e_b_R = T_base*p2; h_b_R = T_base*p3; h_b_L = T_base*p4;
            e_t_L = T_top*p1;  e_t_R = T_top*p2;  h_t_R = T_top*p3;  h_t_L = T_top*p4;
            
            cy = -15 + obj.V_depth; 
            c_L = T_mid*[-7.5; cy; 0; 1]; 
            c_R = T_mid*[ 7.5; cy; 0; 1];
            m_h_R = T_mid*p3; m_h_L = T_mid*p4; 
            
            obj.patchFace(Ax, e_b_L, e_b_R, h_b_R, h_b_L, style.outer, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, e_t_L, e_t_R, h_t_R, h_t_L, style.outer, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, e_b_L, e_b_R, c_R, c_L, style.inner, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, c_L, c_R, e_t_R, e_t_L, style.inner, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, h_b_L, h_b_R, m_h_R, m_h_L, style.hinge, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, m_h_L, m_h_R, h_t_R, h_t_L, style.hinge, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, e_b_R, h_b_R, m_h_R, c_R, style.side, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, c_R, m_h_R, h_t_R, e_t_R, style.side, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, h_b_L, e_b_L, c_L, m_h_L, style.side, style, face_alpha, edge_alpha);
            obj.patchFace(Ax, m_h_L, c_L, e_t_L, h_t_L, style.side, style, face_alpha, edge_alpha);
        end
        
        function patchFace(obj, Ax, p1, p2, p3, p4, col, style, face_alpha, edge_alpha)
            pts = [p1'; p2'; p3'; p4']; 
            patch(Ax, 'Vertices', pts(:,1:3), 'Faces', [1 2 3 4], 'FaceColor', col, ...
                'FaceAlpha', face_alpha, 'EdgeColor', style.edge, 'EdgeAlpha', edge_alpha, ...
                'LineWidth', 0.60, 'FaceLighting', 'gouraud', ...
                'AmbientStrength', 0.62, 'DiffuseStrength', 0.52, 'SpecularStrength', 0.10, ...
                'HandleVisibility', 'off');
        end
        
        function T = transz(obj, z), T=[eye(3) [0;0;z]; 0 0 0 1]; end
        function T = transx(obj, x), T=[eye(3) [x;0;0]; 0 0 0 1]; end
        function T = transy(obj, y), T=[eye(3) [0;y;0]; 0 0 0 1]; end
        function T = getUnitTransform(obj, deg, t_skel)
            % Retain the existing verified geometric transform. The theoretical
            % pressure law determines deg; L8 stroke is reported from its own model.
            T = obj.transy(15) * obj.rotx(-deg) * obj.transy(-15) * obj.transz(t_skel);
        end
        function T = getMidTransform(obj, deg, t_skel)
            T = obj.transy(15) * obj.rotx(-deg/2) * obj.transy(-15) * obj.transz(t_skel/2);
        end
        function R = rotx(obj, deg), a=deg2rad(deg); R=[1 0 0 0; 0 cos(a) -sin(a) 0; 0 sin(a) cos(a) 0; 0 0 0 1]; end
        function R = rotz(obj, deg), a=deg2rad(deg); R=[cos(a) -sin(a) 0 0; sin(a) cos(a) 0 0; 0 0 1 0; 0 0 0 1]; end
    end
end
