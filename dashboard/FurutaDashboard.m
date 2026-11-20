classdef FurutaDashboard < handle
    % Sections 4.5/4.6 dashboard. Input angles and angular rates are SI units.
    % app = FurutaDashboard; app.startDemo();
    % Live input: app.update(struct('t',t,'theta0',a,'theta1',p,'u',u,'mode',mode));
    properties (SetAccess = private)
        Figure
        Samples = table()
        Source = 'Demo'
    end
    properties (Access = private)
        Clock
        Timer
        Axes
        Lines
        Readouts
        Status
        ModeLabel
        SourceLabel
        StartButton
        TwinAxes
        ArmLine
        PendulumLine
        Tip
        LastTime = -inf
        DemoOffset = 0
        WindowSeconds = 15
    end
    methods
        function app = FurutaDashboard(varargin)
            p = inputParser;
            addParameter(p,'Visible','on');
            parse(p,varargin{:});
            app.Figure = uifigure('Name','Furuta | Real-time dashboard', ...
                'Position',[80 70 1250 790],'Visible',p.Results.Visible, ...
                'Color',[0.95 0.96 0.98],'CloseRequestFcn',@(~,~)delete(app));
            app.Figure.Theme = 'light';
            root = uigridlayout(app.Figure,[4 1]);
            root.RowHeight = {65,92,'1x',52};
            root.Padding = [20 16 20 16];
            header = uigridlayout(root,[1 3]);
            header.ColumnWidth = {'1x',210,200};
            uilabel(header,'Text','FURUTA  /  CONTROL MONITOR', ...
                'FontSize',23,'FontWeight','bold','FontColor',[0.08 0.15 0.25]);
            app.SourceLabel = uilabel(header,'Text','SOURCE: DEMO (synthetic)', ...
                'FontWeight','bold','FontColor',[0.35 0.40 0.47]);
            app.ModeLabel = uilabel(header,'Text','MODE: --','FontWeight','bold', ...
                'HorizontalAlignment','center','BackgroundColor',[0.85 0.89 0.95]);
            cards = uigridlayout(root,[1 4]);
            cards.Padding = [0 0 0 0];
            names = {'ARM ANGLE  theta0','PENDULUM ANGLE  theta1','MOTOR COMMAND  u','ANGULAR RATES  arm / pendulum'};
            app.Readouts = gobjects(1,4);
            for k = 1:4
                panel = uipanel(cards,'BackgroundColor','white','BorderType','none');
                cardGrid = uigridlayout(panel,[2 1]); cardGrid.RowHeight = {22,'1x'};
                uilabel(cardGrid,'Text',names{k},'FontSize',11,'FontColor',[0.4 0.45 0.52]);
                app.Readouts(k) = uilabel(cardGrid,'Text','--','FontSize',23, ...
                    'FontWeight','bold','FontColor',[0.08 0.15 0.25]);
            end
            app.Readouts(4).FontSize = 17;
            body = uigridlayout(root,[1 2]); body.ColumnWidth = {'2x','1x'};
            plots = uigridlayout(body,[3 1]);
            titles = {'Pendulum angle | upright = 0 deg','Arm angle','Motor command | normalized demo units'};
            colors = {[0.08 0.53 0.68],[0.43 0.32 0.75],[0.90 0.43 0.17]};
            app.Axes = gobjects(1,3); app.Lines = gobjects(1,3);
            for k = 1:3
                ax = uiaxes(plots); app.Axes(k) = ax;
                title(ax,titles{k}); grid(ax,'on'); xlabel(ax,'Time (s)');
                if k < 3, ylabel(ax,'Angle (deg)'); else, ylabel(ax,'u'); end
                app.Lines(k) = plot(ax,nan,nan,'Color',colors{k},'LineWidth',1.6);
                ax.XLim = [0 app.WindowSeconds];
            end
            side = uigridlayout(body,[3 1]); side.RowHeight = {35,'1x',80};
            uilabel(side,'Text','SYNCHRONIZED SCHEMATIC','FontWeight','bold','FontSize',15);
            ax = uiaxes(side); app.TwinAxes = ax;
            view(ax,35,22); grid(ax,'on'); hold(ax,'on');
            axis(ax,'equal'); xlim(ax,[-1.5 1.5]); ylim(ax,[-1.5 1.5]); zlim(ax,[-1.2 1.2]);
            xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
            plot3(ax,0,0,0,'o','MarkerSize',10,'MarkerFaceColor',[0.2 0.25 0.3]);
            app.ArmLine = plot3(ax,[0 1],[0 0],[0 0],'-','LineWidth',7,'Color',colors{2});
            app.PendulumLine = plot3(ax,[1 1],[0 0],[0 1],'-','LineWidth',4,'Color',colors{1});
            app.Tip = plot3(ax,1,0,1,'o','MarkerSize',9,'MarkerFaceColor',colors{1});
            uilabel(side,'Text',sprintf(['Demo signals only. This schematic follows the displayed angles.\n' ...
                'Live hardware and the team\x2019s digital twin can be connected later.\n' ...
                'Monitoring controls do not command or disable the motor.']), ...
                'WordWrap','on','FontColor',[0.4 0.45 0.52]);
            footer = uigridlayout(root,[1 5]);
            footer.ColumnWidth = {140,120,120,140,'1x'};
            app.StartButton = uibutton(footer,'Text','Start demo','ButtonPushedFcn',@(~,~)app.startDemo());
            uibutton(footer,'Text','Pause display','ButtonPushedFcn',@(~,~)app.stop());
            uibutton(footer,'Text','Reset demo','ButtonPushedFcn',@(~,~)app.resetDemo());
            uibutton(footer,'Text','Export CSV','ButtonPushedFcn',@(~,~)app.exportCSV());
            app.Status = uilabel(footer,'Text','Ready | press Start demo','HorizontalAlignment','right');
            app.Timer = timer('ExecutionMode','fixedSpacing','Period',0.05, ...
                'BusyMode','drop','TimerFcn',@(~,~)app.demoTick(), ...
                'ErrorFcn',@(~,~)app.timerFailed());
        end
        function startDemo(app)
            if strcmp(app.Timer.Running,'on'), return; end
            if ~strcmp(app.Source,'Demo')
                app.clearData();
                app.DemoOffset = 0;
            end
            app.Source = 'Demo';
            app.SourceLabel.Text = 'SOURCE: DEMO (synthetic)';
            app.Clock = tic;
            start(app.Timer);
            app.Status.Text = 'Demo running | nominal refresh 20 Hz';
        end
        function stop(app)
            if ~isempty(app.Timer) && isvalid(app.Timer), stop(app.Timer); end
            if strcmp(app.Source,'Demo') && isfinite(app.LastTime)
                app.DemoOffset = app.LastTime;
            end
            app.Status.Text = 'Display paused';
        end
        function resetDemo(app)
            app.stop(); app.clearData(); app.DemoOffset = 0;
            app.Source = 'Demo'; app.SourceLabel.Text = 'SOURCE: DEMO (synthetic)';
            app.ModeLabel.Text = 'MODE: --';
            for k=1:4, app.Readouts(k).Text = '--'; end
            app.drawTwin(0,0);
            app.Status.Text = 'Demo reset | press Start demo';
        end
        function connectLive(app)
            % Call once before delivering samples from the team's acquisition code.
            app.stop(); app.clearData(); app.Source = 'Live';
            app.SourceLabel.Text = 'SOURCE: LIVE INPUT';
            title(app.Axes(3),'Motor command | team-defined units');
            app.Status.Text = 'Live input selected | waiting for sample';
        end
        function update(app,s)
            % Required: t(s), theta0(rad), theta1(rad), u, mode.
            % Optional: omega0(rad/s), omega1(rad/s). Missing rates show as --.
            required = {'t','theta0','theta1','u','mode'};
            assert(isstruct(s) && isscalar(s) && all(isfield(s,required)), ...
                'FurutaDashboard:Input','Input must be one struct with t, theta0, theta1, u, mode.');
            for key = {'t','theta0','theta1','u'}
                validateattributes(s.(key{1}),{'numeric'},{'real','finite','scalar'});
            end
            assert(s.t>=0 && s.t>app.LastTime,'FurutaDashboard:Time', ...
                'Sample time must be nonnegative and strictly increasing. Reset before a new run.');
            mode = string(s.mode);
            assert(isscalar(mode) && ~ismissing(mode),'FurutaDashboard:Mode','Mode must be scalar text.');
            rates = [nan nan];
            for k=1:2
                key = ['omega' num2str(k-1)];
                if isfield(s,key)
                    validateattributes(s.(key),{'numeric'},{'real','finite','scalar'});
                    rates(k) = s.(key);
                end
            end
            row = table(double(s.t),double(s.theta0),double(s.theta1),double(s.u), ...
                rates(1),rates(2),mode,string(app.Source),'VariableNames', ...
                {'Time_s','Theta0_rad','Theta1_rad','MotorCommand','Omega0_rad_s','Omega1_rad_s','Mode','Source'});
            app.Samples = [app.Samples;row];
            app.LastTime = s.t;
            app.Readouts(1).Text = sprintf('%+.1f deg',rad2deg(s.theta0));
            app.Readouts(2).Text = sprintf('%+.1f deg',rad2deg(s.theta1));
            app.Readouts(3).Text = sprintf('%+.3f',s.u);
            if all(isfinite(rates))
                app.Readouts(4).Text = sprintf('%+.1f / %+.1f deg/s',rad2deg(rates));
            else
                app.Readouts(4).Text = '-- / -- deg/s';
            end
            app.ModeLabel.Text = 'MODE: ' + mode;
            if strcmpi(mode,'Stabilization')
                app.ModeLabel.BackgroundColor = [0.77 0.93 0.84];
            else
                app.ModeLabel.BackgroundColor = [1 0.88 0.66];
            end
            data = app.Samples;
            keep = data.Time_s >= max(0,s.t-app.WindowSeconds);
            values = {rad2deg(data.Theta1_rad(keep)),rad2deg(data.Theta0_rad(keep)),data.MotorCommand(keep)};
            for k=1:3
                app.Lines(k).XData = data.Time_s(keep); app.Lines(k).YData = values{k};
                app.Axes(k).XLim = [max(0,s.t-app.WindowSeconds),max(app.WindowSeconds,s.t)];
            end
            app.drawTwin(s.theta0,s.theta1);
            app.Status.Text = sprintf('%s | t = %.2f s | %d samples',app.Source,s.t,height(data));
            drawnow limitrate;
        end
        function exportCSV(app,path)
            if isempty(app.Samples)
                uialert(app.Figure,'Record some samples before exporting.','No data'); return;
            end
            if nargin<2
                [name,folder] = uiputfile('*.csv','Export dashboard data','furuta_dashboard.csv');
                if isequal(name,0), return; end
                path = fullfile(folder,name);
            end
            writetable(app.Samples,path);
            app.Status.Text = sprintf('Exported %d samples',height(app.Samples));
        end
        function delete(app)
            if ~isempty(app.Timer) && isvalid(app.Timer)
                stop(app.Timer); delete(app.Timer);
            end
            if ~isempty(app.Figure) && isgraphics(app.Figure)
                app.Figure.CloseRequestFcn = []; delete(app.Figure);
            end
        end
    end
    methods (Access = private)
        function clearData(app)
            app.Samples = table(); app.LastTime = -inf;
            for k=1:3
                app.Lines(k).XData = nan; app.Lines(k).YData = nan;
                app.Axes(k).XLim = [0 app.WindowSeconds];
            end
            title(app.Axes(3),'Motor command | normalized demo units');
        end
        function demoTick(app)
            t = app.DemoOffset + toc(app.Clock);
            % Scripted visualization, not a plant simulation or controller design.
            phase = mod(t,24);
            if phase < 7
                p = pi*cos(pi*phase/7); pd = -pi*pi/7*sin(pi*phase/7);
                a = 0.65*sin(2.2*phase); ad = 1.43*cos(2.2*phase);
                u = 0.8*sin(2.2*phase); mode = 'Swing-Up';
            elseif phase < 9
                q = (phase-7)/2; h = 3*q*q-2*q*q*q;
                p = -pi*(1-h); pd = pi*(6*q-6*q*q)/2;
                a = 0.65*sin(2.2*phase); ad = 1.43*cos(2.2*phase);
                u = 0.6*sin(2.2*phase); mode = 'Swing-Up';
            else
                z = phase-9;
                % Smooth capture followed by a repeatable synthetic disturbance.
                a = 0.65*sin(19.8)*exp(-z)*cos(2*z) + 1.43*cos(19.8)*z*exp(-z);
                ad = 0.65*sin(19.8)*exp(-z)*(-cos(2*z)-2*sin(2*z)) + 1.43*cos(19.8)*(1-z)*exp(-z);
                a = a + 0.65*sin(19.8)*z*exp(-z);
                ad = ad + 0.65*sin(19.8)*(1-z)*exp(-z);
                p = 0; pd = 0;
                if z > 6
                    r = z-6;
                    p = deg2rad(8)*r^2*exp(-2*r)*sin(5*r);
                    pd = deg2rad(8)*exp(-2*r)*((2*r-2*r^2)*sin(5*r)+5*r^2*cos(5*r));
                end
                u = max(-1,min(1,-2*p-0.4*pd-0.15*a)); mode = 'Stabilization';
            end
            app.update(struct('t',t,'theta0',a,'theta1',p,'u',u, ...
                'omega0',ad,'omega1',pd,'mode',mode));
        end
        function drawTwin(app,a,p)
            % Unit-length schematic. Pendulum rotates in the arm's tangential plane.
            base = [cos(a),sin(a),0];
            tip = base + [-sin(a)*sin(p),cos(a)*sin(p),cos(p)];
            app.ArmLine.XData = [0 base(1)]; app.ArmLine.YData = [0 base(2)]; app.ArmLine.ZData = [0 0];
            app.PendulumLine.XData = [base(1) tip(1)];
            app.PendulumLine.YData = [base(2) tip(2)]; app.PendulumLine.ZData = [0 tip(3)];
            app.Tip.XData = tip(1); app.Tip.YData = tip(2); app.Tip.ZData = tip(3);
        end
        function timerFailed(app)
            app.stop(); app.Status.Text = 'Demo stopped after an error; see MATLAB Command Window';
        end
    end
end
