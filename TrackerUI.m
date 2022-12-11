function [dev_mode, filter_type, traj, run_mode, lidar,forced] = TrackerUI

fig = uifigure;
fig.Name = "Eppur si muove 2D";
fig.WindowState = 'maximized';
% fig.Color = 'k';

gl = uigridlayout(fig,[6 6]);
gl.RowHeight = {30,'1x'};
gl.ColumnWidth = {'fit','1x'};

lbl = uilabel(gl);
dd = uidropdown(gl);

lbl2 = uilabel(gl);
dd2 = uidropdown(gl);

lbl3 = uilabel(gl);
dd3 = uidropdown(gl);

ax = uiaxes(gl);

btn = uibutton(fig,'push',...
    'Position',[550, 50, 100, 30],...
    'ButtonPushedFcn', @(btn,event) playButtonPushed(btn,ax,fig));

dev_btn = uibutton(fig,'push',...
    'Position',[650, 50, 100, 30],...
    'ButtonPushedFcn', @(dev_btn,event) devButtonPushed(dev_btn,ax,fig));

lbl.Layout.Row = 1;
lbl.Layout.Column = 1;

dd.Layout.Row = 1;
dd.Layout.Column = 2;

ax.Layout.Row = 2;
ax.Layout.Column = [1 6];

lbl2.Layout.Row = 1;
lbl2.Layout.Column = 3;

dd2.Layout.Row = 1;
dd2.Layout.Column = 4;

lbl3.Layout.Row = 1;
lbl3.Layout.Column = 5;

dd3.Layout.Row = 1;
dd3.Layout.Column = 6;

lbl.Text  = "Choose Scenario:";
lbl2.Text = "Choose Running Mode:";
lbl3.Text = "Choose Difficulty Level:";

btn.Text = 'Play';
dev_btn.Text = 'Developer Mode';

dd.Items = ["Select","Linear Trajectory", "Circular Trajectory", "Random Movement"];
dd.Value = "Select";

dd2.Items = ["Select","Simulation","Command Driven"];
dd2.Value = "Select";

dd3.Items = ["Select","Linear Kalman Filter", "Linear Kalman Filter Forced Input","Extended Kalman Filter", "Extended Kalman Filter + Sonar", "Unscented Kalman Filter", "Unscented Kalman Filter + Sonar" ];
dd3.Value = "Select";

set(ax,'color',[0 0.4470 0.7410]);
set(fig,'color',[0 0.4470 0.7410]);
set(ax,'xtick',[],'ytick',[])

dd.ValueChangedFcn = { @changeTraj,ax,fig};
dd2.ValueChangedFcn = { @changeMode,ax,fig};
dd3.ValueChangedFcn = { @changeFilter,ax,fig};

[Seal,X,~] = innateSealDrawer();
for idx = 1:length(X)
    Seal(idx).XData = X{idx} + 12.5;
end
[Shark,Xs,~] = innateSharkDrawer();
for idx = 1:length(Xs)
    Shark(idx).XData = Xs{idx} - 5;
end

uiwait(fig);

    function changeTraj(src,event,ax,fig)
        type = event.Value;
        switch type
            case "Linear Trajectory"  
                traj = 'linear';
            case "Circular Trajectory"
                traj = 'circle'; 
            case "Random Movement"
                traj = 'random';
        end
    end

    function changeMode(src,event,ax,fig)
        type = event.Value;
        switch type
            case "Simulation"
                run_mode = 'simu';
            case "Command Driven"
                run_mode = 'cmdrive';
        end
    end

    function changeFilter(src,event,ax,fig)
        type = event.Value;
        
        switch type
            case "Linear Kalman Filter"
                
                delete(findobj(ax,'type', 'patch'))
                
                [Seal,X,~] = innateSealDrawer();
                for idx = 1:length(X)
                    Seal(idx).XData = X{idx} + 12.5;
                end
                [Shark,Xs,~] = innateSharkDrawer();
                for idx = 1:length(Xs)
                    Shark(idx).XData = Xs{idx} - 5;
                end
                
                filter_type = 'L';
                lidar = false;
                forced = false;
                
            case "Linear Kalman Filter Forced Input"
                
                delete(findobj(ax,'type', 'patch'))
                
                [Seal,X,~] = innateSealDrawer();
                for idx = 1:length(X)
                    Seal(idx).XData = X{idx} + 12.5;
                end
                [Shark,Xs,~] = innateSharkDrawer();
                for idx = 1:length(Xs)
                    Shark(idx).XData = Xs{idx} - 5;
                end
                
                filter_type = 'L';
                lidar = false;
                forced = true;
                
            case "Extended Kalman Filter"
                
                delete(findobj(ax,'type', 'patch'))
                
                [Seal,X,~] = innateSealDrawer();
                for idx = 1:length(X)
                    Seal(idx).XData = X{idx} + 12.5;
                end
                [Shark,Xs,~] = innateSharkDrawer();
                for idx = 1:length(Xs)
                    Shark(idx).XData = Xs{idx} - 5;
                end
                
                filter_type = 'E';
                lidar = false;
                forced = false;
                
            case "Extended Kalman Filter + Sonar"
                
                filter_type = 'E';
                lidar = true;
                forced = false;
                
                delete(findobj(ax,'type', 'patch'))
                
                [Seal,X,~] = innateSealDrawer();
                for idx = 1:length(X)
                    Seal(idx).XData = X{idx} + 12.5;
                end
                [Shark,Xs,~] = innateSharkDrawer();
                for idx = 1:length(Xs)
                    Shark(idx).XData = Xs{idx} - 5;
                end
                
                [Seagull,Xl,Yl] = innateSeagullDrawer();
                for sgl = 1:length(Yl)
                    Seagull(sgl).YData = Yl{sgl}/2 + 1;
                    Seagull(sgl).XData = Xl{sgl}/2 + 4;
                end
                
            case "Unscented Kalman Filter"
                
                delete(findobj(ax,'type', 'patch'))
                
                [Seal,X,~] = innateSealDrawer();
                for idx = 1:length(X)
                    Seal(idx).XData = X{idx} + 12.5;
                end
                [Shark,Xs,~] = innateSharkDrawer();
                for idx = 1:length(Xs)
                    Shark(idx).XData = Xs{idx} - 5;
                end
                
                filter_type = 'U';
                lidar = false;
                forced = false;
                
            case "Unscented Kalman Filter + Sonar"
                
                filter_type = 'U';
                lidar = true;
                forced = false;
                
                delete(findobj(ax,'type', 'patch'))
                
                [Seal,X,~] = innateSealDrawer();
                for idx = 1:length(X)
                    Seal(idx).XData = X{idx} + 12.5;
                end
                [Shark,Xs,~] = innateSharkDrawer();
                for idx = 1:length(Xs)
                    Shark(idx).XData = Xs{idx} - 5;
                end
                
                [Seagull,Xl,Yl] = innateSeagullDrawer();
                for sgl = 1:length(Yl)
                    Seagull(sgl).YData = Yl{sgl}/2 + 1;
                    Seagull(sgl).XData = Xl{sgl}/2 + 4;
                end
        end
    end

    function playButtonPushed(btn,ax,fig)
        dev_mode = false;
        close(fig)
    end

    function devButtonPushed(btn,ax,fig)
        filter_type = 'L';
        lidar = false;
        traj = 'linear';
        run_mode = 'simu';
        dev_mode = true;
        forced = false;
        close(fig)
    end

    function [Seal,x,y] = innateSealDrawer()
        
        hold(ax,'all')
        color = [0.710000 0.400000 0.110000];
        t=0:0.01:2*pi;
        x1=4*cos(t);
        y1=0.9*sin(t);
        Seal = fill(x1,y1,color,'Parent',ax);
        x{1} = x1;
        y{1} = y1;
        
        x2=3.4+0.4*cos(t);
        y2=1.2-1+0.2*sin(t);
        Seal = [Seal, fill(x2,y2,'w','Parent',ax)];
        x{2} = x2;
        y{2} = y2;
        
        x3=2.6+0.4*cos(t);
        y3=1.2-1+0.2*sin(t);
        Seal = [Seal, fill(x3,y3,'w','Parent',ax)];
        x{3} = x3;
        y{3} = y3;
        
        x4=3.35 - 0.1 +0.2*cos(t);
        y4=1.1-1 + 0.1 +0.1*sin(t);
        Seal = [Seal, fill(x4,y4,'k','Parent',ax)];
        x{4} = x4;
        y{4} = y4;
        
        x5=2.55 - 0.1 +0.2*cos(t);
        y5=1.1-1 + 0.1 +0.1*sin(t);
        Seal = [Seal, fill(x5,y5,'k','Parent',ax)];
        x{5} = x5;
        y{5} = y5;
        
        x6=3.25+0.5*cos(t);
        y6= -0.05 + 0.1*sin(t);
        Seal = [Seal, fill(x6,y6,'k','Parent',ax)];
        x{6} = x6;
        y{6} = y6;
        
        x7 = [ -0.55 0.2 0.43]/3 + 2.7;
        y7 = [ -0.6 -0.35 -0.2]/4 - 0.0095;
        Seal = [Seal, fill(x7,y7,'k','Parent',ax)];
        x{7} = x7;
        y{7} = y7;
        
        x8 = 1.1 * [ -0.75 0.4 0.63]/3 + 2.7;
        y8 = [ -0.6 -0.35 -0.2]/3 + 0.071;
        Seal = [Seal, fill(x8,y8,'k','Parent',ax)];
        x{8} = x8;
        y{8} = y8;
        
        x9 = [3.9683 3.7183 3.6417];
        y9 = [-0.1790 -0.0957 -0.0457];
        Seal = [Seal, fill(x9,y9,'k','Parent',ax)];
        x{9} = x9;
        y{9} = y9;
        
        x10 = [4.0200 3.6367 3.5600];
        y10 = [-0.1240 -0.0407 0.0093];
        Seal = [Seal, fill(x10,y10,'k','Parent',ax)];
        x{10} = x10;
        y{10} = y10;
        
        x11 = x10 * 4/3 - 1.15;
        y11 = y10 + 0.014;
        Seal = [Seal, fill(x11,y11,'k','Parent',ax)];
        x{11} = x11;
        y{11} = y11;
        
        x12 = [2.2333 2.8722 3.0000];
        y12 = [-0.0990 -0.0157 0.0343];
        Seal = [Seal, fill(x12,y12,'k','Parent',ax)];
        x{12} = x12;
        y{12} = y12;
    end

    function [Shark,x,y] = innateSharkDrawer()
        
        hold(ax,'all')
        color = [0.370000 0.620000 0.630000];
        t=0:0.01:2*pi;
        x1=4*cos(t);
        y1=0.5*sin(t);
        x1 = 2 * x1;
        y1 = 2 * y1;
        Shark = fill(x1,y1,color,'Parent',ax);
        x{1} = x1;
        y{1} = y1;
        
        x2=2.6+0.2*cos(t);
        y2=1.2-1+0.2*sin(t);
        x2 = 2 * x2;
        y2 = 2 * y2;
        Shark = [Shark, fill(x2,y2,'w','Parent',ax)];
        x{2} = x2;
        y{2} = y2;
        
        x3=3.4+0.2*cos(t);
        y3=1.2-1+0.2*sin(t);
        x3 = 2 * x3;
        y3 = 2 * y3;
        Shark = [Shark, fill(x3,y3,'w','Parent',ax)];
        x{3} = x3;
        y{3} = y3;
        
        x4=2.65+0.1*cos(t);
        y4=1.2-1+0.1*sin(t);
        x4 = 2 * x4;
        y4 = 2 * y4;
        Shark = [Shark, fill(x4,y4,'k','Parent',ax)];
        x{4} = x4;
        y{4} = y4;
        
        x5=3.45+0.1*cos(t);
        y5=1.2-1+0.1*sin(t);
        x5 = 2 * x5;
        y5 = 2 * y5;
        Shark = [Shark, fill(x5,y5,'k','Parent',ax)];
        x{5} = x5;
        y{5} = y5;
        
        x6 = [ 0 0.2 2];
        y6 = [ 0.5 0.7 0.375];
        x6 = 2 * x6;
        y6 = 2 * y6;
        Shark = [Shark, fill(x6,y6,color,'Parent',ax)];
        x{6} = x6;
        y{6} = y6;
        
        x7=2.55 + cos(t);
        y7= - 0.2 + 0.1*sin(t);
        x7 = 2 * x7;
        y7 = 2 * y7;
        Shark = [Shark, fill(x7,y7,[0.960000 0.760000 0.760000],'Parent',ax)];
        x{7} = x7;
        y{7} = y7;
        
        x8 = [ 1.73 2.15 2.6];
        y8 = [ -0.14 -0.29 -0.10];
        x8 = 2 * x8;
        y8 = 2 * y8;
        Shark = [Shark, fill(x8,y8,'w','Parent',ax)];
        x{8} = x8;
        y{8} = y8;
        
        x9 = [ 2.6 2.81 3.32];
        y9 = [ -0.10 -0.29 -0.13];
        x9 = 2 * x9;
        y9 = 2 * y9;
        Shark = [Shark, fill(x9,y9,'w','Parent',ax)];
        x{9} = x9;
        y{9} = y9;
        
        x10 = [ -0.55 0.2 0.41] + 5;
        y10 = [ -0.6 -0.35 -0.6];
        Shark = [Shark, fill(x10,y10,'w','Parent',ax)];
        x{10} = x10;
        y{10} = y10;
        
        x11 = [ 0.89 1.75 1.67] + 5;
        y11 = [ -0.58 -0.3 -0.52];
        Shark = [Shark, fill(x11,y11,'w','Parent',ax)];
        x{11} = x11;
        y{11} = y11;
        
        x12 = [ -0.55 0.2 0.43];
        y12 = [ -0.6 -0.35 -0.2] + 0.3;
        Shark = [Shark, fill(x12,y12,[0.960000 0.760000 0.760000],'Parent',ax)];
        x{12} = x12;
        y{12} = y12;
        
        x13 = [ -0.55 0.2 0.43] + 0.5;
        y13 = [ -0.6 -0.35 -0.2] + 0.3;
        Shark = [Shark, fill(x13,y13,[0.960000 0.760000 0.760000],'Parent',ax)];
        x{13} = x13;
        y{13} = y13;
        
        x14 = [ -0.55 0.2 0.43] - 0.5;
        y14 = [ -0.6 -0.35 -0.2] + 0.3;
        Shark = [Shark, fill(x14,y14,[0.960000 0.760000 0.760000],'Parent',ax)];
        x{14} = x14;
        y{14} = y14;
    end

    function [Seagull,x,y] = innateSeagullDrawer()
        
        hold(ax,'all')
        
        color = [0.950000 0.950000 0.960000];
        wings = [0.550000 0.570000 0.670000];
        beak = [0.930000 0.570000 0.130000];
        
        t=0:0.01:2*pi;
        
        x1=3+2 * cos(t);
        y1=0.1+0.6*sin(t);
        Seagull = [fill(x1,y1,color,'Parent',ax)];
        x{1} = x1;
        y{1} = y1;
        
        x2=3.4+0.4*cos(t);
        y2=1.2-1+0.2*sin(t);
        Seagull = [Seagull, fill(x2,y2,'w','Parent',ax)];
        x{2} = x2;
        y{2} = y2;
        
        x3=2.6+0.4*cos(t);
        y3=1.2-1+0.2*sin(t);
        Seagull = [Seagull, fill(x3,y3,'w','Parent',ax)];
        x{3} = x3;
        y{3} = y3;
        
        x4=3.45+0.2*cos(t);
        y4=1.1-1+0.1*sin(t);
        Seagull = [Seagull, fill(x4,y4,'k','Parent',ax)];
        x{4} = x4;
        y{4} = y4;
        
        x5=2.65+0.2*cos(t);
        y5=1.1-1+0.1*sin(t);
        Seagull = [Seagull, fill(x5,y5,'k','Parent',ax)];
        x{5} = x5;
        y{5} = y5;
        
        x6 = flip([4.3 3 3.0000]);
        y6 = [-0.0990+0.12 -0.0990-0.05 -0.0343];
        Seagull = [Seagull, fill(x6,y6,beak,'Parent',ax)];
        x{6} = x6;
        y{6} = y6;
        
        x7 = [1.04 1.55 0.05];
        y7 = [ -0.034 0.44 0.4];
        Seagull = [Seagull, fill(x7,y7,color,'Parent',ax)];
        x{7} = x7;
        y{7} = y7;
        
        x8 = [0.05 1.04 -2];
        y8 = [0.4 -0.034 0];
        Seagull = [Seagull, fill(x8,y8,wings,'Parent',ax)];
        x{8} = x8;
        y{8} = y8;
        
        x9 = [0.05 -2 -4];
        y9 = [0.4 0 0.5];
        Seagull = [Seagull, fill(x9,y9,wings,'Parent',ax)];
        x{9} = x9;
        y{9} = y9;
        
        x10 = [4.5 4.93 6.05];
        y10 = [0.43 -0.034 0.4];
        Seagull = [Seagull, fill(x10,y10,color,'Parent',ax)];
        x{10} = x10;
        y{10} = y10;
        
        x11 = [4.93 6.05 8];
        y11 = [-0.034 0.4 0];
        Seagull = [Seagull, fill(x11,y11,wings,'Parent',ax)];
        x{11} = x11;
        y{11} = y11;
        
        x12 = [6.05 8 10];
        y12 = [0.4 0 0.5];
        Seagull = [Seagull, fill(x12,y12,wings,'Parent',ax)];
        x{12} = x12;
        y{12} = y12;
        
        x13 = [8 10 13];
        y13 = [0 0.5 0.35];
        Seagull = [Seagull, fill(x13,y13,wings,'Parent',ax)];
        x{13} = x13;
        y{13} = y13;
        
        x14 = [-2 -4 -7];
        y14 = [0 0.5 0.35];
        Seagull = [Seagull, fill(x14,y14,wings,'Parent',ax)];
        x{14} = x14;
        y{14} = y14;
    end
end
