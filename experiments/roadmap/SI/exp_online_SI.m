% Author: Jared Strader
clear all; close all; clc;

%% Load Offline
disp('If exp_online_SI.m is edited to contain the correct filename, an error is returned. See READM.md for details on reproducing the long range experiments.');
load('...');
G = bsp.G_;

%% Start State, Goal State, Initial Belief
x0 = [-9;-9];
P0 = 1*eye(2);
x_goal = [9;9];

%% Search DORM
disp('Running BRMS using d-opt bound...');
bsp.G_=G;
[eig_dorm] = bsp.run_online(x0,...
                            P0,...
                            x_goal,...
                            0.1,...    %epsilon
                            -1,...     %guessed eigenvalue
                            1);        %method
G_dorm = bsp.G_;
data_dorm = fn_get_data_SI(bsp, x_goal, G_dorm, P0, eig_dorm);

                             
disp('Running BRMS using e-opt bound...');
bsp.G_=G;
[eig_brms] = bsp.run_online(x0,...
                            P0,...
                            x_goal,...
                            0.1,...   %epsilon
                            -1,...    %guessed eigenvalue
                            2);       %method
G_brms = bsp.G_;
data_brms = fn_get_data_SI(bsp, x_goal, G_brms, P0, eig_brms);
                                  
disp('Running BRMS using trace of covariance matrix...');
bsp.G_=G;
[eig_brm] = bsp.run_online(x0,...
                           P0,...
                           x_goal,...
                           0.1,...   %epsilon
                           -1,...    %guessed eigenvalue
                           3);       %method
G_brm = bsp.G_;
data_brm = fn_get_data_SI(bsp, x_goal, G_brm, P0, eig_brm);

%% Colors for Plotting
col_b=[0, 0.4470, 0.7410];
col_o=[0.8500, 0.3250, 0.0980];
col_y=[0.9290, 0.6940, 0.1250];
col_p=[0.4940, 0.1840, 0.5560];
col_g=[0.4660, 0.6740, 0.1880];
col_r=[0.8350, 0.0780, 0.1840];

%%  Plot Graph
disp('Plotting...');
figroadmap=figure('Name','Roadmap');
set(gcf, 'Position',  [2000, 100, 1.1*500, 1.1*450])
xlabel('X (meters)');
ylabel('Y (meters)');
axis([bsp.config_limits_(1,:), bsp.config_limits_(2,:)]);
axis square
Rectangle.plot_rects(bsp.obstacles_,[1,0,0]);
fn_plot_graph_with_edges(bsp.G_);
Rectangle.plot_rects(bsp.robot_.gps_regions_,[0, 0.4470, 0.7410, 0.7]);

%plot path BRM
% disp('Plotting path BRM...');
% hold on;
% h_brm=fn_plot_path_SI(x_goal, G_brm, [0,0,1]);

%plot path BRMS
disp('Plotting path BRMS...');
hold on;
h_brms=fn_plot_path_SI(x_goal, G_brms, [1,0,0]);

%plot path DORM
disp('Plotting path DORM...');
hold on;
h_dorm=fn_plot_path_SI(x_goal, G_dorm, [1,1,0]);

%format figure
% legend([h_brm, h_brms, h_dorm],{'BRM','BRMS','DORM'},'Location','northwest');
legend([h_brms, h_dorm],{'BRMS','DORM'},'Location','northwest');
fn_format_fig();
annotation(figroadmap,'textbox',...
    [0.704571428571428 0.131736526946108 0.156142857142857 0.0818363273453094],...
    'String',{'Single','Integrator'},...
    'FontWeight','bold',...
    'FitBoxToText','off',...
    'BackgroundColor',[1 1 1]);

%% Plot BRMS Metrics
linewidth=2;
figbrms=figure('Name','BRMS');
set(gcf, 'Position',  [2000, 100, 1.1*500, 1.1*310])
grid on;

data=data_brms;
dataLambda=[data(:).lambdaP].^2;
dataDet=[data(:).detP];
dataLambdaBound=[data(:).lambdaPBound].^2;
dataDetBound=[data(:).nuPOptimal];
dataLambdaBoundNoProp=[data(:).lambdaPBound_noProp].^2;
dataDetBoundNoProp=[data(:).phiPOptimal];

h4=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambda     ,       'color', col_r, 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\overline{\lambda}[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h1=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDet        ,       'color', 'k'  , 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\det[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h5=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBoundNoProp, 'color', col_r, 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h2=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBoundNoProp,    'color', 'k'  , 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h6=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBound,       'color', col_r, 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\ell_t^+ | \ell_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
h3=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBound   ,       'color', 'k'  , 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\nu_t^+ | \nu_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;

xl1 = xlim;
yl1 = ylim;

patch_set = fn_get_patches(data,bsp.robot_);
tempx = [data(:).x];
for i=1:length(patch_set)
    if(isempty(patch_set(i).patch))
        break;
    end
    ylimits = get(gca,'YLim');
    xlimit1=bsp.robot_.dt_*patch_set(i).patch(1);
    xlimit2=bsp.robot_.dt_*patch_set(i).patch(end);
    hpatch1 = patch([xlimit1,xlimit1,xlimit2,xlimit2],[ylimits(1),ylimits(2),ylimits(2),ylimits(1)],'y');
    set(hpatch1,'facealpha',.7)
    set(hpatch1,'edgecolor',col_b)
    set(hpatch1,'facecolor',col_b)
    set(gca,'children',flipud(get(gca,'children')))
end

h4=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambda     ,       'color', col_r, 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\overline{\lambda}[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h1=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDet        ,       'color', 'k'  , 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\det[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h5=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBoundNoProp, 'color', col_r, 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h2=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBoundNoProp,    'color', 'k'  , 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h6=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBound,       'color', col_r, 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\ell_t^+ | \ell_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
h3=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBound   ,       'color', 'k'  , 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\nu_t^+ | \nu_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;

ldg=legend([h1,h4,h2,h5,h3,h6],'Interpreter','latex','FontSize',16,'NumColumns',3, 'location', 'southoutside','EdgeColor',[1 1 1]);
fn_format_fig();
pause(0.1); %without pause figure isn't formatted correct for some reason

% yticks([10^(-8),10^(-4),10^(0)]);
% axis([[0,165],yl1]);
% 
% annotation(figbrms,'textbox',...
%     [0.781357142857141 0.861671469740634 0.100785714285716 0.0720461095100863],...
%     'String','BRMS',...
%     'FontWeight','bold',...
%     'FitBoxToText','off',...
%     'BackgroundColor',[1 1 1]);

%% Plot DORM Metrics
linewidth=2;
figdorm=figure('Name','DORM');
set(gcf, 'Position',  [2000, 100, 1.1*500, 1.1*310])
grid on;

data=data_dorm;
dataLambda=[data(:).lambdaP].^2;
dataDet=[data(:).detP];
dataLambdaBound=[data(:).lambdaPBound].^2;
dataDetBound=[data(:).nuPOptimal];
dataLambdaBoundNoProp=[data(:).lambdaPBound_noProp].^2;
dataDetBoundNoProp=[data(:).phiPOptimal];

h4=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambda     ,       'color', col_r, 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\overline{\lambda}[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h1=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDet        ,       'color', 'k'  , 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\det[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h5=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBoundNoProp, 'color', col_r, 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h2=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBoundNoProp,    'color', 'k'  , 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h6=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBound,       'color', col_r, 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\ell_t^+ | \ell_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
h3=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBound   ,       'color', 'k'  , 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\nu_t^+ | \nu_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;

xl2 = xlim;
yl2 = ylim;

patch_set = fn_get_patches(data,bsp.robot_);
tempx = [data(:).x];
for i=1:length(patch_set)
    if(isempty(patch_set(i).patch))
        break;
    end
    ylimits = get(gca,'YLim');
    xlimit1=bsp.robot_.dt_*patch_set(i).patch(1);
    xlimit2=bsp.robot_.dt_*patch_set(i).patch(end);
    hpatch1 = patch([xlimit1,xlimit1,xlimit2,xlimit2],[ylimits(1),ylimits(2),ylimits(2),ylimits(1)],'y');
    set(hpatch1,'facealpha',.7)
    set(hpatch1,'edgecolor',col_b)
    set(hpatch1,'facecolor',col_b)
    set(gca,'children',flipud(get(gca,'children')))
end

h4=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambda     ,       'color', col_r, 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\overline{\lambda}[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h1=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDet        ,       'color', 'k'  , 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\det[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h5=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBoundNoProp, 'color', col_r, 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h2=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBoundNoProp,    'color', 'k'  , 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
h6=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBound,       'color', col_r, 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\ell_t^+ | \ell_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
h3=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBound   ,       'color', 'k'  , 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\nu_t^+ | \nu_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;


ldg=legend([h1,h4,h2,h5,h3,h6],'Interpreter','latex','FontSize',16,'NumColumns',3, 'location', 'southoutside','EdgeColor',[1 1 1]);
fn_format_fig();
pause(0.1); %without pause figure isn't formatted correct for some reason

% yticks([10^(-8),10^(-4),10^(0)]);
% axis([[0,190],yl2]);
% 
% annotation(figdorm,'textbox',...
%     [0.781357142857141 0.861671469740634 0.106142857142857 0.0720461095100863],...
%     'String','DORM',...
%     'FontWeight','bold',...
%     'FitBoxToText','off',...
%     'BackgroundColor',[1 1 1]);

%% Plot LQR States and Control Inputs
%BRMS
linewidth=2;
figure('Name','LQR BRMS','WindowStyle', 'docked');

data = data_brms;
x = [data.x];
u = [data.u];

subplot(211);
plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(1,:),'color',col_b,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(2,:),'color',col_o,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
legend('$v_x(t)$','$v_y(t)$','Interpreter','latex');
title('$\mathbf{\bar{u}}_{1:N-1}$','Interpreter','latex');
fn_format_fig();

subplot(212);
plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(1,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(2,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
legend('$x(t)$','$y(t)$','Interpreter','latex');
title('$\mathbf{\bar{x}}_{1:N}$','Interpreter','latex');
fn_format_fig();

%DORM
linewidth=2;
figure('Name','LQR DORM','WindowStyle', 'docked');

data = data_dorm;
x = [data.x];
u = [data.u];

subplot(211);
plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(1,:),'color',col_b,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(2,:),'color',col_o,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
legend('$v_x(t)$','$v_y(t)$','Interpreter','latex');
title('$\mathbf{\bar{u}}_{1:N-1}$','Interpreter','latex');
fn_format_fig();

subplot(212);
plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(1,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(2,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
legend('$x(t)$','$y(t)$','Interpreter','latex');
title('$\mathbf{\bar{x}}_{1:N}$','Interpreter','latex');
fn_format_fig();