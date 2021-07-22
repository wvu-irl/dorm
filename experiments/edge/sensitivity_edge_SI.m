% Author: Jared Strader
clear all; clc;
close all;
%% Settings
%turn gps on or off
use_gps = true;

%start state and goal state
x0 = [0,0]';
xT = [5,1]';
% xT = [rand*5, rand*5]';

%initial covariance
P0 = 1*eye(2);

%noises
param_sigma_vx=1*sqrt(0.1);
param_sigma_vy=1*sqrt(0.1);
param_sigma_x=1*sqrt(0.01);
param_sigma_y=1*sqrt(0.01);

%% Parameters
param_dt=0.1;

%% Environment
xspace = [-50,50];
yspace = [-50,50];

gps_regions(1) = Rectangle.generate_rect(1,...
                                         1.2,...
                                         1,...
                                         1.4);
                                 
gps_regions(2) = Rectangle.generate_rect(3,...
                                         1.2,...
                                         1,...
                                         1.4);
                                     
if(~use_gps)
    gps_regions=[];
end

%% Model
robot = SingleIntegrator2DwithGPS(param_sigma_vx,...
                                  param_sigma_vy,...
                                  param_sigma_x,...
                                  param_sigma_y,...
                                  param_dt,...
                                  gps_regions);

%% BSP (used for computing cost metrics and transfer functions)
bsp = BSP(robot, [], [], [], []);

%% Edge
%states and control inputs for traverse edge between x0 and xT
[x,u] = bsp.robot_.get_states_and_control_inputs(x0,xT);

%% Approximate Maximum Eigenvalue
P=P0;
for t=1:size(x,2)-1
    %states and controls inputs at time step t
    xt=x(:,t);
    ut=u(:,t);

    %process matrices
    L = bsp.robot_.get_process_noise_jacobian(xt,ut);
    Q = bsp.robot_.get_process_noise_covariance(xt);
    Q_tilde = L*Q*L';
    F = bsp.robot_.get_process_jacobian(xt,ut);
    P = F*P*F' + Q_tilde;
end
ell=max(eig(P));

%% Get Transfer Functions (i.e., multi-step and one-step updates)
xi = bsp.get_tf_dopt(x,u,ell);
xi_params = bsp.get_tf_dopt_params(x,u);

%% colors
col_b=[0, 0.4470, 0.7410];
col_o=[0.8500, 0.3250, 0.0980];
col_y=[0.9290, 0.6940, 0.1250];
col_p=[0.4940, 0.1840, 0.5560];
col_g=[0.4660, 0.6740, 0.1880];
col_r=[0.6350, 0.0780, 0.1840];

%% Plot LQR States and Control Inputs
linewidth=2;
% figure('Name','LQR','WindowStyle', 'docked');
figure('Name','LQR');
set(gcf, 'Position',  [100, 100, 1.1*500, 1.1*400])
    
subplot(211);
plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(1,:),'color',col_b,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(2,:),'color',col_o,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
legend('$a_x(t)$','$a_y(t)$','Interpreter','latex','FontSize',14,'Orientation','horizontal');
title('$\mathbf{\bar{u}}_{1:N-1}$','Interpreter','latex');
fn_format_fig();

subplot(212);
plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(1,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(2,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
legend('$x(t)$','$y(t)$','$v_x(t)$','$v_y(t)$','Interpreter','latex','FontSize',14,'Orientation','horizontal');
title('$\mathbf{\bar{x}}_{1:N}$','Interpreter','latex');
fn_format_fig();

%% Plot Trajectory
linewidth=2;
figure('Name','Traj');
set(gcf, 'Position',  [100, 100, 1.1*500, 1.1*400])

%plot trajectory
Rectangle.plot_rects(gps_regions,col_b);
h1=plot(x(1,:),x(2,:),'k','linewidth',linewidth); grid on; hold on;
h2=plot(x(1,1), x(2,1),'color','g','Marker','o','MarkerFaceColor','g','MarkerSize',10,'DisplayName','Start'); hold on;
h3=plot(x(1,end), x(2,end),'color','r','Marker','o','MarkerFaceColor','r','MarkerSize',10,'DisplayName','Goal'); hold on;
% title('$[\mathbf{\bar{x}},\mathbf{\bar{y}}]$','Interpreter','latex');
axis image;
legend([h2,h3],'FontSize',12);
xlabel('X (meters)');
ylabel('Y (meters)');
fn_format_fig();

%% Analysis
eig_vals=[0,0.1,1,10];
hfig=figure;
set(gcf, 'Position',  [100, 100, 1.1*500, 1.1*450])

%plot gps regions
data = fn_get_data_edge(bsp,x,u,P0);
ylimits = get(gca,'YLim');
tempx = [data(:).x];
%patch1
[~,temp]=min(abs(tempx(1,:)-1));
xlimit1 = bsp.robot_.dt_*(temp-1);
[~,temp]=min(abs(tempx(1,:)-2));
xlimit2 = bsp.robot_.dt_*(temp-1);
hpatch1 = patch([xlimit1,xlimit1,xlimit2,xlimit2],[ylimits(1),ylimits(2),ylimits(2),ylimits(1)],'y');
set(hpatch1,'facealpha',.7)
set(hpatch1,'edgecolor',col_b)
set(hpatch1,'facecolor',col_b)
%patch2
[~,temp]=min(abs(tempx(1,:)-3));
xlimit3 = bsp.robot_.dt_*(temp-1);
[~,temp]=min(abs(tempx(1,:)-4));
xlimit4 = bsp.robot_.dt_*(temp-1);
hpatch2 = patch([xlimit3,xlimit3,xlimit4,xlimit4],[ylimits(1),ylimits(2),ylimits(2),ylimits(1)],'y'); hold on;
set(hpatch2,'facealpha',.7)
set(hpatch2,'edgecolor',col_b)
set(hpatch2,'facecolor',col_b)
set(gca,'children',flipud(get(gca,'children')))

%plot metrics
for i=1:length(eig_vals)
    data = fn_get_data_sensitivity_edge(bsp,x,u,P0, eig_vals(i));
    hold on;
    semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).phiPOptimal],'linestyle','-','linewidth',linewidth,'DisplayName','$\overline{\nu}_t^+ |\det[\Sigma_{t-1}^+]$'); hold on;
%     plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).phiPOptimal],'linestyle','-','linewidth',linewidth,'DisplayName','$\overline{\nu}_t^+ |\det[\Sigma_{t-1}^+]$'); hold on;
    grid on;
    fn_format_fig();
end

% annotation(hfig,'arrow',[0.401785714285714 0.198214285714284],...
%     [0.904191616766467 0.904191616766465],'LineWidth',2);
% annotation(hfig,'arrow',[0.401785714285714 0.33392857142857],...
%     [0.868263473053892 0.866267465069856],'LineWidth',2);
% annotation(hfig,'textbox',...
%     [0.401 0.862275449101793 0.234714285714286 0.0499001996007959],...
%     'String',{'GNSS regions'},...
%     'LineStyle','none',...
%     'FontWeight','bold',...
%     'FitBoxToText','off');
% annotation(hfig,'textbox',...
%     [0.727785714285712 0.842315369261476 0.161500000000001 0.0838323353293341],...
%     'String',{'Single','Integrator'},...
%     'FontWeight','bold',...
%     'FitBoxToText','off',...
%     'BackgroundColor',[1 1 1]);