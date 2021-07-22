% Author: Jared Strader
rng(33); %22, 33, 46, 58
clear all; close all; clc;

%% Parameters (user-defined)
% Environment
config_limits = [0, 1;...
                 0, 1];
             
% obstacles(1) = Rectangle.generate_rect(0,0.6,0.4,0.05);
% gps_regions(1) = Rectangle.generate_rect(0,1,0.4,0.4);

obstacles(1) = Rectangle.generate_rect(0,0.4,0.3,0.05);
gps_regions(1) = Rectangle.generate_rect(0,1,0.3,0.6);

% Robot
param_sigma_vx=sqrt(0.1);
param_sigma_vy=sqrt(0.1);
param_sigma_x=sqrt(1);
param_sigma_y=sqrt(1);
param_dt=1;
robot = SingleIntegrator2DwithGPS(param_sigma_vx,...
                                  param_sigma_vy,...
                                  param_sigma_x,...
                                  param_sigma_y,...
                                  param_dt,...
                                  gps_regions);
                              
%% Plot Obstacles and GPS Regions
% figure;
% axis([config_limits(1,1),...
%       config_limits(1,2),...
%       config_limits(2,1),...
%       config_limits(2,2)]);
% 
% %gps regions
% for i=1:length(gps_regions)
%     hold on;
%     rectangle('Position',gps_regions(i).rect,'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor','k');
% end
%   
% %obstacles
% for i=1:length(obstacles)
%     hold on;
%     rectangle('Position',obstacles(i).rect,'FaceColor',[1 0 0],'EdgeColor','k');
% end

%% Construct DORM
bsp = BSP(robot, config_limits, [], obstacles, 0.05);
bsp.enable_hammersley_=true;
bsp.run_offline_qrm(2500);


%% Save Roadmap
filename = sprintf('bsp_%s.mat', datestr(now,'mm-dd-yyyy HH-MM'));
save(filename,'bsp');

%% Plot
disp('Plotting...');
figure;
xlabel('X');
ylabel('Y');
axis([bsp.config_limits_(1,:), bsp.config_limits_(2,:)]);
axis square
Rectangle.plot_rects(bsp.robot_.gps_regions_,[0.3010 0.7450 0.9330]);
Rectangle.plot_rects(bsp.obstacles_,[1,0,0]);
fn_plot_graph_with_edges(bsp.G_);
fn_format_fig();
