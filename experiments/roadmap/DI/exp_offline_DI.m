% Author: Jared Strader
rng(46); %22, 33, 46, 58
clear all; close all; clc;

%% Parameters (user-defined)
% Environment
n_obs=30;
side_obs=[1,2];
n_gps=3;
side_gps=[2,4];
[config_limits, obstacles, gps_regions] = fn_env_DI(n_obs,...
                                                    side_obs,...
                                                    n_gps,...
                                                    side_gps);

% Robot
param_sigma_ax=sqrt(0.01);
param_sigma_ay=sqrt(0.01);
param_sigma_x=sqrt(0.1);
param_sigma_y=sqrt(0.1);
param_dt=0.02;
robot = DoubleIntegrator2DwithGPS(param_sigma_ax,...
                                  param_sigma_ay,...
                                  param_sigma_x,...
                                  param_sigma_y,...
                                  param_dt,...
                                  gps_regions);

%% Construct DORM
bsp = BSP(robot, config_limits, [], obstacles, 1);
bsp.run_offline(2000);

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