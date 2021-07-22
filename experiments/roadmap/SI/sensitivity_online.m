% Author: Jared Strader
clear all; close all; clc;
disp('NOTE: If exp_online_SI.m is edited to contain the correct filename, an error is returned. See READM.md for details on reproducing the long range experiments.');
data_sense=[];
eig_vals=[0,0.1,1,10];
for dataidx=1:length(eig_vals)
    clearvars -except eig_vals data_sense dataidx figroadmap
    
    %% Guessed Eigenvalue for Sensitivity Analysis
    eig_dorm = eig_vals(dataidx);
    disp(['Running DORM for eig=', num2str(eig_dorm)]);

    %% Load Offline
    load('bsp3');

    %%  Plot Graph
    if(dataidx==1)
        disp('Plotting graph...');
        figure(1);
        set(gcf, 'Position',  [2000, 100, 1.1*500, 1.1*450])
        xlabel('X (meters)');
        ylabel('Y (meters)');
        axis([bsp.config_limits_(1,:), bsp.config_limits_(2,:)]);
        axis square
        Rectangle.plot_rects(bsp.obstacles_,[1,0,0]);
        fn_plot_graph_with_edges(bsp.G_);
        Rectangle.plot_rects(bsp.robot_.gps_regions_,[0, 0.4470, 0.7410, 0.7]);
    end
    
    %% Start State, Goal State, Initial Belief
    x0 = [0.1;0.1];
    P0 = 1*eye(2);
    x_goal = [0.9;0.9];
    
    %% Search DORM
    disp('Running BRMS using d-opt bound...');
    [eig_dorm] = bsp.run_online(x0,...
                                P0,...
                                x_goal,...
                                0.1,...      %epsilon
                                eig_dorm,... %guessed eigenvalue
                                1);          %method
    disp(['Completed DORM for eig=', num2str(eig_dorm)]);
%     data_sense(dataidx).bsp = copy(bsp);
%     data_dorm = fn_get_data_SI(bsp, x_goal, bsp.G_, P0, eig_dorm);
%     data_sense(dataidx).data_dorm=data_dorm;
    
    %% Plot path
    disp(['Plotting DORM path for eig=',num2str(eig_vals(dataidx))]);
    figure(1);
    hold on;
    cmap_jet = jet;
    color_idx = floor(length(cmap_jet)*(dataidx/length(eig_vals)));
    path_color = cmap_jet(color_idx,:);
    fn_plot_path_SI(x_goal, bsp.G_, path_color);
end

%% plot paths DORM
% cmap_jet = jet;
% for i=1:length(data_sense)
%     disp(['Plotting DORM path for eig=',num2str(eig_vals(i))]);
%     color_idx = floor(length(cmap_jet)*(i/length(data_sense)));
%     path_color = cmap_jet(color_idx,:);
%     hold on;
%     h_dorm=fn_plot_path_SI(x_goal, data_sense(dataidx).bsp.G_, path_color);
%     pause;
% end

%% Colors for Plotting
% col_b=[0, 0.4470, 0.7410];
% col_o=[0.8500, 0.3250, 0.0980];
% col_y=[0.9290, 0.6940, 0.1250];
% col_p=[0.4940, 0.1840, 0.5560];
% col_g=[0.4660, 0.6740, 0.1880];
% col_r=[0.8350, 0.0780, 0.1840];
% 
% %format figure
% % legend([h_brm, h_brms, h_dorm],{'BRM','BRMS','DORM'},'Location','northwest');
% % legend([h_dorm],{'DORM'},'Location','northwest');
% fn_format_fig();
% annotation(figroadmap,'textbox',...
%     [0.704571428571428 0.131736526946108 0.156142857142857 0.0818363273453094],...
%     'String',{'Single','Integrator'},...
%     'FontWeight','bold',...
%     'FitBoxToText','off',...
%     'BackgroundColor',[1 1 1]);

%% Plot DORM Metrics
% linewidth=2;
% figdorm=figure('Name','DORM');
% set(gcf, 'Position',  [2000, 100, 1.1*500, 1.1*310])
% grid on;
% 
% data=data_dorm;
% dataLambda=[data(:).lambdaP].^2;
% dataDet=[data(:).detP];
% dataLambdaBound=[data(:).lambdaPBound].^2;
% dataDetBound=[data(:).nuPOptimal];
% dataLambdaBoundNoProp=[data(:).lambdaPBound_noProp].^2;
% dataDetBoundNoProp=[data(:).phiPOptimal];
% 
% h4=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambda     ,       'color', col_r, 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\overline{\lambda}[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h1=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDet        ,       'color', 'k'  , 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\det[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h5=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBoundNoProp, 'color', col_r, 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h2=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBoundNoProp,    'color', 'k'  , 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h6=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBound,       'color', col_r, 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\ell_t^+ | \ell_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
% h3=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBound   ,       'color', 'k'  , 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\nu_t^+ | \nu_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
% 
% xl2 = xlim;
% yl2 = ylim;
% 
% patch_set = fn_get_patches(data,bsp.robot_);
% tempx = [data(:).x];
% for i=1:length(patch_set)
%     if(isempty(patch_set(i).patch))
%         break;
%     end
%     ylimits = get(gca,'YLim');
%     xlimit1=bsp.robot_.dt_*patch_set(i).patch(1);
%     xlimit2=bsp.robot_.dt_*patch_set(i).patch(end);
%     hpatch1 = patch([xlimit1,xlimit1,xlimit2,xlimit2],[ylimits(1),ylimits(2),ylimits(2),ylimits(1)],'y');
%     set(hpatch1,'facealpha',.7)
%     set(hpatch1,'edgecolor',col_b)
%     set(hpatch1,'facecolor',col_b)
%     set(gca,'children',flipud(get(gca,'children')))
% end
% 
% h4=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambda     ,       'color', col_r, 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\overline{\lambda}[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h1=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDet        ,       'color', 'k'  , 'linestyle', '-',  'linewidth', linewidth, 'DisplayName', '$\det[\Sigma_{t}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h5=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBoundNoProp, 'color', col_r, 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h2=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBoundNoProp,    'color', 'k'  , 'linestyle', ':', 'linewidth',  linewidth, 'DisplayName', '$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); grid on; xlabel('t (seconds)'); hold on;
% h6=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataLambdaBound,       'color', col_r, 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\ell_t^+ | \ell_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
% h3=semilogy(bsp.robot_.dt_*(0:1:length(data)-1), dataDetBound   ,       'color', 'k'  , 'linestyle', '--', 'linewidth', linewidth, 'DisplayName', '$\nu_t^+ | \nu_{t-1}^+$'); grid on; xlabel('t (seconds)'); hold on;
% 
% 
% ldg=legend([h1,h4,h2,h5,h3,h6],'Interpreter','latex','FontSize',16,'NumColumns',3, 'location', 'southoutside','EdgeColor',[1 1 1]);
% fn_format_fig();
% pause(0.1); %without pause figure isn't formatted correct for some reason
% 
% % yticks([10^(-8),10^(-4),10^(0)]);
% % axis([[0,190],yl2]);
% % 
% % annotation(figdorm,'textbox',...
% %     [0.781357142857141 0.861671469740634 0.106142857142857 0.0720461095100863],...
% %     'String','DORM',...
% %     'FontWeight','bold',...
% %     'FitBoxToText','off',...
% %     'BackgroundColor',[1 1 1]);

%% Plot LQR States and Control Inputs
% %DORM
% linewidth=2;
% figure('Name','LQR DORM','WindowStyle', 'docked');
% 
% data = data_dorm;
% x = [data.x];
% u = [data.u];
% 
% subplot(211);
% plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(1,:),'color',col_b,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
% plot(bsp.robot_.dt_*(0:1:size(u,2)-1),u(2,:),'color',col_o,'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
% legend('$v_x(t)$','$v_y(t)$','Interpreter','latex');
% title('$\mathbf{\bar{u}}_{1:N-1}$','Interpreter','latex');
% fn_format_fig();
% 
% subplot(212);
% plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(1,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
% plot(bsp.robot_.dt_*(0:1:size(x,2)-1),x(2,:),'linewidth',linewidth); grid on; xlabel('t (seconds)'); hold on;
% legend('$x(t)$','$y(t)$','Interpreter','latex');
% title('$\mathbf{\bar{x}}_{1:N}$','Interpreter','latex');
% fn_format_fig();