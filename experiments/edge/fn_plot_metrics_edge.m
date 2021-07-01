function [] = fn_plot_metrics_DI(bsp, x, u, data)
% global robot_;
% global param_alpha_;

%% Colors
col_b=[0, 0.4470, 0.7410];
col_o=[0.8500, 0.3250, 0.0980];
col_y=[0.9290, 0.6940, 0.1250];
col_p=[0.4940, 0.1840, 0.5560];
col_g=[0.4660, 0.6740, 0.1880];
col_r=[0.6350, 0.0780, 0.1840];

%% Plot Bounds for Determinant and Max Eigenvalue
linewidth=2;
% hlogd=figure('Name','Metrics','WindowStyle', 'docked');

%plot data
use_semilog=true;
if(use_semilog)
    h3=semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).nuPOptimal],'color','k','linestyle','--','linewidth',linewidth,'DisplayName','$\overline{\nu}_t^+ | \overline{\nu}_{t-1}^+$'); hold on;
    h2=semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).phiPOptimal],'color','k','linestyle',':','linewidth',linewidth,'DisplayName','$\overline{\nu}_t^+ |\det[\Sigma_{t-1}^+]$'); hold on;
    h1=semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).detP],'color','k','linewidth',linewidth,'DisplayName','$\det[\Sigma_{t-1}^+]$'); hold on;
    h6=semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).lambdaPBound].^bsp.robot_.x_dim_,'color','r','linestyle','--','linewidth',linewidth,'DisplayName','$\ell_t^+\, | \ell_{t-1}^+$'); hold on;
    h5=semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).lambdaPNotProp].^bsp.robot_.x_dim_,'color','r','linestyle',':','linewidth',linewidth,'DisplayName','$\ell_t^+\, | \overline{\lambda}[\Sigma_{t-1}^+]$'); hold on;
    h4=semilogy([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).lambdaP].^bsp.robot_.x_dim_,'color','r','linestyle','-','linewidth',linewidth,'DisplayName','$\overline{\lambda}[\Sigma_{t-1}^+]$'); hold on;
else
    h3=plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).nuPOptimal],'color','k','linestyle','--','linewidth',linewidth,'DisplayName','$\nu_t^+ | \nu_{t-1}^+$'); hold on;
    h2=plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).phiPOptimal],'color','k','linestyle',':','linewidth',linewidth,'DisplayName','$\nu_t^+ |\det[\Sigma_{t-1}^+]$'); hold on;
    h1=plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).detP],'color','k','linewidth',linewidth,'DisplayName','$\det[\Sigma_{t-1}^+]$'); hold on;
    h6=plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).lambdaPBound].^bsp.robot_.x_dim_,'color','r','linestyle','--','linewidth',linewidth,'DisplayName','$\ell_t^+ | \ell_{t-1}^+$'); hold on;
    h5=plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).lambdaPNotProp].^bsp.robot_.x_dim_,'color','r','linestyle',':','linewidth',linewidth,'DisplayName','$\ell_t^+ | \overline{\lambda}[\Sigma_{t-1}^+]$'); hold on;
    h4=plot([bsp.robot_.dt_*(0:1:size(x,2)-1)], [data(:).lambdaP].^bsp.robot_.x_dim_,'color','r','linestyle','-','linewidth',linewidth,'DisplayName','$\overline{\lambda}[\Sigma_{t-1}^+]$'); hold on;
end

%plot patch
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

grid on;
% xlabel('Time (seconds)');
ldg=legend([h1,h4,h2,h5,h3,h6],'Interpreter','latex','FontSize',16,'NumColumns',3, 'location', 'southoutside','EdgeColor',[1 1 1]);
fn_format_fig();

end

