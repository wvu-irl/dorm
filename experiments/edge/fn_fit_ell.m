function [ell_data, xip] = fn_fit_ell(bsp, x, u, data)

%% Colors
col_b=[0, 0.4470, 0.7410];
col_o=[0.8500, 0.3250, 0.0980];
col_y=[0.9290, 0.6940, 0.1250];
col_p=[0.4940, 0.1840, 0.5560];
col_g=[0.4660, 0.6740, 0.1880];
col_r=[0.6350, 0.0780, 0.1840];

%% 
xi_params = fn_get_tf_dopt_params(x,u);

%% Plot Set of Transfer Functions
incre = 100;
ellvec = 0:incre:100000;

%compute tf parameters for set of alphas
P0 = data(1).P;
k=1;
for i=1:length(ellvec)
    %compute transfer function, gamma, and determinant
    [ell_data(k).xi, ell_data(k).GammaVal] = fn_get_tf_dopt(x,u,ellvec(i));
    ell_data(k).nuP = (det(P0)*ell_data(k).xi(1) + ell_data(k).xi(3))/(det(P0)*ell_data(k).xi(2) + ell_data(k).xi(4));
    ell_data(k).logNuP = log(ell_data(k).nuP);
    ell_data(k).alpha = ellvec(i);
    
    %check if xi and gamma entries are equivalent (if not return error)!
    if(ell_data(k).xi(1) ~= ell_data(k).GammaVal(1,1) ||...
       ell_data(k).xi(2) ~= ell_data(k).GammaVal(2,1) ||...
       ell_data(k).xi(3) ~= ell_data(k).GammaVal(1,2) ||...
       ell_data(k).xi(4) ~= ell_data(k).GammaVal(2,2))
        error('\xi values do not match \Gamma!');
    end
    k=k+1;
end

%compute fit for tf parameters for alpha
tempalpha = [100,1000,10000,100000];
tempidx(1) = find(ellvec == tempalpha(1));
tempidx(2) = find(ellvec == tempalpha(2));
tempidx(3) = find(ellvec == tempalpha(3));
tempidx(4) = find(ellvec == tempalpha(4));
tempxi = [ell_data(:).xi];
xip(1,:) = polyfit(tempalpha,tempxi(1,tempidx),3);
xip(2,:) = polyfit(tempalpha,tempxi(2,tempidx),3);
xip(3,:) = polyfit(tempalpha,tempxi(3,tempidx),3);
xip(4,:) = polyfit(tempalpha,tempxi(4,tempidx),3);

%show fit
% for i=1:length(alphavec) 
for i=1:length(ellvec) 
    xifit(1,i) = polyval(xip(1,:),ellvec(i));
    xifit(2,i) = polyval(xip(2,:),ellvec(i));
    xifit(3,i) = polyval(xip(3,:),ellvec(i));
    xifit(4,i) = polyval(xip(4,:),ellvec(i));
    ell_data(i).xifit = xifit(:,i);
end

fiterr = [100*(tempxi(1,:)-xifit(1,:))./tempxi(1,:);...
          100*(tempxi(2,:)-xifit(2,:))./tempxi(2,:);...
          100*(tempxi(3,:)-xifit(3,:))./tempxi(3,:);...
          100*(tempxi(4,:)-xifit(4,:))./tempxi(4,:)];

linewidth=2;

figure('Name','Analysis: det(P) using Xi','WindowStyle', 'docked');
subplot(211)
plot(1:length(ell_data), [ell_data(:).nuP],'linestyle','-','linewidth',linewidth); hold on;
subplot(212)
plot(1:length(ell_data), [ell_data(:).logNuP],'linestyle','-','linewidth',linewidth); hold on;
legend('det(P)','log(det(P))');
fn_format_fig();

figure('Name','Analysis: Xi','WindowStyle', 'docked');
tempxi = [ell_data(:).xi];
subplot(411)
plot(1:length(ell_data), [tempxi(1,:)],'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), [xifit(1,:)],'r:','linewidth',linewidth); hold on;
legend('a, \xi_{00}');
fn_format_fig();
subplot(412)
plot(1:length(ell_data), [tempxi(3,:)],'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), [xifit(3,:)],'r:','linewidth',linewidth); hold on;
legend('b, \xi_{01}');
fn_format_fig();
subplot(413)
plot(1:length(ell_data), [tempxi(2,:)],'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), [xifit(2,:)],'r:','linewidth',linewidth); hold on;
legend('c, \xi_{10}');
fn_format_fig();
subplot(414)
plot(1:length(ell_data), [tempxi(4,:)],'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), [xifit(4,:)],'r:','linewidth',linewidth); hold on;
legend('d, \xi_{11}');
fn_format_fig();

figure('Name','Analysis: Fit Error','WindowStyle', 'docked');
plot(1:length(ell_data), fiterr(1,:),'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), fiterr(3,:),'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), fiterr(2,:),'linestyle','-','linewidth',linewidth); hold on;
plot(1:length(ell_data), fiterr(4,:),'linestyle','-','linewidth',linewidth); hold on;
xlabel('\alpha')
legend('$(a - \hat{a})a/$','$(b - \hat{b})b/$','$(c - \hat{c})c/$','$(d - \hat{d})d/$','Interpreter','latex');
fn_format_fig();
end

