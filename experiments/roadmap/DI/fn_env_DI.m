function [config_limits, obstacles, gps_regions] = fn_env_DI(n_obs,...
                                                             side_obs,...
                                                             n_gps,...
                                                             side_gps)
% generates an environment containing rectangles representing obstacles
% and gps regions
%
%	Authors: Jared Strader

%% State Space
config_limits = [-10, 10;...
                 -10, 10];

%% Generate obstacles
obstacles = Rectangle.generate_rand_rects(config_limits(1,:),...
                                          config_limits(2,:),...
                                          n_obs,...
                                          side_obs);

%% Generate gps regions
gps_regions = Rectangle.generate_rand_rects(config_limits(1,:),...
                                            config_limits(2,:),...
                                            n_gps,...
                                            side_gps);

%% Plot Environment 
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

end

