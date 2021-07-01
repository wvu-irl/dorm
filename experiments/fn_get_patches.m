function [patch_set] = fn_get_patches(data,robot)

patches=[];
for i=1:length(data)
    if(Rectangle.is_in_rect_pts(data(i).x,robot.gps_regions_))
        patches = [patches i];
    end
end

patch_set(1).patch=[];
idx=1;
for i=1:length(patches)
    if(i~=1)
        if(patches(i) - patches(i-1) > 1)
            idx=idx+1;
            patch_set(idx).patch=[];
        end
    end
    
    patch_set(idx).patch = [patch_set(idx).patch patches(i)];
end

end

