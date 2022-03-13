function remove_rclm
%REMOVE_RCLM Remove MATLAB Wrapper ROS CLIENT LIRBARY from the path
%   REMOVE_RCLM removes MATLAB Wrapper ROS CLIENT LIRBARY and all installed 
%   packages from MATLAB search path
%
%   See also BUILD_RCLM, M_CREATE_PKG

path = mfilename('fullpath');
file_rmv = 'remove_rclm';
folder_share = 'share';
share_path = path(1:(end-length(file_rmv)));
setup_path = path(1:(end-length([folder_share '/' file_rmv])));
% TO DO : remove package paths from the search path
load([share_path 'packages.mat'],"package_name_installed");
for package = package_name_installed
    rmpath([setup_path package{:}]);
    rmpath([setup_path package{:} '/script']);
    rmpath([setup_path package{:} '/model']);
    rmpath([setup_path package{:} '/class']);
    fprintf('Package %s has been removed from your search path.\n',package{:})
end

rmpath(share_path);
end


