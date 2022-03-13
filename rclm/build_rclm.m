function build_rclm
%BUILD_RCLM Setup MATLAB Wrapper ROS CLIENT LIRBARY
%   BUILD_RCLM adds all packages from "package_name.m" to MATLAB search path
%
%   See also REMOVE_RCLM (document available only after setup), 
%   M_CREATE_PKG (document available only after setup)


addpath('share')
package_name;
files = dir;
dirFlags = [files.isdir];
subFolders = files(dirFlags); % A structure with extra info.
subFolderNames = {subFolders(3:end).name};
package_name_installed = {};
for package = packages
    if any(strcmp(subFolderNames,package{:}))
        addpath(package{:})
        addpath([package{:} '/script'])
        addpath([package{:} '/model'])
        addpath([package{:} '/class'])
        fprintf('Package %s has been added to your search path.\n',package{:})
        package_name_installed{end+1} = package{:};
    else
        fprintf('Package %s does not exist in this directory.\n',package{:})
    end
end
save('share/packages.mat',"package_name_installed");
savepath
ros2genmsg([pwd '/custom_interface']);
end