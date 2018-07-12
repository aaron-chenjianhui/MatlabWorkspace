function [ k, b, err, rank ] = FindLine( x, y )
%FindLine 
%   

% set RANSAC options
options.epsilon = 1e-6;
options.P_inlier = 1.2;
options.sigma = 1;
options.est_fun = @estimate_line;
options.man_fun = @error_line;
options.mode = 'MSAC';
options.Ps = [];
options.notify_iters = [];
options.min_iters = 100;
options.fix_seed = false;
options.reestimate = true;
options.stabilize = false;

data(1, :) = x;
data(2, :) = y;

[result, opt] = RANSAC(data, options);
k = -result.Theta(1)/result.Theta(2);
b = -result.Theta(3)/result.Theta(2);

err = result.E;
rank = result.r;

end

