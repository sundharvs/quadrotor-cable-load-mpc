function [a,b] = MPCLinearConstraints2(params)
% params = params();

% Just doing a bunch of annoying matrix stuff to get a modular input norm constraint 
a = [-1;1];
a = repmat({a}, 1, 4);  
a = blkdiag(a{:});
a = repmat({a}, 1, params.predHorizon+1);  
a = blkdiag(a{:});

b = repmat([-params.minThrust; params.maxThrust],4,1);
b = repmat(b,params.predHorizon+1,1);




end