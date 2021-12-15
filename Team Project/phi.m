%
% phi.m
%
% propagation of spatial velocity
%
function phimat=phi(R,p)
    phimat=[R zeros(3,3); -R*hat(p) R];
end
