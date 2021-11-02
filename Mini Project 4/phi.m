%
% phi.m
%
% propagation of spatial velocity
%
function Phi=phi(R,p)

Phi=[R zeros(3,3);-R*hat(p) R];

end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)

khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end