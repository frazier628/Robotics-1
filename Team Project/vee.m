function k = vee(K)
% Takes in cross-product matrix and returns original vector form
    k = [-K(2,3); K(1,3); -K(1,2)];
end