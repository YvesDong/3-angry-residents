function k_base_real = ComputeBaseStateIndex(stateSpace, map)
global BASE K
M =  size(map, 1);
N =  size(map, 2);
for x = 1:M
    for y = 1:N
        if map(x,y) == BASE
            base = [x,y];
            break
        end
    end
end
% the loc of base corresponds to k_base in stateSpace, whose 3rd state is 0, no package
for f = 1:K/2
    if stateSpace(2*f-1,1) == base(1) && stateSpace(2*f-1,2) == base(2)
        k_base_real = 2*f-1;
        break
    end
end
end