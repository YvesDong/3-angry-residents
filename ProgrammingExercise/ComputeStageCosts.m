function G = ComputeStageCosts( stateSpace, map, myP)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map)
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and
%           apply control input l.

global GAMMA R P_WIND Nc
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K
global TERMINAL_STATE_INDEX
M =  size(map, 1);
N =  size(map, 2);
load('crash_P.mat');
G = zeros(K,5);
k_base_real = ComputeBaseStateIndex(stateSpace, map);
for k = 1:K/2
    % to exclude the condition that state i resides in a TREE
    m1 = stateSpace(2*k, 1);
    n1 = stateSpace(2*k, 2);
    % five different ctrl inputs
    for i = 1:5
        switch i
            case 1
                m0 = m1;
                n0 = n1+1; % m0 and n0 are middle state after the control action
            case 2
                m0 = m1;
                n0 = n1-1;
            case 3
                m0 = m1+1;
                n0 = n1;
            case 4
                m0 = m1-1;
                n0 = n1;
            case 5
                m0 = m1;
                n0 = n1;
        end
        
        % guarantee that the middle position is an inside-boundry not-TREE point
        if ((m0>=1 && m0<=M && n0>=1 && n0<=N) == 0) || map(m0,n0) == TREE
            G(2*k-1,i) = Inf;
            G(2*k,i) = Inf;
        % G(K, L) = 1 + 9*P(K, BASESTATE-NONPACKAGE, L)
        else
            G(2*k-1,i) = double(10*crash_P(2*k-1,k_base_real, i)+ myP(2*k-1,k_base_real, i)+...
                -crash_P(2*k-1,k_base_real, i)+(1 - myP(2*k-1,k_base_real, i)));
            G(2*k,i) = G(2*k-1,i);
        end
        
        if map(m1, n1) == DROP_OFF
            G(2*k,i) = 0;
        end
    end
end
end




















