function P = ComputeTransitionProbabilities( stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map)
%   computes the transition probabilities between all states in the state
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
M =  size(map, 1);
N =  size(map, 2);
P = zeros(M*N*2,M*N*2,5);
crash_P = zeros(M*N*2,M*N*2,5);
p_wind_dir = 0.25 * P_WIND;
p_windless = 1 - P_WIND;
%% to find the shooters' location in the map
[x_shooter,y_shooter] = find(map==SHOOTER);
shooter = [x_shooter,y_shooter];

%% to find the base' location in the map
[x_base, y_base] = find(map==BASE);
base = [x_base, y_base];
k_base = (base(1)-1)*2*N + base(2)*2 -1; % the loc of base corresponds to k_base in stateSpace, whose 3rd state is 0, no package
%% start the loop for initial state 2k and 2k-1, which are of the same location
for k = 1:0.5*M*N*2
    % to exclude the condition that state i resides in a TREE
    m1 = idivide(k-1,int16(N)) + 1;
    n1 = k - (m1 - 1)*N;
    if map(m1,n1) == TREE
        continue
    % in state i the drone stays at an not-TREE point on the map
    else
        for i = 1:5        
            % five different ctrl inputs
            switch i
                case 1
                    m0 = m1;
                    n0 = n1+1; % m0 and n0 are middle state after the control action
                    index_m = 0;
                    index_n = 2; % the index-level change during state i -> j
                case 2
                    m0 = m1;
                    n0 = n1-1;
                    index_m = 0;
                    index_n = -2;
                case 3
                    m0 = m1+1;
                    n0 = n1;
                    index_m = 2*N;
                    index_n = 0;
                case 4
                    m0 = m1-1;
                    n0 = n1;
                    index_m = -2*N;
                    index_n = 0;
                case 5
                    m0 = m1;
                    n0 = n1;
                    index_m = 0;
                    index_n = 0;
            end
            indexCtrl = index_m + index_n;
            p = zeros(1,5); % a probability vector calculating the hover end up back in base
            
            % guarantee that the middle position is an inside-boundry not-TREE point
            if ((m0>=1 && m0<=M && n0>=1 && n0<=N) == 0) || map(m0,n0) == TREE
                continue
            else
                % next 6 possible positions on the map
                % j = (m0, n0+1, :), north in the second substep
                if n0 ~= N && map(m0, n0+1) ~= TREE
                    p_tri_miss = ComputeMissShotProb(shooter, m0, n0+1);
                    P(2*k, 2*k+indexCtrl+2,i) = p_wind_dir * p_tri_miss;
                    if map(m0, n0+1) == PICK_UP   % when the 3rd state para changes
                        P(2*k-1,2*k+indexCtrl+2,i) = p_wind_dir * p_tri_miss;
                    else
                        P(2*k-1,2*k-1+indexCtrl+2,i) = p_wind_dir * p_tri_miss;
                    end
                    p(1) = p_wind_dir * (1 - p_tri_miss);
                else
                    p(1) = p_wind_dir;
                end
                
                % j = (m0, n0-1, :), south
                if n0 ~= 1 && map(m0, n0-1) ~= TREE
                    p_tri_miss = ComputeMissShotProb(shooter, m0, n0-1);
                    P(2*k,2*k+indexCtrl-2,i) = p_wind_dir * p_tri_miss;
                    if map(m0, n0-1) == PICK_UP 
                        P(2*k-1,2*k+indexCtrl-2,i) = p_wind_dir * p_tri_miss;
                    else
                        P(2*k-1,2*k-1+indexCtrl-2,i) = p_wind_dir * p_tri_miss;
                    end
                    p(2) = p_wind_dir * (1 - p_tri_miss);
                else
                    p(2) = p_wind_dir;
                end
                
                % j = (m0+1, n0, :), east
                if m0 ~= M && map(m0+1, n0) ~= TREE
                    p_tri_miss = ComputeMissShotProb(shooter, m0+1, n0);
                    P(2*k,2*k+indexCtrl+2*N,i) = p_wind_dir * p_tri_miss;
                    if map(m0+1, n0) == PICK_UP 
                        P(2*k-1,2*k+indexCtrl+2*N,i) = p_wind_dir * p_tri_miss;
                    else
                        P(2*k-1,2*k-1+indexCtrl+2*N,i) = p_wind_dir * p_tri_miss;
                    end
                    p(3) = p_wind_dir * (1 - p_tri_miss);
                else
                    p(3) = p_wind_dir;
                end
                
                % j = (m0-1, n0, :), west
                if m0 ~= 1 && map(m0-1, n0) ~= TREE
                    p_tri_miss = ComputeMissShotProb(shooter, m0-1, n0);
                    P(2*k,2*k+indexCtrl-2*N,i) = p_wind_dir * p_tri_miss;
                    if map(m0-1, n0) == PICK_UP 
                        P(2*k-1,2*k+indexCtrl-2*N,i) = p_wind_dir * p_tri_miss;
                    else
                        P(2*k-1,2*k-1+indexCtrl-2*N,i) = p_wind_dir * p_tri_miss;
                    end
                    p(4) = p_wind_dir * (1 - p_tri_miss);
                else
                    p(4) = p_wind_dir;
                end
                
                % j = (m0, n0, :), stay, windless
                p_tri_miss = ComputeMissShotProb(shooter, m0, n0);
                P(2*k,2*k+indexCtrl,i) = p_windless * p_tri_miss;
                if map(m0,n0) == PICK_UP 
                    P(2*k-1,2*k+indexCtrl,i) = p_windless * p_tri_miss;
                else
                    P(2*k-1,2*k-1+indexCtrl,i) = p_windless * p_tri_miss;
                end
                p(5) = p_windless * (1 - p_tri_miss); 
                
                % j = (BASE_POSITION, 1), tree, out of boundry or shot; 
                % rev Nov.27 in this way, we include two end-up-base conditions:
                % crash back and non-crash back
                P(2*k-1, k_base, i) = P(2*k-1, k_base, i) + sum(p);
                P(2*k, k_base, i) = P(2*k, k_base, i) + sum(p);
                crash_P(2*k-1, k_base, i) = sum(p);
                crash_P(2*k, k_base, i) =sum(p);
            end
        end
    end
    
    % terminate the algorithm when hitting the dropoff with package
    if map(m1,n1) == DROP_OFF
        for q = 1:5
            for l = 1:M*N*2
                P(2*k, l, q) = 0;
            end
            P(2*k, 2*k, q) = 1;
        end
    end
    
end

% %% remove the TREE state out of the matrix, to have a 476*476*5 matrix
tree_index_k = [];
for m = 1 : M
    for n = 1 : N
        if map(m, n) == TREE
            tree_index_k = [tree_index_k, (m-1)*2*N+2*n-1, (m-1)*2*N+2*n];
        end
    end
end
P(tree_index_k, :, :) =[];
P(:, tree_index_k, :) =[];
crash_P(tree_index_k, :, :) =[];
crash_P(:, tree_index_k, :) =[];
save('crash_P.mat');
end
