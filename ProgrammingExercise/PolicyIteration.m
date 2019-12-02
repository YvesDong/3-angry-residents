function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)


%% Policy Iter

%initialize a proper policy: All hover
u_opt_ind = 5*ones(K,1);
u_opt_ind(TERMINAL_STATE_INDEX) = 5;
%initialize cost-to-go matrix
J_opt = zeros(K,1);
J_opt(TERMINAL_STATE_INDEX) = 0;
%intitiaze G and P matrix in stage 1 and 2
P_stage_one = zeros(K,K);


% Iterate until cost-to-go stat stationary
iter = 0;

% start iteration
while (1)
    iter = iter + 1;
    disp(['iter=',num2str(iter,'%9.0f')]);
    
    % ------STAGE 1------  
    % define Bellman equation and evaluate
    G_stage_one = zeros(K,1);
    for i = 1:K
        G_stage_one(i) = G(i,u_opt_ind(i));
        for j = 1:K
            P_stage_one(i,j) = P(i,j,u_opt_ind(i));
        end             
    end
    
    % save J_opt from last iter
    J_opt_last = J_opt;
    
    % solve J = G + PJ
    %det_inv = det( eye(K-1)-P_stage_one([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K],...
    %                           [1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K]) );
    %disp(['detinv=',num2str(det_inv,'%9.0f')]);
    J_opt([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K])=... 
    ( inv(eye(K-1)-P_stage_one([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K],...
                               [1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K])))...
                               * G_stage_one([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K]);
      
    % check if J_opt changes
    if  isequal(J_opt_last,J_opt)
        break
    end
    
    
    % ------STAGE 2------ 
    % update the optimal policy for each state i  
    % for each state i ,compute cost for each input 
    G_stage_two=zeros(K,5);
    for u=1:5
        for i = 1:K
            if G(i,u) ~= Inf
                G_stage_two(i,u) = G_stage_two(i,u) + G(i,u);
                for j=1:K
                    G_stage_two(i,u) = G_stage_two(i,u)+P(i,j,u)*J_opt(j);
                end
            else
                G_stage_two(i,u) = Inf;
            end
        end
    end
    
    %find the min Cost-to-go and it's corresponding input
    for i = [1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K]
        [~,u_opt_ind(i)] = min(G_stage_two(i,:));
    end
    

    
    
end



end

