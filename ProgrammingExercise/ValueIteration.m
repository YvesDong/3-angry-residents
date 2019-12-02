function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K 

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX 
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)



%% Initialization
J_opt = zeros(K,1);
u_opt_ind = zeros(K,1);

% maximum error between two iter
err = 1e-04; 

% Iterate until cost has converged
iter = 0;

%% value iteration

while (1)
    % Increase counter
    iter = iter + 1;
    disp(['iter=',num2str(iter,'%9.0f')]);
    %initialize cost-to-go Matrix 
    costToGo = zeros(K,6);
    % Update the value
    for i = 1:K     
        % add up cost-to-go for each input
        for q = 1:5 
            if G(i,q) ~= Inf
                for j = 1:K               
                    costToGo(i,q) = costToGo(i,q) + P(i,j,q)*J_opt(j);
                end
                costToGo(i,q) = costToGo(i,q)+G(i,q);
            else
                costToGo(i,q) = Inf;
            end           
        end
        %find the minimal entry from current costToGo matrix 
        % and the corresponding input
        [costToGo(i,6),u_opt_ind(i)] = min(costToGo(i,1:5));
    end 

    % compute the eroor and determine if to terminate       
    if max(costToGo(:,6)-J_opt) < err
        % update and break
        J_opt = costToGo(:,6);
        break
    else
        % update
        J_opt = costToGo(:,6);
    end
    
end

end