function [ J_opt, u_opt_ind ] = LinearProgramming(myP, myG)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)


%% Linear programming
% solve the linear programming problem maximize the sum of V vector subject
% to the problem A*V<=b, where V = [V(1),.....,V(K)]

% Compute the matrix A & b

% initialization 
A = zeros(K*5,K);
b = zeros(1,K*5);
for i = 1:K
    for j = 1:5
            % set the values of the remaining states based on ctrl inputs
            A(5*(i-1)+j,:) = - myP(i,:,j);
            b(1,5*(i-1)+j) = myG(i,j);
            % set the value of the initial state to 1
            A(5*(i-1)+j,i) = A(5*(i-1)+j,i)+1;
    end
end
% remove the equation where the state costs are inf and the start state is 
% terminal state
infIndex =  [];
for b_index = 1:5*K
    if b(1,b_index) == inf || b_index == 5*TERMINAL_STATE_INDEX
       infIndex = [infIndex,b_index];
    end
end
b(infIndex) = [];
A(infIndex,:) = [];
% the cost of terminal state should now be considered in the lp problem
A(:,TERMINAL_STATE_INDEX) = [];

% represent the sum of all the elements in V as a vector,so that f*V =sum 
% maximization problem, the factors*-1
f =  -ones(1,K-1);
% slove the LP problem to get the value of the optimal costs
% the output of 'linprog' contains the optiaml solution and the minimized 
% target function value, here we only need the optimal solution V(i)
[J_opt, minusMaxSumJ] = linprog(f,A,b);
% find the optimal control input for each state, exclude the terminal state
u_opt_ind = zeros(K-1,1);
% initialization 
myG(TERMINAL_STATE_INDEX,:) = [];
myP(TERMINAL_STATE_INDEX,:,:) = [];
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1);0;J_opt(TERMINAL_STATE_INDEX:K-1)];

for k = 1:K-1
    % compute all the costs based on the current state
    tempV = myG(k,:)+ (squeeze(myP(k,:,:))'*J_opt)';
    % find the control input to minize the costs
    u_opt_ind(k) = find(tempV == min(tempV),1);
end
u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1);HOVER;u_opt_ind(TERMINAL_STATE_INDEX:K-1)];
end

