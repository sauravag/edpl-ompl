%%%%%%%%%%%%%%%%%%%%%
% Post Processing FIRM Planner Log Outout
% Author: Saurav Agarwal
% Copyright: Saurav Agarwal 2015
%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;

% Read rollout data
M_rolloutcost = csvread('RolloutFIRMCostHistory.csv');
t_rolloutcost = M_rolloutcost(:,1);
val_rolloutcost = M_rolloutcost(:,2);

M_rolloutnodesreached = csvread('RolloutFIRMNodesReachedHistory.csv');
t_rolloutnodes = M_rolloutnodesreached(:,1);
val_rolloutnodes = M_rolloutnodesreached(:,2);

M_rolloutsp = csvread('RolloutFIRMSuccessProbabilityHistory.csv');
t_rolloutsp = M_rolloutsp(:,1);
val_rolloutsp = M_rolloutsp(:,2);

M_rolloutV = csvread('RolloutFIRMVelocityHistory.csv');
t_rolloutV = M_rolloutV(:,1);
val_rolloutV = M_rolloutV(:,2);

% Read standard FIRM data

M_firmcost = csvread('StandardFIRMCostHistory.csv');
t_firmcost = M_firmcost(:,1);
val_firmcost = M_firmcost(:,2);

M_firmnodesreached = csvread('StandardFIRMNodesReachedHistory.csv');
t_firmnodes = M_firmnodesreached(:,1);
val_firmnodes = M_firmnodesreached(:,2);

M_firmsp = csvread('StandardFIRMSuccessProbabilityHistory.csv');
t_firmsp = M_firmsp(:,1);
val_firmsp = M_firmsp(:,2);

M_firmV = csvread('StandardFIRMVelocityHistory.csv');
t_firmV = M_firmV(:,1);
val_firmV = M_firmV(:,2);



%%%%%%%%%%%%%%
% Plots
%%%%%%%%%%%%%%
figure(1)
plot(t_firmcost,val_firmcost,'b',t_rolloutcost,val_rolloutcost,'g');
legend('FIRM', 'FIRM with Rollout');
ylabel('Cost');
xlabel('Time');
saveas(1,'Results/CostComparison.jpg');

figure(2)
plot(t_firmnodes,val_firmnodes,'b',t_rolloutnodes,val_rolloutnodes,'g');
legend('FIRM', 'FIRM with Rollout');
ylabel('Number of Stabilizations');
xlabel('Time');
saveas(2,'Results/NodesReachedComparison.jpg');

figure(3)
plot(t_firmsp,val_firmsp,'b',t_rolloutsp,val_rolloutsp,'g');
legend('FIRM', 'FIRM with Rollout');
ylabel('Success Probability');
xlabel('Time');
saveas(3,'Results/SuccessProbComparison.jpg');

figure(4)
plot(t_firmV,val_firmV,'b',t_rolloutV,val_rolloutV,'g');
legend('FIRM', 'FIRM with Rollout');
ylabel('Velocity (m/s)');
ylim([-0.1 0.6]);
xlabel('Time');
saveas(4,'Results/VelocityComparison.jpg');








