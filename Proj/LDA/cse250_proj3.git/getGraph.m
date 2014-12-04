% GET GRAPH FOR RESULTS
global THETA;
global EPOCH;

addpath('./fig');
NAME='3';

figure;
%% t3
plot(THETA(1,:),THETA(2,:),'.');

%% t4
% scatter3(THETA(1,:),THETA(2,:),THETA(3,:),'.','LineWidth',3);

%% t5
% scatter3(THETA(1,:),THETA(2,:),THETA(3,:),[],THETA(4,:),'.','LineWidth',3);


title(['CLUSTERS WHEN #TOPIC=' NAME]);
print('-deps',['./fig/t' NAME]);