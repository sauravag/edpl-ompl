%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze the data from varying graph size
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clc;
clear variables 
dbstop if error;
drawnow

save_flag = 1;

%% Grab all data
baseDirectory = '/Users/sauravagarwal/Dropbox/SLAP_Rollout_FIRM/DATA-Sims/Dec-2016/IncreasingGraphSize/';

% time step of sim
dt = 0.1;

% Get list of files in base folder
fileList = dir(baseDirectory);

% Get a logical vector that tells which is a directory.
dirFlags = [fileList.isdir];

% Extract only those that are directories.
subFolders = fileList(dirFlags);

foldersOfInterest = {};

counter = 0;
for k = 1 : length(subFolders)
    str = textscan(subFolders(k).name,'%s %*[.]');
    
    idx = regexp(str{1},'Nodes');
    if ~isempty(idx{1})
        counter = counter + 1;
        foldersOfInterest{counter} = subFolders(k).name;
    end
end

graphData = struct();

% arrays for plotting
graphSize = zeros(1,length(foldersOfInterest));
avgFirmCost = zeros(1,length(foldersOfInterest));
avgRolloutCost = zeros(1,length(foldersOfInterest));
avgFirmNodesReached = zeros(1,length(foldersOfInterest));
avgRolloutNodesReached = zeros(1,length(foldersOfInterest));
avgFirmTime = zeros(1,length(foldersOfInterest));
avgRolloutTime = zeros(1,length(foldersOfInterest));

for i = 1: length(foldersOfInterest)

    datastr =  textscan(foldersOfInterest{i},'%d %*[.]');

    graphSize(i) = datastr{1};

    [graphData(i).firmData, graphData(i).rolloutData] = grab_data_from_folder(strcat(baseDirectory,foldersOfInterest{i},'/'));

    avgFirmCost(i) = graphData(i).firmData.cost;
    avgRolloutCost(i) = graphData(i).rolloutData.cost;
    
    avgFirmNodesReached(i) = graphData(i).firmData.nodesreached;
    avgRolloutNodesReached(i) = graphData(i).rolloutData.nodesreached;

    avgFirmTime(i) =  graphData(i).firmData.time;
    avgRolloutTime(i) = graphData(i).rolloutData.time;

end

%% Plotting
[graphSize, ii] = sort(graphSize);
graphSize = graphSize(2:end);
avgFirmCost = avgFirmCost(ii(2:end));
avgRolloutCost = avgRolloutCost(ii(2:end));
avgFirmNodesReached = avgFirmNodesReached(ii(2:end));
avgRolloutNodesReached = avgRolloutNodesReached(ii(2:end));
avgFirmTime =  avgFirmTime(ii(2:end))*dt;
avgRolloutTime = avgRolloutTime(ii(2:end))*dt;

% Plot average terminal cost for different graph sizes
mfh1 = figure;
hold on;
plot(graphSize,avgFirmCost,'-k','Linewidth',2);
plot(graphSize,avgRolloutCost,'--b','Linewidth',2);
hold off;
title('Average Terminal Cost for Increasing Graph Nodes');
xlabel('Number of Nodes');
ylabel('Cost');
legend('FIRM','Rollout');
if save_flag
    set(mfh1,'Units','Inches');
    pos = get(mfh1,'Position');
    set(mfh1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(mfh1, 'sim-cost','-dpdf')
    close(mfh1);
end

% Average terminal nodes reached for different graph sizes
mfh2 = figure;
hold on;
plot(graphSize, avgFirmNodesReached,'-k','Linewidth',2);
plot(graphSize, avgRolloutNodesReached,'--b','Linewidth',2);
hold off;
title('Terminal Nodes Reached for Increasing Graph Nodes');
xlabel('Number of Nodes');
ylabel('Nodes Reached');
legend('FIRM','Rollout');
if save_flag
    set(mfh2,'Units','Inches');
    pos = get(mfh2,'Position');
    set(mfh2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(mfh2, 'sim-nodes-reached','-dpdf')
    close(mfh2);
end

% plot average terminal time
mfh3 = figure;
hold on;
plot(graphSize, avgFirmTime,'-k','Linewidth',2);
plot(graphSize, avgRolloutTime,'--b','Linewidth',2);
hold off;
title('Average Terminal Time for Increasing Graph Nodes');
xlabel('Number of Nodes');
ylabel('Time (s)');
legend('FIRM','Rollout');
if save_flag
    set(mfh3,'Units','Inches');
    pos = get(mfh3,'Position');
    set(mfh3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(mfh3, 'sim-time','-dpdf')
    close(mfh3);
end