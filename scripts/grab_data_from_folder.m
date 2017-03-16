function [firmData, rolloutData]  = grab_data_from_folder(folderpath)
% This function will get FIRM and rollout data from chosen folder
% it will return average terminal time, cost and nodes reached

firmData = struct('time',[],'cost',[],'nodesreached',[]);
rolloutData = struct('time',[],'cost',[],'nodesreached',[]);

baseDirectory = strcat(folderpath,'FIRM/');

% Get list of files in base folder
fileList = dir(baseDirectory);

% Get a logical vector that tells which is a directory.
dirFlags = [fileList.isdir];

% Extract only those that are directories.
subFolders = fileList(dirFlags);

Runs = [];

for k = 1 : length(subFolders)
    str = textscan(subFolders(k).name,'%s %*[.]');
    
    idx = regexp(str{1},'run');
    if ~isempty(idx{1})
        Runs = [Runs k];
    end
end

firmcost = struct('history',[]);
firmnodesreached = struct('history',[]);
dpsolvetime = struct('data',[]);
fcounter = 0;

avgFIRMCost = 0;
avgFIRMTime = 0;
avgFIRMNodesReached = 0;

% Running through folder
for i = Runs
    
    str = textscan(subFolders(i).name,'%s %*[.]');
    
    idx = regexp(str{1},'run');
    if ~isempty(idx{1})
        fcounter = fcounter +1;
        
        firmcost(fcounter).history = csvread(strcat(baseDirectory,subFolders(i).name,'/StandardFIRMCostHistory.csv'));
        firmnodesreached(fcounter).history = csvread(strcat(baseDirectory,subFolders(i).name,'/StandardFIRMNodesReachedHistory.csv'));
        
        avgFIRMCost =  avgFIRMCost + firmcost(fcounter).history(end,2);
        avgFIRMTime = avgFIRMTime + firmcost(fcounter).history(end,1);
        avgFIRMNodesReached = avgFIRMNodesReached + firmnodesreached(fcounter).history(end,2);
        
        % read DP solve time
        dpfid = fopen(strcat(baseDirectory,subFolders(i).name,'/DPSolveTime.txt'));
        
        tempData = [];
        
        while true
            
            % read line
            tline = fgetl(dpfid);
            
            % check if it has anything in it
            if ~ischar(tline);
                break;%end of file
            end
            
            % if it had something, lets see what it had
            % Scan the line upto the first space
            % we will identify what data type this is
            str = textscan(tline,'%s %d %*[ ]');
            
            tempData = [tempData, str{2}];
            
        end
        
        dpsolvetime(fcounter).data = double(tempData)/1000.0;
        
        fclose(dpfid);
        
    end
    
end

firmData.time = avgFIRMTime / length(Runs);
firmData.cost = avgFIRMCost / length(Runs);
firmData.nodesreached = ceil(avgFIRMNodesReached / length(Runs));

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze the Rollout Runs 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

baseDirectory = strcat(folderpath,'Rollout/');

% Get list of files in base folder
fileList = dir(baseDirectory);

% Get a logical vector that tells which is a directory.
dirFlags = [fileList.isdir];

% Extract only those that are directories.
subFolders = fileList(dirFlags);

Runs = [];

for k = 1 : length(subFolders)
    str = textscan(subFolders(k).name,'%s %*[.]');
    
    idx = regexp(str{1},'run');
    if ~isempty(idx{1})
        Runs = [Runs k];
    end
end

rolloutcost = struct('history',[]);
rolloutnodesreached = struct('history',[]);
rcounter = 0;

avgRolloutCost = 0;
avgRolloutTime = 0;
avgRolloutNodesReached = 0;

% Running through folder
for i = Runs
    str = textscan(subFolders(k).name,'%s %*[.]');
    
    idx = regexp(str{1},'run');
    if ~isempty(idx{1})
        rcounter = rcounter +1;
        rolloutcost(rcounter).history = csvread(strcat(baseDirectory,subFolders(i).name,'/RolloutFIRMCostHistory.csv'));        
        
        rolloutnodesreached(rcounter).history = csvread(strcat(baseDirectory,subFolders(i).name,'/RolloutFIRMNodesReachedHistory.csv'));
        
        avgRolloutCost =  avgRolloutCost + rolloutcost(rcounter).history(end,2);
        avgRolloutTime = avgRolloutTime + rolloutcost(rcounter).history(end,1);
        avgRolloutNodesReached = avgRolloutNodesReached + rolloutnodesreached(rcounter).history(end,2);
    end
end

rolloutData.time = avgRolloutTime / length(Runs);
rolloutData.cost = avgRolloutCost / length(Runs);
rolloutData.nodesreached = ceil(avgRolloutNodesReached / length(Runs));

end