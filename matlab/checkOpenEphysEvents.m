function checkOpenEphysEvents(spikeFile, openEphysDir, spikeChannelName)

% sometimes the same digital input split into spike and open ephys results
% in different numbers of events in the two systems // given an open ephys
% recording folder, and a spike file, compares the numbers of events in all
% open ephys event channels to a specified event in spike file


% % temp
% spikeFile = 'E:\bonsaiVid\run.mat';
% openEphysDir = 'E:\ephys\ephys_2018-10-23_12-05-25';
% spikeChannelName = 'obsOn';

% initializations
addpath(fullfile(getenv('GITDIR'), 'analysis-tools'))
[channel, openEphysTimes, info] = load_open_ephys_data_faster(fullfile(openEphysDir, 'all_channels.events'));
spikeData = load(spikeFile, spikeChannelName);


spikeEventNum = sum(spikeData.(spikeChannelName).level);
for i = 0:7
    channelEventNum = length(openEphysTimes(logical(info.eventId) & channel==i));
    fprintf('%s: %i events in spike and %i events in openEphys channel %i \n', ...
        spikeChannelName, spikeEventNum, channelEventNum, i+1);
    if channelEventNum==spikeEventNum; disp('^^^CORRECT NUMBER OF EVENTS IN THIS CHANNEL^^^'); end
end
