function func_export_autocage_data_to_matfile(file_dir)

disp([file_dir,'...'])

%% load event data
Date            = ''; % no subfolder
event_file_name = 'EVENTS';
if exist([file_dir '\' Date event_file_name '.mat'], 'file') == 2
    load([file_dir '\' Date event_file_name '.mat']);
else
    event = importdata([file_dir '\' Date event_file_name '.txt'],' ');
    save([file_dir '\' Date event_file_name '.mat'],'event');
end

event_file_name = 'TRIALS';
if exist([file_dir '\' Date event_file_name '.mat'], 'file') == 2
    load([file_dir '\' Date event_file_name '.mat']);
else
    fid = fopen([file_dir '\' Date event_file_name '.txt']);
    textLine = fgets(fid);
    lineCounter = 1;
    while ischar(textLine)
        numbers = sscanf(textLine, '%f');
        trial{lineCounter} = numbers;
        textLine = fgets(fid);
        lineCounter = lineCounter + 1;
    end
    fclose(fid);
    % trial = importdata([file_dir '\' Date '\' event_file_name '.txt'],' ');
    save([file_dir '\' Date event_file_name '.mat'],'trial');
end

event_file_name = 'EVENTST';
if exist([file_dir '\' Date event_file_name '.mat'], 'file') == 2
    load([file_dir '\' Date event_file_name '.mat']);
else
    fid = fopen([file_dir '\' Date event_file_name '.txt']);
    textLine = fgets(fid);
    lineCounter = 1;
    while ischar(textLine)
        numbers = sscanf(textLine, '%f');
        trial_event{lineCounter} = numbers;
        textLine = fgets(fid);
        lineCounter = lineCounter + 1;
    end
    fclose(fid);
    % trial = importdata([file_dir '\' Date '\' event_file_name '.txt'],' ');
    save([file_dir '\' Date event_file_name '.mat'],'trial_event');
end




%% extract events
current_time = 0;
for i = 1:size(event,1)
    if event(i,2) == 1 % restart Arduino
        if event(i,1) > current_time
            current_time = event(i,1);
        end
    else
        event(i,1) = current_time + event(i,1)/1000;
    end
end

file_start_time = min([event(1,1) trial{1}(1)]);


start_datetime = datetime(trial{1}(1),'ConvertFrom','posixtime');
disp(string(start_datetime));

event_time_sec = event(:,1) - file_start_time;
event_time_sec(event_time_sec==0) = 1; % to avoid zero index
event_time_min = event_time_sec/60; % min
event_id       = event(:,2);
event_value    = event(:,3);


Switch_trigger_time             = event_time_min(event_id==7);
Fixation_time                   = event_time_min(event_id==8);
Fixation_value                  = event_value(event_id==8);
Fixation_release_timeup_time    = event_time_min(event_id==9);
Fixation_release_escape_time    = event_time_min(event_id==10);
Fixation_release_struggle_time  = event_time_min(event_id==12);
Fixation_release_time           = event_time_min(event_id==12 | event_id==10 | event_id==9);
Fixation_again_time             = event_time_min(event_id==11);
Fixation_all_time               = event_time_min(event_id==8 | event_id==11);
Fixation_release_timeup_value   = event_value(event_id==9)/1000;
Fixation_release_escape_value   = event_value(event_id==10)/1000;
Fixation_release_struggle_value = event_value(event_id==12)/1000;


%% trial data
trial_num = numel(trial);

current_trial_num = zeros(trial_num,1);
protocol = zeros(trial_num,1);
sub_protocol = zeros(trial_num,1);
trial_type = zeros(trial_num,1);
trial_outcome = zeros(trial_num,1);
trial_stim_index = zeros(trial_num,1);
trial_date = NaN(trial_num,1);
trial_FBpos = zeros(trial_num,1);
trial_LRpos = zeros(trial_num,1);

early_licks_count = NaN(trial_num,1);
is_early_lick_sample = zeros(trial_num,1);
is_early_lick_delay = zeros(trial_num,1);

for i = 1:trial_num
    cell_num = numel(trial{i});
    if cell_num <= 6 % old version
        current_trial_num(i) = trial{i}(1);
        protocol(i) = trial{i}(2);
        sub_protocol(i) = trial{i}(3);
        trial_type(i) = trial{i}(4);
        trial_outcome(i) = trial{i}(5);
        if trial_outcome(i) == 4
            trial_type(i) = 1;
            trial_outcome(i) = 1;
        elseif trial_outcome(i) == 5
            trial_type(i) = 0;
            trial_outcome(i) = 1;
        end
    else
        trial_date(i) = trial{i}(1);
        
        current_trial_num(i) = trial{i}(2);
        protocol(i) = trial{i}(3);
        sub_protocol(i) = trial{i}(4);
        trial_type(i) = trial{i}(5);
        trial_outcome(i) = trial{i}(6);
        if trial_outcome(i) == 4
            trial_type(i) = 1;
            trial_outcome(i) = 1;
        elseif trial_outcome(i) == 5
            trial_type(i) = 0;
            trial_outcome(i) = 1;
        end
        trial_stim_index(i) = trial{i}(7);
        states = trial{i}(8:end);
        if any(states>20) % compatable to old version
            trial_FBpos(i) = states(1);
            trial_LRpos(i) = states(2);
            states = states(3:end);
        end
        if protocol(i) >= 22
            early_licks_count(i) = sum(states == 2 | states == 4);
            is_early_lick_sample(i) = ismember(2,states);
            is_early_lick_delay(i) = ismember(4,states);
        end
    end
    
end

trial_time_sec = (trial_date-file_start_time);
trial_time_min = (trial_date-file_start_time)/60;

protocol_advance = diff(protocol);
protocol_advance_index = find(protocol_advance ~=0);
subprotocol_advance = diff(sub_protocol);
subprotocol_advance_index = find(subprotocol_advance ~=0);







%% saving data
variables_all = [
    '''start_datetime'',',...
    '''event_time_sec'',',...
    '''event_time_min'',',...
    '''event_id'',',...
    '''event_value'',',...
    '''event_time_sec'',',...
    '''Switch_trigger_time'',',...
    '''Fixation_time'',',...
    '''Fixation_value'',',...
    '''Fixation_release_timeup_time'',',...
    '''Fixation_release_escape_time'',',...
    '''Fixation_release_struggle_time'',',...
    '''Fixation_release_time'',',...
    '''Fixation_again_time'',',...
    '''Fixation_all_time'',',...
    '''Fixation_release_timeup_value'',',...
    '''Fixation_release_escape_value'',',...
    '''Fixation_release_struggle_value'',',...
    '''trial_num'',',...
    '''current_trial_num'',',...
    '''protocol'',',...
    '''sub_protocol'',',...
    '''trial_type'',',...    % 0-right; 1-left; 2-either
    '''trial_outcome'',',... % 0-ignore; 1-success; 2-error; 3-others  
    '''trial_stim_index'',',...  % 0-noStim; 1-sample; 2-delay
    '''trial_date'',',...    % trial time, unix time
    '''trial_time_sec'',',...
    '''trial_time_min'',',...
    '''trial_FBpos'',',...
    '''trial_LRpos'',',...
    '''early_licks_count'',',...
    '''is_early_lick_sample'',',...
    '''is_early_lick_delay''',...
];



eval(['save([file_dir, ''.mat''],',variables_all,')']);

