clear
%%
data_folder = '.\Data\';
Mouse_ID    = 'YH100'; % folder

%% load event data
Date            = ''; % no subfolder
event_file_name = 'EVENTS';
if exist([data_folder Mouse_ID '\' Date event_file_name '.mat'], 'file') == 2
    load([data_folder Mouse_ID '\' Date event_file_name '.mat']);
else
    event = importdata([data_folder Mouse_ID '\' Date event_file_name '.txt'],' ');
    save([data_folder Mouse_ID '\' Date event_file_name '.mat'],'event');
end

event_file_name = 'TRIALS';
if exist([data_folder Mouse_ID '\' Date event_file_name '.mat'], 'file') == 2
    load([data_folder Mouse_ID '\' Date event_file_name '.mat']);
else
    fid = fopen([data_folder Mouse_ID '\' Date event_file_name '.txt']);
    textLine = fgets(fid);
    lineCounter = 1;
    while ischar(textLine)
        numbers = sscanf(textLine, '%f');
        trial{lineCounter} = numbers;
        textLine = fgets(fid);
        lineCounter = lineCounter + 1;
    end
    fclose(fid);
    % trial = importdata([data_folder Mouse_ID '\' Date '\' event_file_name '.txt'],' ');
    save([data_folder Mouse_ID '\' Date event_file_name '.mat'],'trial');
end

event_file_name = 'EVENTST';
if exist([data_folder Mouse_ID '\' Date event_file_name '.mat'], 'file') == 2
    load([data_folder Mouse_ID '\' Date event_file_name '.mat']);
else
    fid = fopen([data_folder Mouse_ID '\' Date event_file_name '.txt']);
    textLine = fgets(fid);
    lineCounter = 1;
    while ischar(textLine)
        numbers = sscanf(textLine, '%f');
        trial_event{lineCounter} = numbers;
        textLine = fgets(fid);
        lineCounter = lineCounter + 1;
    end
    fclose(fid);
    % trial = importdata([data_folder Mouse_ID '\' Date '\' event_file_name '.txt'],' ');
    save([data_folder Mouse_ID '\' Date event_file_name '.mat'],'trial_event');
end

%% extract events
event(:,1) = event(:,1)/1000;       % convert timestamp unit to sec
current_time = 0;
for i = 1:size(event,1)
    event(i,1) = current_time + event(i,1);
    
    if event(i,2) == 1 % restart Arduino
        if event(i,1) > current_time
            current_time = event(i,1);
        end
    end
end
start_datetime = datetime(trial{1}(1),'ConvertFrom','posixtime');% event(1,1) trial{1}(1)
disp(string(start_datetime));

event_time_sec = round(event(:,1) - event(1,1));
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

%% plot event data and save figure
figure, % ('Position', get(0, 'Screensize'))

% Head Fixation
is_fixation = 1.2* ones(event_time_sec(end),1); %  resolution: sec
for i = 1:length(Fixation_time)
    if i < length(Fixation_time)
        release_time = Fixation_release_time(Fixation_release_time >= Fixation_time(i) & Fixation_release_time<Fixation_time(i+1));
        if ~isempty(release_time)
            is_fixation(round(60*Fixation_time(i)):round(60*release_time(end))) = 4.3;
        end
    else
        if Fixation_release_time(end) >= Fixation_time(end)
            is_fixation(round(60*Fixation_time(end)):round(60*Fixation_release_time(end))) = 4.3;
        end
    end
end
subplot(2,1,1,'FontSize',16), hold on,
h1 = plot(Switch_trigger_time, 4*ones(size(Switch_trigger_time)), 'b^');
h2 = plot(Fixation_time, 3*ones(size(Fixation_time)), 'bv');
h3 = plot(Fixation_release_timeup_time, 2*ones(size(Fixation_release_timeup_time)), 'g*');
h4 = plot(Fixation_release_escape_time, 2*ones(size(Fixation_release_escape_time)), 'r*');
h5 = plot(Fixation_release_struggle_time, 2*ones(size(Fixation_release_struggle_time)), 'b*');
h6 = plot(Fixation_again_time, 3*ones(size(Fixation_again_time)), 'gv');
% plot((1:event_time_sec(end))/60,is_fixation,'c');

xlim([0,event_time_min(end)]); ylim([0 6]);
title(['Head Fixation', ' (num: ', num2str(numel(Fixation_time)), ')']);
set(gca,'YTick',[2 3 4],'YTickLabel',{'Release','Fixation','Trigger'});

% fixation time
subplot(2,1,2,'FontSize',16), hold on,
h3 = stem(Fixation_release_struggle_time, Fixation_release_struggle_value,'b');
h1 = stem(Fixation_release_timeup_time, Fixation_release_timeup_value,'g');
% h2 = stem(Fixation_release_escape_time, Fixation_release_escape_value,'r');
xlim([0,event_time_min(end)]); ylabel('Duration (s)');
ylim([0 1.1*max(Fixation_release_timeup_value)]);

minute_to_light_on = round(60*(18.5-(hour(start_datetime)+minute(start_datetime)/60)));
if minute_to_light_on > 0
    h = fill([0 minute_to_light_on minute_to_light_on 0],[0 0 70 70],[.5 .5 .5]);
    set(h,'facealpha',.5, 'EdgeColor','none');
    fill_start = minute_to_light_on+720:1440:event_time_min(end);
    for i = 1:numel(fill_start)
        if fill_start(i)+720 < event_time_min(end)
            h = fill([fill_start(i) fill_start(i)+720 fill_start(i)+720 fill_start(i)],[0 0 70 70],[.5 .5 .5]);
            set(h,'facealpha',.5, 'EdgeColor','none');
        else
            h = fill([fill_start(i) event_time_min(end) event_time_min(end) fill_start(i)],[0 0 70 70],[.5 .5 .5]);
            set(h,'facealpha',.5, 'EdgeColor','none');
        end
    end
else
    fill_start = minute_to_light_on:1440:event_time_min(end);
    for i = 1:numel(fill_start)
        if fill_start(i)+720 < event_time_min(end)
            h = fill([fill_start(i) fill_start(i)+720 fill_start(i)+720 fill_start(i)],[0 0 70 70],[.5 .5 .5]);
            set(h,'facealpha',.5, 'EdgeColor','none');
        else
            h = fill([fill_start(i) event_time_min(end) event_time_min(end) fill_start(i)],[0 0 70 70],[.5 .5 .5]);
            set(h,'facealpha',.5, 'EdgeColor','none');
        end
    end
end

fixation_interval = zeros(numel(Fixation_time),1);
if ~isempty(Fixation_time)
    for i = 1:numel(Fixation_time)-1
        release_time = Fixation_release_time(Fixation_release_time >= Fixation_time(i) & Fixation_release_time<=Fixation_time(i+1));
        if ~isempty(release_time)
            fixation_interval(i) = release_time(end)-Fixation_time(i);
        end
    end
    if Fixation_release_time(end) >= Fixation_time(end)
        fixation_interval(end) = Fixation_release_time(end)-Fixation_time(end);
    end
end
fixation_interval(fixation_interval>60) = []; % artifact
% fixation_interval(fixation_interval<2/60) = [];
avg_interval = mean(fixation_interval)*60;
total_interval = sum(fixation_interval);

num_days = numel(fill_start)-1;
night_day_fix = zeros(num_days,2);
fixation_num_days = zeros(num_days,1);
avg_fixation_duration_days = zeros(num_days,1);
total_duration_days = zeros(num_days,1);
struggle_rate = zeros(num_days,1);
for i = 1:num_days
    night_day_fix(i,1) = numel(find(Fixation_release_time>fill_start(i) & Fixation_release_time<fill_start(i)+720));
    night_day_fix(i,2) = numel(find(Fixation_release_time>fill_start(i)+720 & Fixation_release_time<fill_start(i)+1440));
    fixation_num_days(i) = numel(find(Fixation_release_time>fill_start(i) & Fixation_release_time<fill_start(i)+1440));
    total_duration_days(i) = sum(Fixation_release_timeup_value(Fixation_release_timeup_time>fill_start(i) & Fixation_release_timeup_time<fill_start(i)+1440)) + ...
        sum(Fixation_release_escape_value(Fixation_release_escape_time>fill_start(i) & Fixation_release_escape_time<fill_start(i)+1440)) + ...
        sum(Fixation_release_struggle_value(Fixation_release_struggle_time>fill_start(i) & Fixation_release_struggle_time<fill_start(i)+1440));
    % avg_fixation_duration_days(i) = total_duration_days(i)/fixation_num_days(i);
    avg_fixation_duration_days(i) = 60*mean(fixation_interval(Fixation_time>fill_start(i) & Fixation_time<fill_start(i)+1440));
    struggle_rate(i) = numel(find(Fixation_release_struggle_time>fill_start(i) & Fixation_release_struggle_time<fill_start(i)+1440))/fixation_num_days(i);
end
night_ratio = night_day_fix(:,1)./(night_day_fix(:,1)+night_day_fix(:,2));


range_bin = 0:60:1440;
bin_num = zeros(1,numel(range_bin)-1);
for i_day = 1:num_days
    for i = 1:numel(range_bin)-1
        bin_num(i) = bin_num(i)+numel(find(Fixation_release_time>fill_start(i_day)+range_bin(i) & Fixation_release_time<fill_start(i_day)+range_bin(i+1)));
    end
end
bin_num = bin_num/numel(i_day);

% Fixation_release_time = Fixation_release_time - 17*60;
% Fixation_release_time(Fixation_release_time < 0) = [];
% Fixation_release_time = rem(Fixation_release_time,1440);
% range_bin = 0:120:1440;
% bin_num = zeros(1,numel(range_bin)-1);
% for i = 1:numel(range_bin)-1
%     bin_num(i) = sum(Fixation_release_time>range_bin(i) & Fixation_release_time<range_bin(i+1));
% end
% figure, subplot(2,2,1),bar(bin_num/17);

title(['Head Fixation Duration', ' (avg: ', num2str(avg_interval), ' sec, total: ', num2str(total_interval), ' min, ' num2str(total_interval/numel(fill_start)), 'min per day, ', num2str(numel(fill_start)), ' days)']);
xlabel(['Time (min)', ' Start at:', char(start_datetime)]);
legend([h1,h3],{'timeup-release','self-release'});
% legend([h1,h2,h3],{'timeup-release','escape','self-release'});

%% trial data
figure, subplot(3,1,1), hold on,

trial_num = numel(trial);
% % data in Trials.txt
    % now.unixtime
    % S.currentTrialNum
    % S.ProtocolType: 10-fixation; 21-sample; 22-delay; 23-optostim
    % S.ProtocolHistoryIndex: '0: Free licking -> Headfixation 25 sec;',...'1-9: Sample Protocol',...'10-18: Delay Protocol (18: final)',...'19: Start to Optical Stimulation'
    % S.TrialType: 0-right; 1-left; 2-either
    % S.Trial_Outcome: 0-ignore; 1-success; 2-error; 3-others
    % trial_stim_index: 0-noStim; 1-sample; 2-delay
    % state_visited: 0,1,2,...
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
    
    %     switch trial_outcome(i)
    %         case 0
    %             h1 = plot(current_trial_num(i), trial_type(i), 'go', 'MarkerSize', 5);
    %         case 1
    %             h2 = plot(current_trial_num(i), trial_type(i), 'g.', 'MarkerSize', 15);
    %         case 2
    %             h3 = plot(current_trial_num(i), trial_type(i), 'r.', 'MarkerSize', 15);
    %         case 3
    %             h4 = plot(current_trial_num(i), trial_type(i), 'ro', 'MarkerSize', 5);
    %     end
end
% xlim([current_trial_num(1)-10 current_trial_num(end)+10]);ylim([-1.5 2.5]);
protocol_advance = diff(protocol);
protocol_advance_index = find(protocol_advance ~=0);
subprotocol_advance = diff(sub_protocol);
subprotocol_advance_index = find(subprotocol_advance ~=0);
% if ~isempty(protocol_advance_index)
%     for i = 1:numel(protocol_advance_index)
%         plot([current_trial_num(protocol_advance_index(i)) current_trial_num(protocol_advance_index(i))],[-1.5 2.5],'r-','LineWidth', 2);
%     end
% end
% if ~isempty(subprotocol_advance_index)
%     plot([current_trial_num(subprotocol_advance_index) current_trial_num(subprotocol_advance_index)],[-1.5 2.5],'r--');
% end
%
% xlabel('Trials');
% set(gca,'YTick', [0 1], 'YTickLabel', {'Right','Left'});

plot(trial_FBpos,'b');
plot(trial_LRpos,'r');
xlim([current_trial_num(1)-10 current_trial_num(end)+10]); ylim([-1 256]);
title('Motor Advance (blue-FB,red-LR)');

%%%%%%%%%%%%%%%%%%%%
subplot(3,1,2), hold on,
hit_iSession = (trial_outcome==1);
hit_iSession = double(hit_iSession);
hit_iSession(trial_outcome==0) = NaN;

hit_iSession_left = trial_type==1 & trial_outcome==1;
hit_iSession_left = double(hit_iSession_left);
hit_iSession_left(trial_type==0) = NaN;
hit_iSession_left(trial_outcome==0) = NaN;

hit_iSession_right = trial_type==0 & trial_outcome==1;
hit_iSession_right = double(hit_iSession_right);
hit_iSession_right(trial_type==1) = NaN;
hit_iSession_right(trial_outcome==0) = NaN;

avg_size = 50;
h2 = plot(current_trial_num,movmean(hit_iSession_left,1.5*avg_size,'omitnan'),'color',[1 0.7 0.7]);
h3 = plot(current_trial_num,movmean(hit_iSession_right,1.5*avg_size,'omitnan'),'color',[0.7 0.7 1]);
h1 = plot(current_trial_num,movmean(hit_iSession,avg_size,'omitnan'),'k','LineWidth',1);

perff = movmean(hit_iSession,avg_size,'omitnan');
YH89_lc = perff(2160:end);

title(['Performance (',num2str(avg_size), ' trails avg.)']);
if ~isempty(subprotocol_advance_index)
    plot([current_trial_num(subprotocol_advance_index) current_trial_num(subprotocol_advance_index)],[-1.5 2.5],'r--');
end
if ~isempty(protocol_advance_index)
    for i = 1:numel(protocol_advance_index)
        plot([current_trial_num(protocol_advance_index(i)) current_trial_num(protocol_advance_index(i))],[-1.5 2.5],'r-','LineWidth', 2);
    end
end
plot(current_trial_num,0.7*ones(size(current_trial_num,1),1), 'g');
xlim([current_trial_num(1)-10 current_trial_num(end)+10]);ylim([0 1]); xlabel('Trials'); ylabel('Performance');

%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3,1,3), hold on
if ~isempty(subprotocol_advance_index)
    plot([current_trial_num(subprotocol_advance_index) current_trial_num(subprotocol_advance_index)],[0 20],'r--');
end
if ~isempty(protocol_advance_index)
    for i = 1:numel(protocol_advance_index)
        plot([current_trial_num(protocol_advance_index(i)) current_trial_num(protocol_advance_index(i))],[0 20],'r-','LineWidth', 2);
    end
end
% avg_size = 20;
is_early_lick = logical(is_early_lick_sample) | logical(is_early_lick_delay);% is_early_lick_delay
is_early_lick = double(is_early_lick);
is_early_lick(trial_outcome==0) = NaN;
% perf_tmp = movmean(double(early_licks_count),avg_size,'omitnan')
% ylim([0 20]); ylabel(['EarlyLicks per trial (avg. ',num2str(avg_size),')']);

ylim([0 1]); ylabel(['EarlyLick Rate (, avg. ',num2str(avg_size),')']);

plot(current_trial_num, movmean(is_early_lick,avg_size,'omitnan'));
xlim([current_trial_num(1)-10 current_trial_num(end)+10]);xlabel('Trials');



%% meta-info
figure,
subplot(2,3,1),hold on,
subprotocol_num = diff([0; subprotocol_advance_index; current_trial_num(end)]);
if numel(subprotocol_advance_index) >=18
    protocol_num = diff([0; protocol_advance_index; subprotocol_advance_index(end); current_trial_num(end)]);
else
    protocol_num = diff([0; protocol_advance_index; current_trial_num(end)]);
end
bar(subprotocol_num); xlabel('SubProtocols'); ylabel('Trial Number');
title(num2str(protocol_num'));

% subplot(2,3,3),hold on,
% if numel(subprotocol_advance_index) >=18
%     new_protocol_advance_index = [protocol_advance_index; subprotocol_advance_index(end); current_trial_num(end)];
% else
%     new_protocol_advance_index = [protocol_advance_index; current_trial_num(end)];
% end
% bar((trial_date(new_protocol_advance_index)-event(1,1))/(24*60*60));
% title('Protocol-Days'); xlabel('Protocols');ylabel('Days');

subplot(2,3,3), hold on, % reaction time distribution
for opto_trial_num = 1:numel(trial)
    if trial{opto_trial_num}(4) == 19
        break;
    end
end

rt = NaN(numel(trial_event)-opto_trial_num+1,1);
for i = opto_trial_num:numel(trial_event)
    if trial_outcome(i) == 1  % && trial_type(i) == 0
        event_tmp = trial_event{i};
        num = event_tmp(2); % total event num
        timing_39 = []; timing_lick = [];
        for j = 1:num
            if event_tmp(3+(j-1)*2) == 39
                timing_39 = [timing_39 event_tmp(3+(j-1)*2+1)];
            elseif event_tmp(3+(j-1)*2) == 0 || event_tmp(3+(j-1)*2) == 2
                timing_lick = [timing_lick event_tmp(3+(j-1)*2+1)];
            end
        end
        for j = 2:numel(timing_39)
            if timing_39(j)-timing_39(j-1) == 1000
                go_cue = timing_39(j-1);
                break;
            end
        end
        timing_dif = timing_lick - go_cue;
        timing_dif = timing_dif(timing_dif>0);
        if ~isempty(timing_dif)
            rt(i-opto_trial_num+1) = 0.1*timing_dif(1);
        end
    end
end
h = histogram(rt,0:1000); xlabel('RT (ms)'); ylabel('Counts');
title([Mouse_ID ' - RT distribution' ' | <50ms: ' num2str(sum(h.BinCounts(1:50)/numel(rt)))]);


subplot(2,3,2),hold on,
plot(night_ratio,'Marker', 'o');
xlabel('Days');ylabel('Percetage of Trials in Dark Cycle');

subplot(2,3,4),hold on,
bar(bin_num/numel(Fixation_time));
set(gca,'XTick', [1 6 12 18 24], 'XTickLabel', {'1','6','12','18','24'});
xlabel('Hours'); ylabel('Percentage');

subplot(2,3,5),hold on, % inter-fixation interval (ifi)
ifi = zeros(numel(Fixation_time),1);
if ~isempty(Fixation_time)
    for i = 2:numel(Fixation_time)
        release_time = Fixation_release_time(Fixation_release_time < Fixation_time(i) & Fixation_release_time > Fixation_time(i-1));
        if ~isempty(release_time)
            ifi(i) = Fixation_time(i)-release_time(end);
        end
    end
end
ifi(1) = [];
% diff_fix = diff(Fixation_release_time);
% diff_fix(diff_fix>2)=2;
% hist(diff_fix,0:0.05:2);
% ifi(ifi>2)=2;
histogram(ifi,0:0.05:2);
xlim([0 2]); xlabel('Inter-fixation interval (min)'); ylabel('Counts');
%%
%%%%%%%%%%%%% licks
subplot(2,3,6),hold on, % Fig. 3B YH121, subplot(2,2,4)
plot_trial = 100;
plot_range = numel(trial_event)-plot_trial+1:numel(trial_event);
for i = plot_range
    event_tmp = trial_event{i};
    num = event_tmp(2); % total event num
    lick_left = []; lick_right = []; time_up = [];
    for j = 1:num
        if event_tmp(3+(j-1)*2) == 0
            lick_left = [lick_left event_tmp(3+(j-1)*2+1)];
        elseif event_tmp(3+(j-1)*2) == 2
            lick_right = [lick_right event_tmp(3+(j-1)*2+1)];
        elseif event_tmp(3+(j-1)*2) == 39 && (event_tmp(3+(j-1)*2))
            time_up = [time_up event_tmp(3+(j-1)*2+1)];
        end
    end
    if ~isempty(lick_left)
        h1 = plot(lick_left/10000,i-plot_range(1)+1,'b.','MarkerSize',15);
    end
    if ~isempty(lick_right)
        h2 = plot(lick_right/10000,i-plot_range(1)+1,'r.','MarkerSize',15);
    end
    %     if ~isempty(time_up)
    %         h3 = plot(time_up/10000,i,'g*');
    %     end
end
line([0.1 0.1],[1 plot_trial]);
line([1.3 1.3],[1 plot_trial]);
line([2.6 2.6],[1 plot_trial]);
xlim([0 4]); xlabel('Time (s)'); ylabel('Trial Number');
% legend([h1(1) h2(1) h3(1)],'Lick Left','Lick Right','Time Up');
legend([h1(1) h2(1)],'Lick Left','Lick Right');