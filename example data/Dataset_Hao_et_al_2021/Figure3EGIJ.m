Mouse_ID    = { 'YH77'      'YH80'      'YH81'      'YH85'    'YH86' ...
                'YH87'      'YH88'      'YH82'      'YH94'      'YH96' ...
                'YH98'      'YH99'      'YH100'     'YH101'     'YH102' ...
                'YH104'     'YH106'     'YH108'     'YH109'     'YH112' ...
                'YH113'     'YH115'     'YH117'     'YH119'     'YH123' ...
                'YH120'     'YH121'     'YH124'     'YH110'     'YH126' ...
                'YH128'     'YH131'}; % 32/37
trial_range = {2216:14267	3582:33190	2201:16890	4074:21160  1840:9623 ...
               1660:14160   3190:16490  1652:16610  1750:23520  2068:19980 ...
               1900:13410   2046:9092   2312:12830	2000:12800	2097:13290 ...
               2309:14990   1896:31900  1957:42400  2031:30200  2025:19480 ...
               1707:15610   2211:20970  2361:21900  1704:20800  1664:13140 ...
               1921:16450   1891:11340  1300:27190  1792:20770  1568:28610 ...
               1762:11720   1154:20280}; % only protocol 3+
           
figure, 


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur3E %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,1),hold on,
load autocage_trial.mat % hc_trial
avg_size = 500;
for i_mice = 1:numel(hc_trial)
    tmp = movmean(hc_trial(i_mice).hit_all,avg_size,'omitnan');
    trial_range_i_mice = trial_range{i_mice};
	plot(tmp(trial_range_i_mice(1):trial_range_i_mice(end)),'LineWidth',0.5);
end
ylim([0.3 1]); xlim([0 35000]);


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur3G %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,2),hold on,
load manual_rig_trial.mat % meta_mice
for i_mice = 1:length(meta_mice)
    tmp = movmean((meta_mice{i_mice}.data.R_hit_allSession|meta_mice{i_mice}.data.L_hit_allSession),avg_size,'omitnan');
	plot(tmp,'LineWidth',0.5);
end
ylim([0.3 1]); xlim([0 35000]);



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur3I %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i_mice = 1:numel(hc_trial)
    % hc_total_days(i_mice) = (hc_trial(i_mice).trial_date(end)-hc_trial(i_mice).trial_date(1))/3600/60;
    hc_total_days(i_mice) = hc_trial(i_mice).num_days;
    hc_trial_nums(i_mice) = numel(hc_trial(i_mice).hit_all);
end
hc_trial_per_day = hc_trial_nums./hc_total_days;
subplot(4,8,18), hold on,
plot(1+(0.6*rand(numel(hc_trial),1)-0.3),hc_trial_per_day,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);

for i_mice = 1:length(meta_mice)
    rig_trial_per_day(i_mice) = mean(meta_mice{i_mice}.session_nTrials);
end
plot(2+(0.6*rand(numel(rig_trial_per_day),1)-0.3),rig_trial_per_day,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
barwitherr([std(hc_trial_per_day) std(rig_trial_per_day)], [mean(hc_trial_per_day) mean(rig_trial_per_day)],'edgecolor','none','linewidth',2,'facecolor','k');% Plot with errorbars
xlim([0.5 2.5]);  ylabel('Trial number'); ylim([0 1200]);
p1 = ranksum(hc_trial_per_day,rig_trial_per_day)
set(gca, 'xtick',[1 2],'xticklabel',{'Homecage','Manual rig'});



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur3J %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k = 1;
for i_mice = 1:length(meta_mice)
    if meta_mice{i_mice}.isP70 == 1
        rig_learning_days(k) = meta_mice{i_mice}.p70Session;
        rig_learning_trials(k) = meta_mice{i_mice}.p70Trial;
        k = k + 1;
    end
end
k = 1;
for i_mice = 1:numel(hc_trial)
    perf_tmp = movmean(hc_trial(i_mice).hit_all,100,'omitnan');
    com_tmp = perf_tmp>0.7 & hc_trial(i_mice).sub_protocol>17;
    hc_learning_trials(k) = find(com_tmp,1);
    hc_learning_days(k) = (hc_trial(i_mice).trial_date(hc_learning_trials(k))-hc_trial(i_mice).trial_date(1))/(3600*24);
    k = k + 1;
end

subplot(4,8,19), hold on, % learning days
plot(1+(0.6*rand(numel(hc_learning_days),1)-0.3),hc_learning_days,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
plot(2+(0.6*rand(numel(rig_learning_days),1)-0.3),rig_learning_days,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
h = barwitherr([std(hc_learning_days) std(rig_learning_days)], [mean(hc_learning_days) mean(rig_learning_days)],'edgecolor','none','linewidth',2,'facecolor','k');% Plot with errorbars
xlim([0.5 2.5]);  ylabel('Trial number'); ylim([0 100]);
p2 = ranksum(hc_learning_days, rig_learning_days)
set(gca, 'xtick',[1 2],'xticklabel',{'Homecage','Manual rig'});

subplot(4,8,20), hold on, % learning trials
plot(1+(0.6*rand(numel(hc_learning_trials),1)-0.3),hc_learning_trials,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
plot(2+(0.6*rand(numel(rig_learning_trials),1)-0.3),rig_learning_trials,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
barwitherr([std(hc_learning_trials) std(rig_learning_trials)], [mean(hc_learning_trials) mean(rig_learning_trials)],'edgecolor','none','linewidth',2,'facecolor','k');% Plot with errorbars
xlim([0.5 2.5]);  ylabel('Trial number'); ylim([0 30000]);
p3 = ranksum(hc_learning_trials, rig_learning_trials)
set(gca, 'xtick',[1 2],'xticklabel',{'Homecage','Manual rig'});