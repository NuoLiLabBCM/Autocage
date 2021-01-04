% for Figure 5
clear
close all;
load reverse_mice.dat % reverse_mice
%%
Mouse_ID = {
    'YH160';...
    'YH148';...
    'YH157';...
    'YH167';...
    'YH168';... % example mouse
    'YH169';... % example mouse
    'YH171';...
    'YH172';...
    };
example_mouse = [0 0 0 0 1 1 0 0];
reverse_only = [0 0 1 1 1 1 1 1];
task_learning = [13550, 4214, 3428, 4112, 5371, 2458, 2700, 2211];
avg_reverse_learning = zeros(numel(Mouse_ID),1);
example_index = 0;
x_lim0 = 0;
before_reverse = 500; % trials
%%
for i_mice = 1:numel(Mouse_ID)
    % load event data
    trial = reverse_mice{i_mice}; 
    
    % trial data
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
    
    reverse_tiral = [];
    
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
            if i>2 && numel(trial{i})>10 && numel(trial{i-1})>10
                if trial{i}(10) ~= trial{i-1}(10)
                    reverse_tiral = [reverse_tiral i];
                end
            end
            if protocol(i) >= 22
                early_licks_count(i) = sum(states == 2 | states == 4);
                is_early_lick_sample(i) = ismember(2,states);
                is_early_lick_delay(i) = ismember(4,states);
            end
        end
    end
    % xlim([current_trial_num(1)-10 current_trial_num(end)+10]);ylim([-1.5 2.5]);
    protocol_advance = diff(protocol);
    protocol_advance_index = find(protocol_advance ~=0);
    subprotocol_advance = diff(sub_protocol);
    subprotocol_advance_index = find(subprotocol_advance ~=0);
    
    %%%%%%%%%%%%%%%%%%%%
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
    if ~isempty(reverse_tiral)
        for i = 1:numel(reverse_tiral)
            xline(reverse_tiral(i),'b','linewidth',2);
        end
        reverse_trial_num = diff(reverse_tiral);
        avg_reverse_learning(i_mice) = mean(reverse_trial_num);
    end
    
    if strcmp(Mouse_ID{i_mice},'YH148')
        reverse_trial_num = reverse_trial_num(2:end);
    end
    
    figure(2), 

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur5C %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3,3,6), hold on, plot(reverse_trial_num/reverse_trial_num(1),'-o');
    if i_mice == numel(Mouse_ID)
        xlabel('Reverse number');
        ylabel('Relative criteria trial number');
    end
    

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur5B %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if example_mouse(i_mice) == 1
        example_index = example_index + 1;
        subplot(4,2,2*(example_index-1)+1), hold on, 
        h1 = plot(movmean(hit_iSession(reverse_tiral(1)-before_reverse:end),avg_size,'omitnan'),'k','LineWidth',1);
        if ~isempty(reverse_tiral)
            for i = 1:numel(reverse_tiral)
                h2 = xline(reverse_tiral(i)-reverse_tiral(1)+before_reverse,'b','linewidth',2);
            end
        end
        h3 = yline(0.7,'g'); ylim([0,1]);
        x_lim = get(gca,'xlim');
        if x_lim(2) > x_lim0
            x_lim0 = x_lim(2);
        end
    end
end

figure(2), % set the same x limit
for i = 1:example_index
    subplot(4,2,2*i-1),
    xlim([0,x_lim0]);
    set(gca,'xtick',before_reverse:2000:x_lim0,'xticklabel',0:2000:x_lim0-before_reverse);
    if i == example_index
        legend(h2,'Reversal');
        ylabel('Performance'); xlabel('Trial number');
    end
end


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur5D %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X = task_learning(reverse_only==1);
y = avg_reverse_learning(reverse_only==1);
subplot(3,3,9),hold on, plot(X, y,'o');
xlabel('Task learning (trials)'); ylabel('Reverse learning (trials)');
[b,bint,r,rint,stats] = regress(y,[ones(1,numel(X)); X]');
plot([min(X), max(X)], [b(1)+b(2)*min(X) b(1)+b(2)*max(X)]);
title(['R^2: ' num2str(stats(1)) '; p = ' num2str(stats(3))]);