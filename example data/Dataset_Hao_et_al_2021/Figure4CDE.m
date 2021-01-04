% for figure 4

figure,

load regression_data.mat % autocage_p_value manual_rig_p_value fraction_pred_mice



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur4C %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

full = NaN(32,150);
full_S = NaN(32,150);
shuffle = NaN(32,150);
full_A = NaN(32,150);
for i = 1:32
    full(i,1:size(fraction_pred_mice{i},1)) = fraction_pred_mice{i}(:,1)';
    full_S(i,1:size(fraction_pred_mice{i},1)) = fraction_pred_mice{i}(:,2)';
    shuffle(i,1:size(fraction_pred_mice{i},1)) = fraction_pred_mice{i}(:,3)';
    full_A(i,1:size(fraction_pred_mice{i},1)) = fraction_pred_mice{i}(:,4)';
end
figure, subplot(2,2,1), hold on,
shadedErrorBar([],nanmean(full),nanstd(full)/sqrt(32),'lineprops','k');
shadedErrorBar([],nanmean(full_S),nanstd(full_S)/sqrt(32),'lineprops','b');
shadedErrorBar([],nanmean(full_A),nanstd(full_A)/sqrt(32),'lineprops','r');
shadedErrorBar([],nanmean(shuffle),nanstd(shuffle)/sqrt(32),'lineprops','--k');
legend('full model','full - (S0-5)', 'full - (A1-5)', 'Shuffled');


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur4D %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_all = cell(18,1);
for i_mice = 1:numel(autocage_p_value)
    p_values = autocage_p_value{i_mice};
    for j = 1:18
        for i_disp = 1:150
            if i_disp <= size(p_values,1)
                p_all{j}(i_mice,i_disp) = double(p_values(i_disp,j)<0.05); % multiple correction
            else
                p_all{j}(i_mice,i_disp) = NaN;
            end
        end
    end
end
subplot(2,2,1),hold on,
for i = 1:18
    plot(nanmean(p_all{i}));
end

p_all = cell(18,1);
for i_mice = 1:numel(manual_rig_p_value)
    p_values = manual_rig_p_value{i_mice};
    for j = 1:18
        for i_disp = 1:150
            if i_disp <= size(p_values,1)
                p_all{j}(i_mice,i_disp) = double(p_values(i_disp,j)<0.05); % multiple correction
            else
                p_all{j}(i_mice,i_disp) = NaN;
            end
        end
    end
end
subplot(2,2,2),hold on,
for i = 1:18
    plot(nanmean(p_all{i}));
end
legend({'current stim','1-back stim','2-back stim','3-back stim','4-back stim','5-back stim',...
    '1-back action','2-back action','3-back action','4-back action','5-back action',...
    '1-back reward','2-back reward','3-back reward','4-back reward','5-back reward',...
    'average stim','win-stay-lose-switch'});





% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur4E %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


sig_count = zeros(18,2);
p_thres = 0.05; % w/o multiple correction
consec_bins = 3;

for i_setup = 1:2
    if i_setup == 1
        p_value_mice = autocage_p_value;
    else
        p_value_mice = manual_rig_p_value;
    end
    for i_mice = 1:numel(p_value_mice)
        p_value = p_value_mice{i_mice};
        for i_reg = 1:18
            tmp_p = p_value(:,i_reg);
            for j = 1:numel(tmp_p)-consec_bins+1
                if all(tmp_p(j:j+consec_bins-1)<p_thres)
                    sig_count(i_reg,i_setup) = sig_count(i_reg,i_setup) + 1;
                    break;
                end
            end
        end
    end
    sig_count_f(:,i_setup) = sig_count(:,i_setup)/numel(p_value_mice);
end

figure,

subplot(2,2,3), hold on,
i_disp = [1 2 3 7 8 9 12]; % S0, S1, S2, A1, A2, A3, and R1
i_other = [4:6 10:11 13:18];
data = sig_count_f(i_disp,:);
err = data.*(1-data);
err(:,1) = err(:,1)/sqrt(32);
err(:,2) = err(:,2)/sqrt(64);
barwitherr(err,data);
legend('home-cage','manual-rig','all others avg.');

data_other = sig_count(i_other,:);
all_other = sum(data_other')/96;
shadedErrorBar([0.5 numel(i_disp)+.5],[mean(all_other) mean(all_other)],[std(all_other) std(all_other)],'lineprops','--k');