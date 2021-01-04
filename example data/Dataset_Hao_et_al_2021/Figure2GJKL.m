%% for figure 2
load autocage_fixation.mat % fixation_all

figure,
mouse_num = 37;
disp_num = 40; % days


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur2G %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(4,3,7), hold on,
for i = 1:mouse_num
    plot(fixation_all(i).total_duration_days/60,'color',[.6 .6 .6],'linewidth',0.5); 
end
ylabel('Total duration per day (min)');
ylim([0 150]); xlim([0.5 disp_num+0.5]);
total_duration_mean = zeros(disp_num,1);
for i = 1:disp_num
    k = 0;
    for j = 1:mouse_num
        if i <= fixation_all(j).num_days && ~isnan(fixation_all(j).avg_fixation_duration_days(i))
            total_duration_mean(i) = total_duration_mean(i) + fixation_all(j).total_duration_days(i);
            k = k + 1;
        end
    end
    total_duration_mean(i) = total_duration_mean(i)/k;
end
plot(total_duration_mean/60,'k','linewidth',2);

subplot(4,18,36+6+1),hold on,
total_duration_mouse = zeros(mouse_num,1);
for i = 1:mouse_num
    total_duration_mouse(i) = mean(fixation_all(i).total_duration_days)/60;
end
plot(1+(0.6*rand(mouse_num,1)-0.3),total_duration_mouse,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
h = barwitherr(std(total_duration_mouse), mean(total_duration_mouse),'edgecolor','none','linewidth',2,'facecolor','k');% Plot with errorbars
set(h,'FaceAlph',0.5);
xlim([0.5 1.5]);ylim([0 150]); axis off;
title([num2str(mean(total_duration_mouse),'%.0f') ' \pm ' num2str(std(total_duration_mouse),'%.0f')]);



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur2J %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(4,3,10), hold on,
for i = 1:mouse_num
    plot(fixation_all(i).struggle_rate*100,'color',[.6 .6 .6],'linewidth',0.5); 
end
ylabel('Self-release rate (%)');
ylim([0 80]); xlim([0.5 disp_num+0.5]);
struggle_rate_mean = zeros(disp_num,1);
for i = 1:disp_num
    k = 0;
    for j = 1:mouse_num
        if i <= fixation_all(j).num_days && ~isnan(fixation_all(j).avg_fixation_duration_days(i))
            struggle_rate_mean(i) = struggle_rate_mean(i) + fixation_all(j).struggle_rate(i);
            k = k + 1;
        end
    end
    struggle_rate_mean(i) = struggle_rate_mean(i)/k;
end
plot(struggle_rate_mean*100,'k','linewidth',2);

subplot(4,18,54+6+1),hold on,
struggle_rate_mouse = zeros(mouse_num,1);
for i = 1:mouse_num
    struggle_rate_mouse(i) = 100*fixation_all(i).overall_struggle_rate;
end
plot(1+(0.6*rand(mouse_num,1)-0.3),struggle_rate_mouse,'o','color',[.6 .6 .6],'MarkerSize',5,'LineWidth',1.5);
h = barwitherr(std(struggle_rate_mouse), mean(struggle_rate_mouse),'edgecolor','none','linewidth',2,'facecolor','k');% Plot with errorbars
set(h,'FaceAlph',0.5);
xlim([0.5 1.5]); ylim([0 80]); axis off;
title([num2str(mean(struggle_rate_mouse),'%.0f') ' \pm ' num2str(std(struggle_rate_mouse),'%.0f')]);



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur2K %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(4,3,3), hold on,
bin24_all = [];
for i_mouse = 1:mouse_num
    bin24_all = [bin24_all; fixation_all(i_mouse).bin24./fixation_all(i_mouse).num_days];
end
barwitherr(std(bin24_all)/sqrt(mouse_num),mean(bin24_all),'facecolor','k');
% h = bar(mean(cat(1,fixation_all.bin24)),'facecolor','k');
xlim([0 25]); ylim([0 15]);
set(gca,'XTick', [1 6 12 18 24], 'XTickLabel', {'1','6','12','18','24'});
xlabel('Hours'); ylabel('Avg. fixation number/mouse/day');



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%% Figur2L %%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(4,3,6),hold on,
ifi = cat(1,fixation_all.ifi);
ifi(ifi>1.5) = 1.5;
h = histogram(ifi,0:0.05:1.5,'Normalization','probability','facecolor','k');
xlabel('Inter-fixation interval (min)'); ylabel('Fraction');