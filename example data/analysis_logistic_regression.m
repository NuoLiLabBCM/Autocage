%% load data
clear all;
close all;
addpath('./func/');

% put mice data here
mice_all = {
    '.\Data\YH100';...
    '.\Data\Test';...
    };


%%
warning off;


% all mouse data
fraction_pred_mice = cell(length(mice_all),2);
p_value_mice = cell(length(mice_all),1);
for i_mice = 1:length(mice_all) %
    
    
    %% load data
    mice_name = mice_all{i_mice};
    load([mice_name,'.mat']);
    
    Trial_type_allSession = trial_type*2-1;     % -1 right, 1 left
    Outcome_allSession = trial_outcome;         % 1 correct; 0 error; NaN no response
    Outcome_allSession(Outcome_allSession==3 | Outcome_allSession==0)=nan;
    Outcome_allSession(Outcome_allSession==2)=0;
    Protocol_allSession = protocol;
    
    
    % discard protocol 0 data
    i_discard = find(Protocol_allSession==10);
    Trial_type_allSession(i_discard) = [];
    Outcome_allSession(i_discard) = [];
    Protocol_allSession(i_discard) = [];
    
    nTrials = numel(Trial_type_allSession);
    
    
    %% build regressors
    % current trial stimulus:  -1 right, 1 left
    S_0 = Trial_type_allSession;
    
    history = 5;
    
    % stimulus history
    S = zeros(nTrials,history);
    for i = 1:history
        S(:,i) = [NaN(i,1); Trial_type_allSession(1:end-i)];
    end
    
    % action history
    Action = NaN(nTrials,1);
    for i = 1:nTrials
        if Outcome_allSession(i) == 1
            Action(i) = Trial_type_allSession(i);
        elseif Outcome_allSession(i) == 0
            Action(i) = -1*Trial_type_allSession(i); % double(~Trial_type_allSession(i)); %
        else
            Action(i) = NaN;
        end
    end
    A = zeros(nTrials,history);
    for i = 1:history
        A(:,i) = [NaN(i,1); Action(1:end-i)];
    end
    
    % reward history
    R = zeros(nTrials,history);
    for i = 1:history
        R(:,i) = [NaN(i,1); Outcome_allSession(1:end-i)];
    end
    R(R==0) = -1;
    
    % average stimulus
    Avg_S = NaN(nTrials,1);
    for i = 30:nTrials
        Avg_S(i) = mean(Trial_type_allSession(i-29:i));
    end
    
    % win-stay-lose-switch
    WSLS = A(:,1).*R(:,1);
    
    % exclude protocol 2 and 3
    condition = Protocol_allSession>3;
    S_0 = S_0(condition);
    S = S(condition,:);
    Action = Action(condition);
    A = A(condition,:);
    R = R(condition,:);
    Avg_S = Avg_S(condition);
    WSLS = WSLS(condition);
    
    
    X = [S_0 S A R Avg_S WSLS];
    y = Action; % categorical
    y(y==-1) = 0; % [0,N]
    
    
    
    %% regression
    window_size = 500; step = 100;
    window_center = window_size/2:step:numel(y)-window_size/2;
    
    n_win = min(150,numel(window_center));
    window_center = window_center(1:n_win);
    
    p_sim = zeros(numel(window_center),18);
    p_sim_t = zeros(numel(window_center),18);
    pred_win = zeros(numel(window_center),19);
    pred_win_std = zeros(numel(window_center),19);
    
    B = NaN(size(X,2)+1,numel(window_center));
    dev = NaN(numel(window_center),1); % dev = -2 * log_likelihood
    stats = cell(numel(window_center),1);
    mouse_perf_win = NaN(numel(window_center),1);
    mouse_bias_win = NaN(numel(window_center),1);
    correct_rate_predicted = NaN(numel(window_center),1);
    log_likelihood = NaN(numel(window_center),1);
    cv_bit = cell(size(X,2)+1,1);
    perf_Outcome_allSession = Outcome_allSession(condition);
    perf_TrialType_allSession = Trial_type_allSession(condition);
    model_p_value = zeros(size(X,2),numel(window_center));
    r_left_all = zeros(numel(window_center),1);
    ur_left_all = zeros(numel(window_center),1);
    fraction_pred_win = zeros(numel(window_center),1);
    cv_bit_win = zeros(numel(window_center),1);
    
    
    for i_win = 1:numel(window_center)
                
        range_tmp = window_center(i_win)-window_size/2+1:window_center(i_win)+window_size/2;
        disp(['Time window ',num2str(i_win), '/',num2str(numel(window_center)), '   Trial ',num2str(min(range_tmp)),'-',num2str(max(range_tmp))]);

        
        perf_Outcome_allSession_range = perf_Outcome_allSession(range_tmp);
        perf_TrialType_allSession_range = perf_TrialType_allSession(range_tmp);
        X_range = X(range_tmp,:);
        y_range = y(range_tmp);
        
        
        
        predict_correct_CV = [];
        for i_cross_validation = 1:9
            
            % split training & testing data first <=================
            if i_cross_validation == 1
                train_set_all = (1:400);        %first 400 trials (leave out 20 trial in the middle, becaue your regressors go back 20 trials
                test_set_all = (421:500);       %last 100 trials
            elseif i_cross_validation == 2
                train_set_all = ([1:350 451:500]);
                test_set_all = (371:430);
            elseif i_cross_validation == 3
                train_set_all = ([1:300 401:500]);
                test_set_all = (321:380);
            elseif i_cross_validation == 4
                train_set_all = ([1:250 351:500]);
                test_set_all = (271:330);
            elseif i_cross_validation == 5
                train_set_all = ([1:200 301:500]);
                test_set_all = (221:280);
            elseif i_cross_validation == 6
                train_set_all = ([1:150 251:500]);
                test_set_all = (171:230);
            elseif i_cross_validation == 7
                train_set_all = ([1:100 201:500]);
                test_set_all = (121:180);
            elseif i_cross_validation == 8
                train_set_all = ([1:50 151:500]);
                test_set_all = (71:130);
            elseif i_cross_validation == 9
                train_set_all = ([101:500]);
                test_set_all = (1:80);
            end
            
            
            
            % fit model
            train_set = train_set_all;
            test_set = test_set_all;
            
            
            predict_correct = [];
            y_correct = [];
            for model = 0:18 % 0: full model; 1-18: partial model A_1

                if model ~= 0 % partial model
                    [B_cv,dev_cv,stats_cv] = glmfit(X_range(train_set,:), y_range(train_set) , 'binomial', 'link', 'logit');
                    B_cv_partial = B_cv;
                    B_cv_partial(model+1) = 0;   % column 1 is beta, for model 1 (S0), set second column to 0
                    p_predicted_cv = glmval(B_cv_partial,X_range(test_set,:),'logit');
                    
                else
                    [B_cv,dev_cv,stats_cv] = glmfit(X_range(train_set,:), y_range(train_set) , 'binomial', 'link', 'logit');
                    p_predicted_cv = glmval(B_cv,X_range(test_set,:),'logit');
                end
                
                p_predicted_cv = (p_predicted_cv>0.5);
                
                if model == 0 % full model
                    predict_correct(:,1) = (y_range(test_set)==p_predicted_cv);
                else
                    predict_correct(:,model+1) = (y_range(test_set)==p_predicted_cv);
                end
            end
            
            predict_correct_CV = cat(1, predict_correct_CV, predict_correct);
            
        end
        
        
        %% bootstrap
        btstrp_num = 1000;
        pred_perf_iBtstrp = zeros(btstrp_num,19);
        %warning('off','last');
        for i_btstrp = 1:btstrp_num
            i_sample = randsample(size(predict_correct_CV,1),size(predict_correct_CV,1),'true');
            for i_model = 1:19
                predict_correct_ibtstrp = predict_correct_CV(i_sample,i_model);
                pred_perf_iBtstrp(i_btstrp,i_model) = sum(predict_correct_ibtstrp)/size(predict_correct_ibtstrp,1);
            end
        end
        
        
        
        for model = 1:18
            mean_diff = pred_perf_iBtstrp(:,1)-pred_perf_iBtstrp(:,model+1);
            p_sim(i_win,model)=sum(mean_diff<=0)/length(mean_diff); %<=========== there many case where both model gave the same preidction. the GLM is degenerate
        end
        pred_win(i_win,:) = mean(pred_perf_iBtstrp);
        pred_win_std(i_win,:) = std(pred_perf_iBtstrp);
        mouse_perf_win(i_win) = nansum(perf_Outcome_allSession_range)/sum(~isnan(perf_Outcome_allSession_range));
        perf_l = nansum(perf_Outcome_allSession_range(perf_TrialType_allSession_range == 1))/sum(~isnan(perf_Outcome_allSession_range(perf_TrialType_allSession_range == 1)));
        perf_r = nansum(perf_Outcome_allSession_range(perf_TrialType_allSession_range == -1))/sum(~isnan(perf_Outcome_allSession_range(perf_TrialType_allSession_range == -1)));
        mouse_bias_win(i_win) = perf_l - perf_r;

    end
    
    fraction_pred_mice{i_mice,1} = pred_win;
    fraction_pred_mice{i_mice,2} = pred_win_std;
    p_value_mice{i_mice} = p_sim;
    
    
    
    
    %% plot data
    figure,
    
    subplot(4,1,1), title([mice_name ': Performance']);
    hold on, plot(mouse_perf_win*100);
    ylim([20 90]);xlim([0, numel(window_center)+1]);
    plot([0 numel(window_center)+1],[50 50],'--k'); % plot([0 numel(window_center)+1],[0.7 0.7],'--k');
    ylabel('Correct Rate');
    
    subplot(4,1,2), ylabel('Fraction of predict');
    hold on,
    %         plot(pred_win(:,1),'k');
    %         plot(pred_win(:,2),'b');
    %         plot(pred_win(:,8),'r');
    
    shadedErrorBar([],pred_win(:,2),pred_win_std(:,2),'lineprops','b');
    shadedErrorBar([],pred_win(:,8),pred_win_std(:,8),'lineprops','r');
    shadedErrorBar([],pred_win(:,1),pred_win_std(:,1),'lineprops','k');
    xlim([0 numel(window_center)+1]);
    legend('S0','A1','full','Location','eastoutside');
    
    subplot(4,1,3), hold on,
    for i = 1:size(p_sim,1)
        for j = 1:size(p_sim,2)
            if p_sim(i,j) < 0.001 % /numel(window_center) % multiple comparison
                h1 = plot(i,j,'o','markersize',6);
            elseif p_sim(i,j) < 0.01 % /numel(window_center)
                h2 = plot(i,j,'o','markersize',4);
            elseif p_sim(i,j) < 0.05 % /numel(indow_center)
                h3 = plot(i,j,'o','markersize',2);
            end
        end
    end
    ylabel('models'); title('w/o multiple correction!!!');
    xlim([0 numel(window_center)+1]);
    ylim([0.5 18+.5]);
    set(gca,'ytick',1:18,'yticklabel',{'full-S0','full-S1','','','','','full-A1','','','','','full-R1','','','','','full-Avg_S','full-WSLS'});
    %legend([h1, h2, h3],{'p<0.001','p<0.01','p<0.05'},'Location','eastoutside');
    
    subplot(4,1,4), hold on,
    p_sim_tmp = p_sim;
    p_sim_tmp(p_sim<0.001) = 0.001;
    h = semilogy(p_sim_tmp);
    set(h(1),'linewidth',2)
    set(h(7),'linewidth',2)
    set(gca, 'YScale', 'log')
    xlim([0.5 size(p_sim_tmp,1)+.5]);
    xlabel('Time window'); ylabel('p-value');
    legend('S0','S1','S2','S3','S4','S5','A1','A2','A3','A4','A5','R1','R2','R3','R4','R5','avgS','WSLS','Location','eastoutside');
    
end
