function varargout = homecage_gui(varargin)
% HOMECAGE_GUI shows graphic user interfaces of the automatic homecages
%
%      Current Cage Number: 
%           30
%
%      Function:
%           Display Messages received from (up to 30) Arduino_Controllers (Homecages);
%           Send motor_move command to Controller to move 3 motors;
%
%
%       Created on         09/15/2018   by Y.H.
%       Last modified on   01/18/2019   by Y.H.

%% GUI preparation
handles.total_cage_num = 30;
handles.default_color = [.7 .9 .8];  % default background color [.93 .93 .93];

hsv_map = hsv(24);
handles.mymap = hsv_map(1:8,:); % 1:8 from red to yellow to green
% figure,colormap(mymap);

serialPorts = instrhwinfo('serial');
% nPorts = length(serialPorts.SerialPorts);

handles.hfigure = figure('name','HomeCage GUI', 'numbertitle','off', 'MenuBar','none',...
    'CloseRequestFcn', @closeGUI_Callback, 'Position', get(0,'Screensize'));
for cage_i = 1:handles.total_cage_num
    handles.hsubplot(cage_i) = subplot(4,8,cage_i);
    box on;
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    
    % Current supplot panel posititon
    pos(cage_i,:) = get(handles.hsubplot(cage_i), 'Position'); % [0.1300 0.7673 0.0759 0.1577]
    % set(handles.hsubplot(cage_i),'Position',[pos(cage_i,1)+0.06,pos(cage_i,2),pos(cage_i,3),pos(cage_i,4)]);
    if ~exist(['./Data/Cage',int2str(cage_i),'/YH00'], 'dir')
        mkdir(['./Data/Cage',int2str(cage_i)],'/YH00');
    end
    % db_table stores each cage's Number, COM port, Mice name, startDate
    handles.db_table{cage_i}.Cage = cage_i;
    handles.db_table{cage_i}.startDate = NaN;
    handles.db_table{cage_i}.Mouse = 'YH00';
    handles.db_table{cage_i}.COM = 'NaN';
    
    % Title (cage number)
    handles.htext_title(cage_i) = uicontrol('Style','text','String',['Cage ',int2str(cage_i),' - '],...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'FontSize',11, 'FontWeight','bold',...
        'Position',[pos(cage_i,1),pos(cage_i,2)+0.158,0.04,0.02]);
    % Mice Name
    handles.hedit_mice(cage_i) = uicontrol('Style','edit','String','YH00',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@edit_mice_Callback,...
        'Position',[pos(cage_i,1)+0.037,pos(cage_i,2)+0.159,0.02,0.02]);
    % Days
    handles.htext_days(cage_i) = uicontrol('Style','text','String','00 days ',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Position',[pos(cage_i,1)+0.057,pos(cage_i,2)+0.16,0.03,0.015]); % 'ButtonDownFcn',@text_days_Buttondown,...
    
    % Popup for chosing COM port
    handles.hpopup(cage_i) = uicontrol('Style','popupmenu',...
        'String',[{'Chose a port'} ; serialPorts.SerialPorts],...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@popup_com_Callback,...
        'ButtonDownFcn',@popup_com_Buttondown,...
        'Position',[pos(cage_i,1)+0.005,pos(cage_i,2)+0.102,0.032,0.03],...
        'TooltipString', 'Right-click to update COM list');
    handles.hbutton_open(cage_i) = uicontrol('Style','pushbutton','String','open',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@button_open_Callback,...
        'Position',[pos(cage_i,1)+0.01,pos(cage_i,2)+0.134,0.02,0.018]);
    % trial numbers in 24 hr.
    handles.htext_trailNum24hr(cage_i) = uicontrol('style','text',...
        'String', 'TrialNum/24hr',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.035,pos(cage_i,2)+0.134,0.036,0.018]);
    
    % Select a start date button
    handles.hedit_date(cage_i) = uicontrol('Style','edit','String','dd-mmm',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@edit_date_Callback,...
        'Position',[pos(cage_i,1)+0.04,pos(cage_i,2)+0.11,0.025,0.02]);
    handles.hbutton_chooseStartDate(cage_i) = uicontrol('Style','pushbutton','String','...',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@button_chooseStartDate_Callback,...
        'Position',[pos(cage_i,1)+0.065,pos(cage_i,2)+0.11,0.01,0.02]);
    
    % Plot button
    handles.hbutton_plotPW(cage_i) = uicontrol('Style','pushbutton','String','Plot_P/W',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@button_plotPW_Callback,...
        'Position',[pos(cage_i,1)+0.04,pos(cage_i,2)+0.07,0.03,0.03],...
        'Enable','on');
    % Open Message txt file
    handles.hbutton_msg(cage_i) = uicontrol('Style','pushbutton','String','Msg',...
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@button_msg_Callback,...
        'Position',[pos(cage_i,1)+0.005,pos(cage_i,2)+0.07,0.03,0.03]);
    % Trial No. and Perf100
    handles.htext_trailNum(cage_i) = uicontrol('style','text',...
        'String', 'Trial No.',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.005,pos(cage_i,2)+0.045,0.03,0.02]);
    handles.htext_hf1(cage_i) = uicontrol('style','text',...
        'String', ' - ',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.035,pos(cage_i,2)+0.045,0.01,0.02]);
    handles.htext_perf100(cage_i) = uicontrol('style','text',...
        'String', '0%',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.045,pos(cage_i,2)+0.045,0.03,0.02]);
    %     handles.htext_percent1(cage_i) = uicontrol('style','text',...
    %         'String', '%',        'Units', 'normalized',...
    %         'Position',[pos(cage_i,1)+0.065,pos(cage_i,2)+0.045,0.01,0.02]);
    
    % Protocol info: currentProtocol-trialsNumsinThisProtocol-Perf30
    handles.htext_Protocol(cage_i) = uicontrol('style','text',...
        'String', 'Protocol - TrialNum - 0%',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.005,pos(cage_i,2)+0.025,0.07,0.02]);
    %     handles.htext_hf2(cage_i) = uicontrol('style','text',...
    %         'String', '-',        'Units', 'normalized',...
    %         'Position',[pos(cage_i,1)+0.015,pos(cage_i,2)+0.025,0.005,0.02]);
    %     handles.htext_trialNumProtocol(cage_i) = uicontrol('style','text',...
    %         'String', '0',        'Units', 'normalized',...
    %         'Position',[pos(cage_i,1)+0.02,pos(cage_i,2)+0.025,0.02,0.02]);
    %     handles.htext_hf3(cage_i) = uicontrol('style','text',...
    %         'String', '-',        'Units', 'normalized',...
    %         'Position',[pos(cage_i,1)+0.04,pos(cage_i,2)+0.025,0.005,0.02]);
    %     handles.htext_perf30(cage_i) = uicontrol('style','text',...
    %         'String', '0',        'Units', 'normalized',...
    %         'Position',[pos(cage_i,1)+0.045,pos(cage_i,2)+0.025,0.02,0.02]);
    %     handles.htext_percent2(cage_i) = uicontrol('style','text',...
    %         'String', '%',        'Units', 'normalized',...
    %         'Position',[pos(cage_i,1)+0.065,pos(cage_i,2)+0.025,0.005,0.02]);
    
    % Weight info xx g
    handles.htext_weight1(cage_i) = uicontrol('style','text',...
        'String', 'Weight:',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.01,pos(cage_i,2)+0.005,0.02,0.02]);
    handles.htext_weight2(cage_i) = uicontrol('style','text',...
        'String', '0.0',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.03,pos(cage_i,2)+0.005,0.03,0.02]);
    handles.htext_g(cage_i) = uicontrol('style','text',...
        'String', 'g',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.06,pos(cage_i,2)+0.005,0.005,0.02]);
    
    set_background_color(cage_i,handles.default_color); % set default background color
end


% Protocol Info
handles.htext_protocolInfo = uicontrol('Style','text',...
    'String',{'Protocol Info: ',...
                         '0: Free licking -> Headfixation 25 sec;',...
                         '1-9: Sample Protocol',...
                         '10-18: Delay Protocol (18: final)',...
                         '19: Start to Optical Stimulation'},...
    'Units', 'normalized',        'Tag', 'text_errorMsg',...
    'Position',[0.01,0.67,0.1,0.2]);


% listbox for error message
handles.hlistbox = uicontrol('Style','listbox',...
    'Units', 'normalized','Position',[0.005 0.17 0.12 0.3]);
handles.hbutton_deleteSel = uicontrol('Style','pushbutton','String','Delete Sel',...
    'Units', 'normalized',        'Tag', 'button_deleteSel',...
    'Callback',@button_deleteSel_Callback,...
    'Position',[0.08,0.13,0.04,0.03]);
handles.hbutton_clearAll = uicontrol('Style','pushbutton','String','Clear All',...
    'Units', 'normalized',        'Tag', 'button_clearAll',...
    'Callback',@button_clearAll_Callback,...
    'Position',[0.035,0.13,0.04,0.03]);
handles.htext_errorMsg = uicontrol('Style','text','String','Error Message',...
    'Units', 'normalized',        'Tag', 'text_errorMsg',...
    'Position',[0.035,0.47,0.06,0.02]);


% Show Color bar for the trial number in last 24 hours
colormap(handles.mymap);
colorbar('Ticks',0.0625:0.125:1,'TickLabels',{'80','160','240','320','400','480','560 (24hr)','640 Trials'},...
    'Position',[0.72, 0.11, 0.01, 0.16]);

% motor panel
handles.hpanel = uipanel('Title','Motor Control',...
    'Position',[.765 .11 .09 .17]);
% Popup for chosing Which Cage
handles.htext_whichCage = uicontrol('Style','text','String','Cage: ',...
    'Units', 'normalized',        'Tag', 'text_whichcage',...
    'Position',[0.78,0.225,0.02,0.02]);
handles.hpopup_Cage = uicontrol('Style','popupmenu',...
    'String',[{'Chose a Cage'} ; 1:30],...
    'Units', 'normalized',        'Tag', 'popup_whichcage',...
    'Position',[0.80,0.23,0.035,0.02],...
    'TooltipString', 'Which Cage to Control');
% motor FB
handles.htext_motorFB = uicontrol('Style','text','String','MotorFB',...
    'Units', 'normalized',        'Tag', 'text_motorFB',...
    'Position',[0.77,0.185,0.03,0.02]);
handles.hedit_motorFB = uicontrol('Style','edit','String','0',...
    'Units', 'normalized',        'Tag', 'edit_motorFB',...
    'Position',[0.80,0.19,0.025,0.02]);
handles.hbutton_motorFB = uicontrol('Style','pushbutton','String','Move',...
    'Units', 'normalized',        'Tag', 'button_motorFB',...
    'Callback',{@button_moveMotor,'FB'},...
    'Position',[0.83,0.19,0.02,0.02]);
% motor LR
handles.htext_motorLR = uicontrol('Style','text','String','motorLR',...
    'Units', 'normalized',        'Tag', 'text_motorLR',...
    'Position',[0.77,0.155,0.03,0.02]);
handles.hedit_motorLR = uicontrol('Style','edit','String','70',...
    'Units', 'normalized',        'Tag', 'edit_motorLR',...
    'Position',[0.80,0.16,0.025,0.02]);
handles.hbutton_motorLR = uicontrol('Style','pushbutton','String','Move',...
    'Units', 'normalized',        'Tag', 'button_motorLR',...
    'Callback',{@button_moveMotor,'LR'},...
    'Position',[0.83,0.16,0.02,0.02]);
% motor Pole
handles.htext_motorPole = uicontrol('Style','text','String','motorPole',...
    'Units', 'normalized',        'Tag', 'text_motorPole',...
    'Position',[0.77,0.125,0.03,0.02]);
handles.hedit_motorPole = uicontrol('Style','edit','String','30',...
    'Units', 'normalized',        'Tag', 'edit_motorPole',...
    'Position',[0.80,0.13,0.025,0.02]);
handles.hbutton_motorPole = uicontrol('Style','pushbutton','String','Move',...
    'Units', 'normalized',        'Tag', 'button_motorPole',...
    'Callback',{@button_moveMotor,'Pole'},...
    'Position',[0.83,0.13,0.02,0.02]);



% load and save parameters (COM Port and Mice Name)
handles.hbutton_saveParas = uicontrol('Style','pushbutton','String','Save Paras',...
    'Units', 'normalized',        'Tag', 'button_saveparas',...
    'Callback',@button_saveParas_Callback,...
    'Position',[0.86,0.21,0.04,0.03]);
handles.hbutton_loadParas = uicontrol('Style','pushbutton','String','Load Paras',...
    'Units', 'normalized',        'Tag', 'button_loadparas',...
    'Callback',@button_loadParas_Callback,...
    'Position',[0.86,0.16,0.04,0.03]);

% Create Timer to update GUI (background color and weight) periodically
TimerInteval = 600;       % Default timer interval (1 min)
handles.update_timer = timer('Name','MainTimer','TimerFcn',{@updateGUI},...
    'Period',TimerInteval,'ExecutionMode','fixedRate');
start(handles.update_timer);
%% Callback Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% Callback Functions %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Mice Name
    function edit_mice_Callback(source,eventdata)
        % Create a folder for the mice under the dirctory of that Cage
        Cage = get(source,'Tag');
        Mice = get(source,'String');
        if ~exist(['./Data/Cage',Cage,'/',Mice], 'dir')
            mkdir(['./Data/Cage',Cage,'/',Mice]);
        end
        handles.db_table{str2double(Cage)}.Mouse = Mice;
    end

%     function text_days_Buttondown(source,eventdata)
%         % Create a folder for the mice under the dirctory of that Cage
%         Cage = get(source,'Tag');
%         str = get(source,'String');
%         days = str2double(str(1:2));
%         if days < 9
%             set(handles.htext_days(str2double(Cage)),'String',['0', num2str(days+1),' days']);
%         else
%             set(handles.htext_days(str2double(Cage)),'String',[num2str(days+1),' days']);
%         end
%     end


% Popup Menu: left click to chose a port
    function popup_com_Callback(source,eventdata)
        % Chose a COM port
        Cage = str2double(get(source,'Tag'));
        value = get(source,'Value');
        str = get(source,'String');
        if value == 1
            handles.db_table{Cage}.COM = 'NaN';
        else
            handles.db_table{Cage}.COM = str{value};
        end
    end
% Popup Menu: right click to update COM list
    function popup_com_Buttondown(source,eventdata)
        Cage = str2double(get(source,'Tag'));
        serialPorts_tmp = instrhwinfo('serial');
        set(handles.hpopup(Cage),'String',[{'Chose a port'} ; serialPorts_tmp.SerialPorts]);
    end

% Open a COM port
    function button_open_Callback(source,eventdata)
        % Display mesh plot of the currently selected data.
        % handles = guidata(source);
        Cage = get(source,'Tag');
        Cage_num = str2double(Cage);
        if strcmp(get(source,'String'), 'open')
            % open com port
            str_com = handles.db_table{Cage_num}.COM;
            if strcmp(str_com(1:3), 'COM')
                serial_obj = instrfind();
                for i = 1:numel(serial_obj)
                    if strcmp(serial_obj(i).Name(8:end),str_com)
                        fclose(serial_obj(i));
                        delete(serial_obj(i));
                    end
                end
                % open COM port
                handles.s{Cage_num} = serial(str_com);
                handles.s{Cage_num}.BytesAvailableFcnMode = 'terminator';
                handles.s{Cage_num}.terminator = 'CR/LF';
                handles.s{Cage_num}.BaudRate = 115200;
                handles.s{Cage_num}.InputBufferSize = 5200;
                handles.s{Cage_num}.BytesAvailableFcn = {@serial_callback};%@instrcallback;
                try
                    fopen(handles.s{Cage_num});
                    
                    % eval(['handles.s',Cage,'=s;']);
                catch e
                    disp('error in Open Serial Port...');
                    disp(e);
                    msgbox('Serial port open failed.');
                end
                set(source,'String','close');
                % open MSG.txt file to write
                
                % handles.s{Cage_num}.RecordName = ['./Data/Cage',Cage,'/',Mice/msg_r.txt];
                % handles.s{Cage_num}.RecordDetail = 'verbose';
                % record(handles.s{Cage_num});
                
                try
                    Mice = get(handles.hedit_mice(Cage_num),'String');
                    handles.fileID(Cage_num) = fopen(['./Data/Cage',Cage,'/',Mice,'/msg.txt'],'a');
                catch e
                    disp('Create Message File...');
                    disp(e);
                end
                
                % trial-time and weight mat file to record
                if exist(['./Data/Cage',Cage,'/',Mice,'/allmat.mat'], 'file') == 0
                    handles.matObj{Cage_num} = matfile(['./Data/Cage',Cage,'/',Mice,'/allmat.mat'],'Writable',true);
                    handles.matObj{Cage_num}.trial_time(1,1:4) = [NaN NaN NaN NaN]; % trial num, datetime,trial type, outcome
                    handles.matObj{Cage_num}.weight = NaN(1,121);% 1st col: datetime; 2:121 col: weight data in float (4 bytes/data)
                    handles.matObj{Cage_num}.startDate = handles.db_table{Cage_num}.startDate; % serial datetime % del
                else
                    handles.matObj{Cage_num} = matfile(['./Data/Cage',Cage,'/',Mice,'/allmat.mat'],'Writable',true);
                end
                
                % update background color
                try
                    [mRow,~] = size(handles.matObj{Cage_num},'trial_time');
                    trial_time = handles.matObj{Cage_num}.trial_time(1:mRow,1:4);
                    current_time = now;
                    for j = mRow:-1:2
                        if trial_time(j,2) < current_time-1
                            break;
                        end
                    end
                    if ~isempty(j)
                        num_trial_24hr = trial_time(mRow,1) - trial_time(j,1);
                        set(handles.htext_trailNum24hr(Cage_num),'String',[num2str(num_trial_24hr),' (24hr)']);
                        if num_trial_24hr > 640
                            num_trial_24hr = 640;
                        end
                        if num_trial_24hr == 0
                            num_trial_24hr = 1;
                        end
                        set_background_color(Cage_num,handles.mymap(ceil(num_trial_24hr/80),:));
                    else
                        set(handles.htext_trailNum24hr(Cage_num),'String',[num2str(0),' (24hr)']);
                        set_background_color(Cage_num,handles.mymap(1,:));
                    end
                catch e
                    disp('Update GUI... trial num in 24 hr');
                    disp(e);
                    set_background_color(Cage_num,handles.mymap(1,:));
                end
                
                % update weight
                try
                    [mRow,~] = size(handles.matObj{Cage_num},'weight');
                    if mRow > 12 % 20 sec data
                        weight_data = handles.matObj{Cage_num}.weight(mRow-12:mRow,2:121);
                    else
                        weight_data = handles.matObj{Cage_num}.weight(1:mRow,2:121);
                    end
                    [mRow, ~] = size(weight_data); % 120 mCol
                    weight_data2 = zeros(mRow, 30);
                    for k = 1:mRow
                        for j = 1:30
                            weight_data2(k,j) = typecast(uint8(weight_data(k,j*4-3:j*4)), 'single');
                        end
                    end
                    [N,edges] = histcounts(weight_data2(:),20);
                    set(handles.htext_weight2(Cage_num), 'String', num2str(edges(N==max(N))));
                catch e
                    disp('Update GUI... weight');
                    disp(e);
                end
                
                % update days
                set(handles.htext_days(Cage_num),'String',sprintf('%2.1f days',now - handles.db_table{Cage_num}.startDate));
                
            else
                msgbox('Please chose a COM port first!');
            end
        else
            % close com port
            try
                fclose(handles.fileID(Cage_num));
                fclose(handles.s{Cage_num});
                delete(handles.s{Cage_num});
            catch e
                disp('Close a Serial Port...');
                disp(e);
            end
            set(source,'String','open');
            set_background_color(Cage_num,handles.default_color); % set default background color
        end
    end

% set start Date button
    function button_chooseStartDate_Callback(source,eventdata)
        Cage_num = get(source,'Tag');
        %         uicalendar('Weekend', [1 0 0 0 0 0 1], ...
        %             'SelectionType', 1, ...
        %             'DestinationUI', handles.hedit_date(str2double(Cage_num)));
        D = uigetdate();
        try
            set(handles.hedit_date(str2double(Cage_num)),'String',datestr(D,'dd-mmm'));
            handles.db_table{str2double(Cage_num)}.startDate = D;
            handles.matObj{Cage_num}.startDate = D;
        catch e
            disp('Choose Start Date!');
            disp(e);
        end
    end

% Plot buttons
    function button_plotPW_Callback(source,eventdata)
        figure,
        % plot trial history data in last 24 hr
        subplot(3,1,1),hold on,
        Cage_num = str2double(get(source,'Tag'));
        [mRow,~] = size(handles.matObj{Cage_num},'trial_time'); % trial num, datetime,trial type, outcome
        trialData = handles.matObj{Cage_num}.trial_time(1:mRow,1:4);
        trialData_time = trialData(:,2);
        current_time = now;
        [trialData_time_24,~] = find(trialData_time>current_time-1);
        if ~isempty(trialData_time_24)
            num_trial_24hr = numel(trialData_time_24);
            for i = trialData_time_24(1):mRow
                % 'Others'-3 || Reward-1 || No Response-0 || Time Out (error)-2
                switch trialData(i,4)
                    case 0
                        plot(trialData(i,2),trialData(i,3),'bo');
                    case 1
                        plot(trialData(i,2),trialData(i,3),'g.','markersize',25);
                    otherwise
                        plot(trialData(i,2),trialData(i,3),'r.','markersize',25);
                end
            end
            xlim([current_time-1 current_time]); ylim([-0.5 1.5]);
            set(gca,'ytick',[0 1],'yticklabel',{'Right','Left'});
            set(gca,'xtick',[current_time-1 current_time-0.75 current_time-0.5 current_time-0.25 current_time],'xticklabel',{'0','6','12','18','24'});
            title([num2str(num_trial_24hr), ' trials in last 24-hour']);
        end
        
        % plot weight data in last 48 hours
        subplot(3,1,2),hold on,
        [mRow,~] = size(handles.matObj{Cage_num},'weight');
        weight_date = handles.matObj{Cage_num}.weight(1:mRow,1);
        [weight_date_24,~] = find(weight_date>current_time-1);
        [weight_date_48,~] = find(weight_date>current_time-2);
        [weight_date_72,~] = find(weight_date>current_time-3);
        if ~isempty(weight_date_24)
            weight = handles.matObj{Cage_num}.weight(weight_date_24(1):mRow,1:121);
            [mRow2,~] = size(weight);
            weight_data = zeros(30,mRow2);
            for i = 1:mRow2
                weight_data(:,i) = typecast(uint8(weight(i,2:121)), 'single')';
            end
            plot(weight(:,1),mean(weight_data),'k.');
            %
            %             sample_ind = 20:20:mRow2;
            %             weight_data_avg = zeros(numel(sample_ind),1);
            %             for i = 1:numel(sample_ind)
            %                 weight_data_tmp = weight_data(:,sample_ind(i)-19:sample_ind(i));
            %                 [N,edges] = histcounts(weight_data_tmp(:),20);
            %                 max_weight = edges(N==max(N));
            %                 weight_data_avg(i) = max_weight(1);
            %             end
            %             plot(weight(20:20:mRow2,1),weight_data_avg,'linewidth',3);
            %
            [N,edges] = histcounts(weight_data(:),100);
            ind = find(N==max(N));
            weight_24 = edges(ind(1));
        end
        if ~isempty(weight_date_48)
            weight = handles.matObj{Cage_num}.weight(weight_date_48(1):weight_date_24(1),1:121);
            [mRow2,~] = size(weight);
            if mRow2 > 1
                weight_data = zeros(30,mRow2);
                for i = 1:mRow2
                    weight_data(:,i) = typecast(uint8(weight(i,2:121)), 'single')';
                end
                [N,edges] = histcounts(weight_data(:),100);
                ind = find(N==max(N));
                weight_48 = edges(ind(1));
            else
                weight_48 = NaN;
            end
        end
        if ~isempty(weight_date_72)
            weight = handles.matObj{Cage_num}.weight(weight_date_72(1):weight_date_48(1),1:121);
            [mRow2,~] = size(weight);
            if mRow2 > 1
                weight_data = zeros(30,mRow2);
                for i = 1:mRow2
                    weight_data(:,i) = typecast(uint8(weight(i,2:121)), 'single')';
                end
                [N,edges] = histcounts(weight_data(:),100);
                ind = find(N==max(N));
                weight_72 = edges(ind(1));
            else
                weight_72 = NaN;
            end
        end
        title(['Avg. ', num2str(weight_24), ' g in 24-hr (48-hr: ', num2str(weight_48),' g, 72-hr: ',num2str(weight_72), ' g)']);
        xlim([current_time-1 current_time]);
        ylabel('weight (g)'); xlabel('Time (hour)'); ylim([15 30]);
        set(gca,'xtick',[current_time-1 current_time-22/24 current_time-20/24 current_time-18/24 current_time-16/24 ...
            current_time-14/24 current_time-12/24 current_time-10/24 current_time-8/24 current_time-6/24 ...
            current_time-4/24 current_time-2/24 current_time],...
            'xticklabel',{'0','2','4','6','8','10','12','14','16','18','20','22','24'});
    end

% Open Message txt file
    function button_msg_Callback(source,eventdata)
        % Display mesh plot of the currently selected data.
        Cage_num = get(source,'Tag');
        Mice = get(handles.hedit_mice(str2double(Cage_num)),'String');
        eval(['!notepad ','./Data/Cage',Cage_num,'/',Mice,'/msg.txt &']); % return immediately with '&'
    end

% move Motor button callback
    function button_moveMotor(hObject, eventdata, arg)
        value = get(handles.hpopup_Cage,'Value');
        str = get(handles.hpopup_Cage,'String');
        if strcmp(str{value},'Chose a Cage')
            msgbox('Please chose a cage to control');
            return;
        else
            Cage_num = str2double(str{value});
        end
        if strcmp(get(handles.hbutton_open(Cage_num),'String'), 'open')
            msgbox(['The COM port of Cage ', num2str(Cage_num), ' is CLOSED!']);
            return;
        end
        switch arg
            case 'FB'
                fwrite(handles.s{Cage_num},'F'); % Command
                fwrite(handles.s{Cage_num},uint8(str2double(get(handles.hedit_motorFB,'String')))); % Value
            case 'LR'
                fwrite(handles.s{Cage_num},'L'); % Command
                fwrite(handles.s{Cage_num},uint8(str2double(get(handles.hedit_motorLR,'String')))); % Value
            case 'Pole'
                fwrite(handles.s{Cage_num},'P'); % Command
                fwrite(handles.s{Cage_num},uint8(str2double(get(handles.hedit_motorPole,'String')))); % Value
            otherwise
                
        end
    end

% save and load button callbacks
    function button_saveParas_Callback(hObject, eventdata)
        for i = 1:handles.total_cage_num
            para(i).com_chose = handles.db_table{i}.COM;
            para(i).mouse_name = handles.db_table{i}.Mouse;
            para(i).startDate = handles.db_table{i}.startDate;
        end
        uisave('para','./para.mat')
    end
    function button_loadParas_Callback(hObject, eventdata)
        [file,path] = uigetfile;
        if ~isequal(file,0)
            file_loaded = load([path file]);
            for i = 1:handles.total_cage_num
                str = get(handles.hpopup(i),'String');
                index = find(strcmp(str, file_loaded.para(i).com_chose));
                if ~isempty(index)
                    set(handles.hpopup(i), 'Value', index);
                    handles.db_table{i}.COM = str{index};
                end
                
                set(handles.hedit_mice(i), 'String', file_loaded.para(i).mouse_name);
                handles.db_table{i}.Mouse = file_loaded.para(i).mouse_name;
                if ~exist(['./Data/Cage',int2str(i),'/',file_loaded.para(i).mouse_name], 'dir')
                    mkdir(['./Data/Cage',int2str(i),'/',file_loaded.para(i).mouse_name]);
                end
                
                if ~isnan(file_loaded.para(i).startDate)
                    set(handles.hedit_date(i),'String',datestr(file_loaded.para(i).startDate,'dd-mmm'));
                    handles.db_table{i}.startDate = file_loaded.para(i).startDate;
                    handles.matObj{i}.startDate = file_loaded.para(i).startDate;
                end
            end
        end
    end

    function button_deleteSel_Callback(hObject, eventdata)
        
        selectedId = get(handles.hlistbox, 'Value');        % get id of selectedLabelName
        existingItems = get(handles.hlistbox, 'String');    % get current listbox list
        
        % Identify the items: if in the list only one ites has been added the
        % returned list is a char array
        if(class(existingItems) == 'char')
            upd_list = '';
            set(handles.hlistbox, 'String', upd_list)
        else
            % If the returned list is a cell array there are three cases
            n_items = length(existingItems);
            if(selectedId == 1)
                % The first element has been selected
                upd_list={existingItems{2:end}};
            elseif(selectedId == n_items)
                % The last element has been selected
                upd_list={existingItems{1:end-1}};
                % Set the "Value" property to the previous element
                set(handles.hlistbox, 'Value', selectedId-1);
            else
                % And element in the list has been selected
                upd_list={existingItems{1:selectedId-1} existingItems{selectedId+1:end}};
            end
        end
        % Update the list
        set(handles.hlistbox, 'String', upd_list);     % restore cropped version of label list
        
        %         % Disable the delete pushbutton if there are no more items
        %         existingItems = get(handles.hlistbox, 'String');
        %         if(isempty(existingItems))
        %             handles.hbutton_deleteSel.Enable='off';
        %         end
    end

    function button_clearAll_Callback(hObject, eventdata)
        set(handles.hlistbox, 'String', '');
    end
%% Serial Callback
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  Serial Callback   %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function serial_callback(obj,event)
        % determine which serial port and which cage
        obj_name = get(obj, 'Name'); % 'Serial-COM7'
        com_port = obj_name(8:end);
        for i = 1:handles.total_cage_num
            if strcmp(get(handles.hbutton_open(i),'String'), 'close') && strcmp(handles.db_table{i}.COM,com_port)
                break;
            end
        end
        % i will be the Cage Num
        
        % read data and store in Cage i folder
        A = fscanf(obj);
        switch A(1)
            case 'T' % trial number  perf 100   protocol-trialNum-perf30k
                try
                    ind_colon = find(A == ':');
                    ind_semicolon = find(A == ';');
                    
                    [mRow,~] = size(handles.matObj{i},'trial_time');
                    handles.matObj{i}.trial_time(mRow+1,1:2) = [str2double(A(ind_colon(1)+1:ind_semicolon(1)-1)), now];
                    set(handles.htext_trailNum(i), 'String', A(ind_colon(1)+1:ind_semicolon(1)-1), 'Fontsize', 8+(3*mod(str2double(A(ind_colon(1)+1:ind_semicolon(1)-1)),2)));
                    
                    set(handles.htext_perf100(i), 'String', A(ind_colon(2)+1:ind_semicolon(2)-1));
                    
                    set(handles.htext_Protocol(i), 'String', A(ind_colon(3)+1:ind_semicolon(3)-1));
                    
                    % fseek(handles.fileID(i),0,'bof');
                    fprintf(handles.fileID(i),[A(ind_colon(1)-9:ind_semicolon(1)+1) A(ind_colon(2)-7:ind_semicolon(2)-2) '; Protocol:' A(ind_colon(3)+1:ind_semicolon(3)-2) A(end-1:end)]);
                catch e
                    disp('Serial Callback-Trial');
                    disp(e);
                end
                
            case 'P' % protocol Outcomes ...
                try
                    % Protocol:P_DELAY; Trial type:Right; Outcome:2; Reward No.:9170
                    ind_colon = find(A == ':');
                    ind_semicolon = find(A == ';');
                    if strcmp(A(ind_colon(2)+1:ind_semicolon(2)-1),'Right') || strcmp(A(ind_colon(2)+1:ind_semicolon(2)-1),'Either-Right')
                        trial_type = 0;
                    else
                        trial_type = 1;
                    end
                    [mRow,~] = size(handles.matObj{i},'trial_time');
                    handles.matObj{i}.trial_time(mRow,3:4) = [trial_type, str2double(A(ind_colon(3)+1))];
                    
                    % fseek(handles.fileID(i),0,'bof');
                    fprintf(handles.fileID(i),A);
                catch e
                    disp('Serial Callback-Protocol');
                    disp(e);
                end
                
            case 'W' % weight
                try
                    [mRow, ~] = size(handles.matObj{i},'weight');
                    handles.matObj{i}.weight(mRow+1,1) = now;
                    handles.matObj{i}.weight(mRow+1,2:121) = double(uint8(A(4:123)));
                catch e
                    disp('Serial Callback-Weight');
                    disp(e);
                end
                
            case 'E' % error msg
                try
                    existingItems = get(handles.hlistbox, 'String');    % get current listbox list
                    n_items = length(existingItems);
                    existingItems{n_items+1} = ['Cage ',num2str(i),': ',A(4:end-2),' (',datestr(now),')'];
                    set(handles.hlistbox, 'String', existingItems);
                    set(handles.hlistbox, 'Value', n_items+1);
                    fprintf(handles.fileID(i),A(4:end));
                catch e
                    disp('Serial Callback-Error');
                    disp(e);
                end
                
            otherwise
                % fseek(handles.fileID(i),0,'bof');
                fprintf(handles.fileID(i),A(4:end));
        end
    end
%% Timer Callback (Update GUI)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%  Timer Callback (Update GUI)   %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function updateGUI(hObject, eventdata)
        for i = 1:handles.total_cage_num
            if strcmp(get(handles.hbutton_open(i),'String'), 'close')
                % update background color
                try
                    [mRow,~] = size(handles.matObj{i},'trial_time');
                    trialData = handles.matObj{i}.trial_time(1:mRow,1:4);
                    
                    trialData_time = trialData(:,2);
                    current_time = now;
                    [trialData_time_24,~] = find(trialData_time>current_time-1);
                    if ~isempty(trialData_time_24)
                        num_trial_24hr = numel(trialData_time_24);
                        set(handles.htext_trailNum24hr(i),'String',[num2str(num_trial_24hr),' (24hr)']);
                        if num_trial_24hr > 640
                            num_trial_24hr = 640;
                        end
                        set_background_color(i,handles.mymap(ceil(num_trial_24hr/80),:));
                    end
                catch e
                    disp('Update GUI... trial num in 24 hr');
                    disp(e);
                    set_background_color(i,handles.mymap(1,:));
                end
                
                % update weight
                try
                    [mRow,~] = size(handles.matObj{i},'weight');
                    if mRow > 200 % 300 sec data
                        weight_data = handles.matObj{i}.weight(mRow-200:mRow,2:121);
                    else
                        weight_data = handles.matObj{i}.weight(1:mRow,2:121);
                    end
                    [mRow2, ~] = size(weight_data); % 120 mCol
                    weight_data2 = zeros(mRow2, 30);
                    for k = 1:mRow2
                        weight_data2(k,:) = typecast(uint8(weight_data(k,:)), 'single');
                    end
                    [N,edges] = histcounts(weight_data2(:),50);
                    ind = find(N==max(N));
                    set(handles.htext_weight2(i), 'String', num2str(edges(ind(1))));
                catch e
                    disp('Update GUI... weight');
                    disp(e);
                end
                
                % update days
                set(handles.htext_days(i),'String',sprintf('%2.1f days',now - handles.db_table{i}.startDate));
            end
        end
    end
%% Close Window Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  Close Window Callbacks   %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function closeGUI_Callback(hObject, eventdata)
        % handles = guidata(hObject);
        selection = questdlg('Close This Figure?',...
            'Close Request Function',...
            'Yes','No','No');
        switch selection
            case 'Yes'
                disp('HomeCage GUI window closing...');
                stop(handles.update_timer); % stop timely update
                for i = 1:handles.total_cage_num
                    if strcmp(get(handles.hbutton_open(i),'String'), 'close')
                        try
                            fclose(handles.fileID(i)); % close file object
                            fclose(handles.s{i});      % close serial port
                            delete(handles.s{i});
                        catch e
                            disp(e);
                        end
                    end
                end
                delete(hObject);
            case 'No'
                return
        end
       
    end

%% Other Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Other Functions   %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function set_background_color(ind,color)
        set(handles.hsubplot(ind),'Color',color);
        set(handles.hpopup(ind),'BackgroundColor',color);
        set(handles.hbutton_open(ind),'BackgroundColor',color);
        set(handles.htext_trailNum24hr(ind),'BackgroundColor',color);
        set(handles.hbutton_chooseStartDate(ind),'BackgroundColor',color);
        set(handles.hbutton_msg(ind),'BackgroundColor',color);
        set(handles.hbutton_plotPW(ind),'BackgroundColor',color);
        set(handles.htext_trailNum(ind),'BackgroundColor',color);
        set(handles.htext_hf1(ind),'BackgroundColor',color);
        set(handles.htext_perf100(ind),'BackgroundColor',color);
        % set(handles.htext_percent1(ind),'BackgroundColor',color);
        set(handles.htext_Protocol(ind),'BackgroundColor',color);
        % set(handles.htext_hf2(ind),'BackgroundColor',color);
        % set(handles.htext_trialNumProtocol(ind),'BackgroundColor',color);
        % set(handles.htext_hf3(ind),'BackgroundColor',color);
        % set(handles.htext_perf30(ind),'BackgroundColor',color);
        % set(handles.htext_percent2(ind),'BackgroundColor',color);
        set(handles.htext_weight1(ind),'BackgroundColor',color);
        set(handles.htext_weight2(ind),'BackgroundColor',color);
        set(handles.htext_g(ind),'BackgroundColor',color);
    end

end