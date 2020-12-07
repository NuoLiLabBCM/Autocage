function homecage_gui(varargin)
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
%       Created on                 09/15/2018   by Y.H.
%       Last modified on           09/06/2020   by Y.H.
%       modified to ver1.0 on      11/30/2020

%% GUI preparation
global handles
handles.total_cage_num = 30;
handles.default_color = [.7 .9 .8];  % default background color [.93 .93 .93];
handles.trial_24 = [];

hsv_map = hsv(24);
handles.mymap = hsv_map(1:8,:); % 1:8 from red to yellow to green

handles.hfigure = figure('name','HomeCage GUI', 'numbertitle','off', 'MenuBar','none',...
    'CloseRequestFcn', @closeGUI_Callback, 'Position', get(0,'Screensize'));
for cage_i = 1:handles.total_cage_num
    handles.hsubplot(cage_i) = subplot(4,8,cage_i);
    box on;
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    
    % Current supplot panel posititon
    pos(cage_i,:) = get(handles.hsubplot(cage_i), 'Position'); 
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
        'Position',[pos(cage_i,1)+0.057,pos(cage_i,2)+0.16,0.03,0.015]);
    
    % Popup for chosing COM port
    if isempty(serialportlist)
    handles.hpopup(cage_i) = uicontrol('Style','popupmenu',...
        'String',{'Choose a port'},... 
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@popup_com_Callback,...
        'ButtonDownFcn',@popup_com_Buttondown,...
        'Position',[pos(cage_i,1)+0.005,pos(cage_i,2)+0.102,0.032,0.03],...
        'TooltipString', 'Right-click to update COM list');
    else
        handles.hpopup(cage_i) = uicontrol('Style','popupmenu',...
        'String',[{'Choose a port'} serialportlist],... 
        'Units', 'normalized',        'Tag', num2str(cage_i),...
        'Callback',@popup_com_Callback,...
        'ButtonDownFcn',@popup_com_Buttondown,...
        'Position',[pos(cage_i,1)+0.005,pos(cage_i,2)+0.102,0.032,0.03],...
        'TooltipString', 'Right-click to update COM list');
    end
    
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
    
    % Protocol info: currentProtocol-trialsNumsinThisProtocol-Perf30
    handles.htext_Protocol(cage_i) = uicontrol('style','text',...
        'String', 'iProt - iTrial - 00%',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.003,pos(cage_i,2)+0.025,0.048,0.02]);
    
    handles.htext_early(cage_i) = uicontrol('style','text',...
        'String', 'EL:',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.051,pos(cage_i,2)+0.025,0.01,0.02]);
    
    handles.htext_earlylick(cage_i) = uicontrol('style','text',...
        'String', '00%',        'Units', 'normalized',...
        'Position',[pos(cage_i,1)+0.061,pos(cage_i,2)+0.025,0.015,0.02]);
    
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
    'Position',[0.01,0.77,0.1,0.2]);

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

% control panel
handles.hpanel = uipanel('Title','Control Panel',...
    'Position',[.005 .5 .12 .35]);

% Popup for chosing Which Cage
handles.htext_whichCage = uicontrol('Style','text','String','Cage: ',...
    'Units', 'normalized',        'Tag', 'text_whichcage',...
    'Position',[0.01,0.805,0.02,0.02]);
handles.hpopup_Cage = uicontrol('Style','popupmenu',...
    'String',[{'Choose a Cage'} ; 1:handles.total_cage_num],...
    'Units', 'normalized',        'Tag', 'popup_whichcage',...
    'Position',[0.03,0.81,0.04,0.02],...
    'Callback',{@button_moveSet,'Read'},...
    'TooltipString', 'Which Cage to Control');
handles.hbutton_readParas = uicontrol('Style','pushbutton','String','Read',...
    'Units', 'normalized',        'Tag', 'button_read',...
    'Callback',{@button_moveSet,'Read'},...
    'Position',[0.075,0.805,0.04,0.025]);

% motor FB
handles.htext_motorFB = uicontrol('Style','text','String','MotorFB',...
    'Units', 'normalized',        'Tag', 'text_motorFB',...
    'Position',[0.01,0.765,0.03,0.02]);
handles.hedit_motorFB = uicontrol('Style','edit','String','0',...
    'Units', 'normalized',        'Tag', 'edit_motorFB',...
    'Position',[0.04,0.77,0.025,0.02]);
handles.hbutton_motorFB = uicontrol('Style','pushbutton','String','Move & Set',...
    'Units', 'normalized',        'Tag', 'button_motorFB',...
    'Callback',{@button_moveSet,'FB'},...
    'Position',[0.07,0.77,0.04,0.02]);

% motor LR
handles.htext_motorLR = uicontrol('Style','text','String','MotorLR',...
    'Units', 'normalized',        'Tag', 'text_motorLR',...
    'Position',[0.01,0.74,0.03,0.02]);
handles.hedit_motorLR = uicontrol('Style','edit','String','0',...
    'Units', 'normalized',        'Tag', 'edit_motorLR',...
    'Position',[0.04,0.745,0.025,0.02]);
handles.hbutton_motorLR = uicontrol('Style','pushbutton','String','Move & Set',...
    'Units', 'normalized',        'Tag', 'button_motorLR',...
    'Callback',{@button_moveSet,'LR'},...
    'Position',[0.07,0.745,0.04,0.02]);

% motor Pole
handles.htext_motorPole = uicontrol('Style','text','String','MotorPole',...
    'Units', 'normalized',        'Tag', 'text_motorPole',...
    'Position',[0.01,0.715,0.03,0.02]);
handles.hedit_motorPole = uicontrol('Style','edit','String','0',...
    'Units', 'normalized',        'Tag', 'edit_motorPole',...
    'Position',[0.04,0.72,0.025,0.02]);
handles.hbutton_motorPole = uicontrol('Style','pushbutton','String','Move',...
    'Units', 'normalized',        'Tag', 'button_motorPole',...
    'Callback',{@button_moveSet,'Pole'},...
    'Position',[0.07,0.72,0.04,0.02]);

% motor FB final
handles.htext_finalFB = uicontrol('Style','text','String','FinalFB',...
    'Units', 'normalized',        'Tag', 'text_finalFB',...
    'Position',[0.01,0.69,0.03,0.02]);
handles.hedit_finalFB = uicontrol('Style','edit','String','0',...
    'Units', 'normalized',        'Tag', 'edit_finalFB',...
    'Position',[0.04,0.695,0.025,0.02]);
handles.hbutton_finalFB = uicontrol('Style','pushbutton','String','Set',...
    'Units', 'normalized',        'Tag', 'button_finalFB',...
    'Callback',{@button_moveSet,'Final'},...
    'Position',[0.07,0.695,0.04,0.02]);

% Reward left right
handles.htext_rewardLeft = uicontrol('Style','text','String','Left (s)',...
    'Units', 'normalized',        'Tag', 'text_rewardLeft',...
    'Position',[0.01,0.66,0.03,0.02]);
handles.hedit_rewardLeft = uicontrol('Style','edit','String','0.03',...
    'Units', 'normalized',        'Tag', 'edit_rewardLeft',...
    'Position',[0.04,0.665,0.025,0.02]);
handles.htext_rewardRight = uicontrol('Style','text','String','Right (s)',...
    'Units', 'normalized',        'Tag', 'text_rewardRight',...
    'Position',[0.01,0.635,0.03,0.02]);
handles.hedit_rewardRight = uicontrol('Style','edit','String','0.03',...
    'Units', 'normalized',        'Tag', 'edit_rewardRight',...
    'Position',[0.04,0.64,0.025,0.02]);

handles.hbutton_reward = uicontrol('Style','pushbutton','String','Set&Reward',...
    'Units', 'normalized',        'Tag', 'button_reward',...
    'Callback',{@button_moveSet,'Reward'},...
    'Position',[0.07,0.6525,0.04,0.02]);

% weight
handles.htext_tare = uicontrol('Style','text','String','Weight (s)',...
    'Units', 'normalized',        'Tag', 'text_tare',...
    'Position',[0.01,0.605,0.03,0.02]);
handles.hedit_tare = uicontrol('Style','edit','String','0',...
    'Units', 'normalized',        'Tag', 'edit_tare',...
    'Position',[0.04,0.61,0.025,0.02]);
handles.hbutton_tare = uicontrol('Style','pushbutton','String','Tare',...
    'Units', 'normalized',        'Tag', 'button_tare',...
    'Callback',{@button_moveSet,'Tare'},...
    'Position',[0.07,0.61,0.04,0.02]);

% Struggle threshold H and L
handles.htext_struggleH = uicontrol('Style','text','String','Struggle Thres (g)',...
    'Units', 'normalized',        'Tag', 'text_struggleH',...
    'Position',[0.01,0.55,0.032,0.04]);
handles.hedit_struggleH = uicontrol('Style','edit','String','36',...
    'Units', 'normalized',        'Tag', 'edit_struggleH',...
    'Position',[0.04,0.58,0.025,0.02]);
handles.hedit_struggleL = uicontrol('Style','edit','String','-1',...
    'Units', 'normalized',        'Tag', 'edit_struggleL',...
    'Position',[0.04,0.555,0.025,0.02]);
handles.hbutton_struggleSet = uicontrol('Style','pushbutton','String','Set',...
    'Units', 'normalized',        'Tag', 'button_struggleSet',...
    'Callback',{@button_moveSet,'Struggle'},...
    'Position',[0.07,0.5675,0.04,0.02]);

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
TimerInteval = 600;       % Default timer interval (10 min)
handles.update_timer = timer('Name','MainTimer', 'TimerFcn',{@updateGUI},...
    'Period',TimerInteval,'ExecutionMode','fixedRate');
start(handles.update_timer);

SerialTimerInterval = 1; % every 1 second
handles.serial_update_timer = timer('Name','serialTimer','TimerFcn',{@serialTimer},...
    'Period',SerialTimerInterval,'ExecutionMode','fixedRate');
start(handles.serial_update_timer);


%% Callback Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% Callback Functions %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Mice Name
    function edit_mice_Callback(source,~)
        % Create a folder for the mice under the dirctory of that Cage
        Cage = get(source,'Tag');
        Mice = get(source,'String');
        if ~exist(['./Data/Cage',Cage,'/',Mice], 'dir')
            mkdir(['./Data/Cage',Cage,'/',Mice]);
        end
        handles.db_table{str2double(Cage)}.Mouse = Mice;
    end

% Popup Menu: left click to chose a port
    function popup_com_Callback(source,~)
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
    function popup_com_Buttondown(source,~)
        Cage = str2double(get(source,'Tag'));
        set(handles.hpopup(Cage),'String',[{'Choose a port'} serialportlist]);
    end

% Open a COM port
    function button_open_Callback(source,~)
        % Display mesh plot of the currently selected data.
        % handles = guidata(source);
        Cage = get(source,'Tag');
        Cage_num = str2double(Cage);
        if strcmp(get(source,'String'), 'open')
            % open com port
            str_com = handles.db_table{Cage_num}.COM;
            if strcmp(str_com(1:3), 'COM')
                openOK=1;
                for i_cage = 1:handles.total_cage_num
                    if (i_cage ~= Cage_num) && strcmp(get(handles.hbutton_open(i_cage),'String'), 'close') && strcmp(handles.db_table{i_cage}.COM,str_com)
                        msgbox([str_com ' is being used. Please choose another COM port']);
                        openOK=0;
                        break;
                    end
                end
                
                if openOK
                    try
                        handles.s{Cage_num}=serialport(str_com, 115200);
                        configureTerminator(handles.s{Cage_num},"CR/LF");
                        configureCallback(handles.s{Cage_num},"terminator",@serial_callback);
                    catch e
                        disp('error in Open Serial Port...');
                        disp(e);
                        msgbox('Serial port open failed.');
                        return;
                    end
                    set(source,'String','close');
                    
                    Mice = get(handles.hedit_mice(Cage_num),'String');
                    
                    if exist(['./Data/Cage',Cage,'/',Mice,'/allmat.mat'], 'file') == 2
                        eval(['delete ','./Data/Cage',Cage,'/',Mice,'/allmat.mat']);
                    end
                    if exist(['./Data/Cage',Cage,'/',Mice,'/msg.txt'], 'file') == 2
                        eval(['delete ','./Data/Cage',Cage,'/',Mice,'/msg.txt']);
                    end
                    
                    handles.fileID(Cage_num) = fopen(['./Data/Cage',Cage,'/',Mice,'/msg.txt'],'a');
                    
                    % trial-time and weight mat file to record
                    handles.matObj{Cage_num} = matfile(['Data/Cage',Cage,'/',Mice,'/allmat.mat'],'Writable',true);
                    handles.matObj{Cage_num}.weight = zeros(1,41);% 1st col: datetime; 2:161 col: weight data in float (4 bytes/data)
                                     
                    handles.localdata{Cage_num}.startDate = handles.db_table{Cage_num}.startDate; 
                    handles.localdata{Cage_num}.tRow = 1;
                    handles.localdata{Cage_num}.trial_time = zeros(2000,5);  
                    handles.localdata{Cage_num}.wRow = 0;
                    handles.localdata{Cage_num}.weight = zeros(200,41);
                    handles.localdata{Cage_num}.ind=1;
                    
                  % update background color
                  try
                      set(handles.htext_trailNum24hr(Cage_num),'String',[num2str(0),' (24hr)']);
                      set_background_color(Cage_num,handles.mymap(1,:));                   
                  catch e
                      disp('Update GUI... trial num in 24 hr');
                      disp(e);
                      set_background_color(Cage_num,handles.mymap(1,:));
                  end
                  
                    % update days
                    set(handles.htext_days(Cage_num),'String',sprintf('%2.1f days',now - handles.db_table{Cage_num}.startDate));
                end
            else
                msgbox('Please choose a COM port first!');
            end
        else
            % close com port
            try
                fclose(handles.fileID(Cage_num));
                delete(handles.s{Cage_num});
                Mice = get(handles.hedit_mice(Cage_num),'String');
                if exist(['./Data/Cage',Cage,'/',Mice,'/allmat.mat'], 'file') == 2
                    eval(['delete ','./Data/Cage',Cage,'/',Mice,'/allmat.mat']);
                end
                if exist(['./Data/Cage',Cage,'/',Mice,'/msg.txt'], 'file') == 2
                    eval(['delete ','./Data/Cage',Cage,'/',Mice,'/msg.txt']);
                end
            catch e
                disp('Close a Serial Port...');
                disp(e);
            end
            set(source,'String','open');
            set(handles.htext_trailNum(Cage_num), 'String', 'Trial No.');
            set(handles.htext_perf100(Cage_num), 'String', '0%');
            set(handles.htext_Protocol(Cage_num), 'String', 'iProt - iTrial - 00%');
            set_background_color(Cage_num,handles.default_color); % set default background color
        end
    end

% set start Date button
    function button_chooseStartDate_Callback(source,~)
        Cage_num = get(source,'Tag');
        D = uigetdate();
        try
            set(handles.hedit_date(str2double(Cage_num)),'String',datestr(D,'dd-mmm'));
            handles.db_table{str2double(Cage_num)}.startDate = D;
        catch e
            disp('Choose Start Date!');
            disp(e);
        end
    end

% Plot buttons
    function button_plotPW_Callback(source,~)
        Cage_num = str2double(get(source,'Tag'));
        if strcmp(get(handles.hbutton_open(Cage_num),'String'), 'open')
            msgbox(['The COM port of Cage ', num2str(Cage_num), ' is CLOSED!']);
            return;
        else
            selection = questdlg('Do you want to pull data from SD card (need some time) or plot data stored in Matlab?',...
                'Choose',...
                'Pull SD','Plot data','Plot data');
            switch selection
                case 'Pull SD'
                    write(handles.s{Cage_num},'A', 'char'); % Command
                case 'Plot data'
                    % plot trial history data in last 24 hr (below)
                    figure,
                    subplot(3,1,1),hold on,
                    
                    %[mRow,~] = size(handles.matObj{Cage_num},'trial_time'); % trial num, datetime,trial type, outcome
                    mRow = handles.localdata{Cage_num}.tRow;
                    trialData = handles.localdata{Cage_num}.trial_time(1:mRow,1:4);
                    trialData_time = trialData(:,2);
                    current_time = now;
                    [trialData_time_24,~] = find(trialData_time>current_time-1);
                   
                    if ~isempty(trialData_time_24)
                        num_trial_24hr = numel(trialData_time_24);
                        for i = trialData_time_24(1):trialData_time_24(end)
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
                    else
                        title('No trials in last 24-hour');
                    end
                    
                    clear trialData trialData_time trialData_time_24;                  
                    
                    % plot weight data in last 24 hours
                    subplot(3,1,2),hold on,
                    %[mRow,~] = size(handles.matObj{Cage_num},'weight');
                    mRow = handles.localdata{Cage_num}.wRow;
                    if mRow > 0
                        weight_date = handles.matObj{Cage_num}.weight(1:mRow,1);
                        [weight_date_24,~] = find(weight_date>current_time-1);
                        if ~isempty(weight_date_24)
                            weight = handles.matObj{Cage_num}.weight(weight_date_24(1):mRow,1:41);
                            [mRow2,~] = size(weight);
                            weight_data = zeros(40,mRow2);
                            for i = 1:mRow2
                                weight_data(:,i) = (weight(i,2:41))';%typecast(uint8(weight(i,2:161)), 'single')';
                            end
                            plot(weight(:,1),mean(weight_data),'k.');
                            [N,edges] = histcounts(weight_data(:),100);
                            ind = find(N==max(N));
                            weight_24 = edges(ind(1));
                        end
                        title(['Avg. ', num2str(weight_24), ' g in last 24-hr']);
                        xlim([current_time-1 current_time]);
                        ylabel('weight (g)'); xlabel('Time (hour)'); ylim([15 30]);
                        set(gca,'xtick',[current_time-1 current_time-22/24 current_time-20/24 current_time-18/24 current_time-16/24 ...
                            current_time-14/24 current_time-12/24 current_time-10/24 current_time-8/24 current_time-6/24 ...
                            current_time-4/24 current_time-2/24 current_time],...
                            'xticklabel',{'0','2','4','6','8','10','12','14','16','18','20','22','24'});
                    else
                        title('No Avg. weight');
                    end
                    clear weight_date weight_date_24 weight weight_data;
            end
        end
    end

% Open Message txt file
    function button_msg_Callback(source,~)
        Cage_num = get(source,'Tag');
        if strcmp(get(handles.hbutton_open(str2double(Cage_num)),'String'), 'open')
            msgbox(['The COM port of Cage ', Cage_num, ' is CLOSED!']);
            return;
        else
            Mice = get(handles.hedit_mice(str2double(Cage_num)),'String');
            eval(['!notepad ','./Data/Cage',Cage_num,'/',Mice,'/msg.txt &']); % return immediately with '&'
        end
    end

% move Motor button callback
    function button_moveSet(~, ~, arg)
        value = get(handles.hpopup_Cage,'Value');
        str = get(handles.hpopup_Cage,'String');
        if strcmp(str{value},'Choose a Cage')
            msgbox('Please choose a cage to control');
            return;
        else
            Cage_num = str2double(str{value});
        end
        if strcmp(get(handles.hbutton_open(Cage_num),'String'), 'open')
            msgbox(['The COM port of Cage ', num2str(Cage_num), ' is CLOSED!']);
            return;
        end
        switch arg
            case 'Read'
                write(handles.s{Cage_num},'C','char'); % Command
            case 'FB'
                if str2double(get(handles.hedit_motorFB,'String')) > 100
                    selection = questdlg('Make sure the lickports are higher than the headport floor!',...
                        'Warning',...
                        'Yes','No','No');
                    switch selection
                        case 'Yes'
                            write(handles.s{Cage_num},'F', 'char'); % Command
                            write(handles.s{Cage_num},str2double(get(handles.hedit_motorFB,'String')),'uint8'); % Value
                        case 'No'
                            return;
                    end
                else
                    write(handles.s{Cage_num},'F','char'); % Command
                    write(handles.s{Cage_num},str2double(get(handles.hedit_motorFB,'String')),'uint8'); % Value
                end
            case 'LR'
                write(handles.s{Cage_num},'L','char'); % Command
                write(handles.s{Cage_num},str2double(get(handles.hedit_motorLR,'String')),'uint8'); % Value
            case 'Pole'
                write(handles.s{Cage_num},'P','char'); % Command
                write(handles.s{Cage_num},str2double(get(handles.hedit_motorPole,'String')),'uint8'); % Value
            case 'Final'
                write(handles.s{Cage_num},'f','char'); % Command
                write(handles.s{Cage_num},str2double(get(handles.hedit_finalFB,'String')),'uint8'); % Value
                msgbox('Done!');
            case 'Reward'
                write(handles.s{Cage_num},'R','char'); % Command
                writeline(handles.s{Cage_num},get(handles.hedit_rewardLeft,'String'));
                writeline(handles.s{Cage_num},get(handles.hedit_rewardRight,'String'));
            case 'Tare'
                selection = questdlg('Make sure the mouse is not on the plateform!',...
                    'Warning',...
                    'Yes','No','No');
                switch selection
                    case 'Yes'
                        write(handles.s{Cage_num},'T','char'); % Command
                        msgbox('Done!');
                    case 'No'
                        return;
                end
            case 'Struggle' 
                write(handles.s{Cage_num},'S','char'); % Command
                writeline(handles.s{Cage_num},get(handles.hedit_struggleL,'String'));
                writeline(handles.s{Cage_num},get(handles.hedit_struggleH,'String'));
                msgbox('Done!');
            otherwise
                disp('no this command - button_moveSet');
        end
    end

% save and load button callbacks
    function button_saveParas_Callback(~, ~)
        for i = 1:handles.total_cage_num
            para(i).com_chose = handles.db_table{i}.COM;
            para(i).mouse_name = handles.db_table{i}.Mouse;
            para(i).startDate = handles.db_table{i}.startDate;
        end
        uisave('para','./para.mat')
    end

    function button_loadParas_Callback(~, ~)
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
                end
            end
        end
    end

    function button_deleteSel_Callback(~, ~)
        
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
                upd_list={existingItems(2:end)};
            elseif(selectedId == n_items)
                % The last element has been selected
                upd_list={existingItems(1:end-1)};
                % Set the "Value" property to the previous element
                set(handles.hlistbox, 'Value', selectedId-1);
            else
                % And element in the list has been selected
                upd_list={existingItems{1:selectedId-1} existingItems{selectedId+1:end}};
            end
        end
        % Update the list
        set(handles.hlistbox, 'String', upd_list);     % restore cropped version of label list
    end

    function button_clearAll_Callback(~, ~)
        set(handles.hlistbox, 'String', '');
    end

%% Serial Callback
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  Serial Callback   %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function serial_callback(obj,~)
        % determine which serial port and which cage
        com_port = get(obj, 'Port'); % 'Serial-COM7'
        for i_cage = 1:handles.total_cage_num
            if strcmp(get(handles.hbutton_open(i_cage),'String'), 'close') && strcmp(handles.db_table{i_cage}.COM,com_port)
                break;
            end
        end
        % i_cage will be the Cage Num
        
        % read data and store in Cage i_cage folder
        A = char(readline(obj));
        switch A(1)
            case 'T' % trial number  perf 100   protocol-trialNum-perf30k
                try
                    ind_colon = find(A == ':');
                    ind_semicolon = find(A == ';');
                    
                    %[mRow,~] = size(handles.matObj{i_cage},'trial_time');
                    mRow = handles.localdata{i_cage}.tRow;
                    
                   % handles.matObj{i_cage}.trial_time(mRow+1,1:2) = [str2double(A(ind_colon(1)+1:ind_semicolon(1)-1)), now];
                    handles.localdata{i_cage}.trial_time(mRow,1:2) = [str2double(A(ind_colon(1)+1:ind_semicolon(1)-1)), now];
                    handles.localdata{i_cage}.trial_time(mRow,5) = str2double(A(ind_colon(4)+1));
                    
                    set(handles.htext_trailNum(i_cage), 'String', A(ind_colon(1)+1:ind_semicolon(1)-1), 'Fontsize', 8+(3*mod(str2double(A(ind_colon(1)+1:ind_semicolon(1)-1)),2)));                   
                    set(handles.htext_perf100(i_cage), 'String', A(ind_colon(2)+1:ind_semicolon(2)-1));                    
                    set(handles.htext_Protocol(i_cage), 'String', A(ind_colon(3)+1:ind_semicolon(3)-1));
                    
                    fprintf(handles.fileID(i_cage),[A(ind_colon(1)-9:ind_semicolon(1)+1) A(ind_colon(2)-7:ind_semicolon(2)-2) '; Protocol:' A(ind_colon(3)+1:ind_semicolon(3)-2) '; early:' A(ind_colon(4)+1) '\n']);                 
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
                                                   
                    %[mRow,~] = size(handles.matObj{i_cage},'trial_time');
                    mRow = handles.localdata{i_cage}.tRow;                  
                    handles.localdata{i_cage}.trial_time(mRow,3:4) = [trial_type, str2double(A(ind_colon(3)+1))];
                    if mRow >= 2000
                         mRow=2000;
                         handles.localdata{i_cage}.trial_time(1:mRow-1,:)=handles.localdata{i_cage}.trial_time(2:mRow,:); 
                         handles.localdata{i_cage}.trial_time(mRow,:)=[0 0 0 0 0];  
                    else
                        mRow=mRow+1;
                    end   
                    handles.localdata{i_cage}.tRow = mRow;
                    
                    fprintf(handles.fileID(i_cage),[A '\n']);
                catch e
                    disp('Serial Callback-Protocol');
                    disp(e);
                end
                
            case 'W' % weight 
                try
                    indT=handles.localdata{i_cage}.ind;
                    handles.localdata{i_cage}.weight(indT,1)=now;
                    handles.localdata{i_cage}.weight(indT,2:41)=typecast(uint8(A(4:163)), 'single');                   
                    if indT>=200                      
                        indM = handles.localdata{i_cage}.wRow;
                        handles.matObj{i_cage}.weight(indM+1:indM+indT,:) = handles.localdata{i_cage}.weight;                     
                        handles.localdata{i_cage}.wRow = indM+indT;
                        indT=1;
                    else
                        indT=indT+1;
                    end                    
                    handles.localdata{i_cage}.ind = indT;
                catch e
                    disp('Serial Callback-Weight');
                    disp(size(A));
                    disp(e);
                end
                %disp('Weight-end');
                
            case 'E' % error msg
                try
                    existingItems = get(handles.hlistbox, 'String');    % get current listbox list
                    n_items = length(existingItems);
                    existingItems{n_items+1} = ['Cage ',num2str(i_cage),': ',A(4:end),' (',datestr(now),')'];
                    set(handles.hlistbox, 'String', existingItems);
                    set(handles.hlistbox, 'Value', n_items+1);
                    fprintf(handles.fileID(i_cage),[A(4:end) '\n']);
                catch e
                    disp('Serial Callback-Error');
                    disp(e);
                end
                clear existingItems;
                
            case 'C' % control panel info
                try
                    ind_semicolon = find(A == ';');
                    set(handles.hedit_motorFB, 'String', A(2:ind_semicolon(1)-1));
                    set(handles.hedit_motorLR, 'String', A(ind_semicolon(1)+1:ind_semicolon(2)-1));
                    set(handles.hedit_motorPole, 'String', A(ind_semicolon(2)+1:ind_semicolon(3)-1));
                    set(handles.hedit_finalFB, 'String', A(ind_semicolon(3)+1:ind_semicolon(4)-1));
                    set(handles.hedit_rewardLeft, 'String', A(ind_semicolon(4)+1:ind_semicolon(5)-1));
                    set(handles.hedit_rewardRight, 'String', A(ind_semicolon(5)+1:ind_semicolon(6)-1));
                    set(handles.hedit_tare, 'String', A(ind_semicolon(6)+1:ind_semicolon(7)-1));
                    set(handles.hedit_struggleL, 'String', A(ind_semicolon(7)+1:ind_semicolon(8)-1));
                    set(handles.hedit_struggleH, 'String', A(ind_semicolon(8)+1:ind_semicolon(9)-1));
                catch e
                    disp('Serial Callback - Control panel info');
                    disp(e);
                end
            % 
            case 'A' % receiving trial data for the last 24-hr
                try
                    handles.trial_24(end+1,:) = str2num(A(2:end));
                catch e
                    disp('Serial Callback - receiving 24-hr data');
                    disp(e);
                end
            case 'N' % end of receiving data and plot
                try
                    % plot data in handles.trial_24: col1: timestamp; col2:trial_type; col3: trial_outcome
                    time_now = str2double(A(2:end));
                    if isempty (handles.trial_24)
                        figure, subplot(2,1,1), title('No trials in last 24 hrs');
                    else
                        timestamp = handles.trial_24(:,1);
                        trial_type = handles.trial_24(:,2);
                        trial_outcome = handles.trial_24(:,3);
                        % 'Others'-3 || Reward-1 || No Response-0 || Time Out (error)-2
                        no_response = (trial_outcome == 0);
                        correct = (trial_outcome == 1);
                        error = (trial_outcome == 2);
                        figure, subplot(2,1,1), hold on,
                        plot(timestamp(no_response),trial_type(no_response),'bo');
                        plot(timestamp(correct),trial_type(correct),'g.','markersize',25);
                        plot(timestamp(error),trial_type(error),'r.','markersize',25);
                        xlim([time_now-24*3600 time_now]); ylim([-0.5 1.5]);
                        set(gca,'ytick',[0 1],'yticklabel',{'Right','Left'});
                        set(gca,'xtick',[time_now-24*3600 time_now-18*3600 time_now-12*3600 time_now-6*3600 time_now],'xticklabel',{'0','6','12','18','24'});
                        title([num2str(numel(timestamp)), ' trials in last 24-hour']);
                    end
                    % after plot, reset
                    handles.trial_24 = [];
                catch e
                    disp('Serial Callback - end of receiving 24-hr data and plot');
                    disp(e);
                end
            
            otherwise
                try
                    fprintf(handles.fileID(i_cage),[A(4:end) '\n']);
                catch e
                    disp('Serial Callback - end of otherwise');
                    disp(e);
                end
        end
        clear A;
    end

%% Timer Callback (Update GUI)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%  Timer Callback (Update GUI)   %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function serialTimer(~, ~)     
        for i_cage = 1:handles.total_cage_num
            if strcmp(get(handles.hbutton_open(i_cage),'String'), 'close')
                %disp(['serialTimer' num2str(i_cage) all_serialPorts]);
                try
                    getpinstatus(handles.s{i_cage});
                catch e                  
                    fclose(handles.fileID(i_cage));
                    delete(handles.s{i_cage});
                    Mice = get(handles.hedit_mice(i_cage),'String');
                    if exist(['./Data/Cage',num2str(i_cage),'/',Mice,'/allmat.mat'], 'file') == 2
                        eval(['delete ','./Data/Cage',num2str(i_cage),'/',Mice,'/allmat.mat']);
                    end
                    if exist(['./Data/Cage',num2str(i_cage),'/',Mice,'/msg.txt'], 'file') == 2
                        eval(['delete ','./Data/Cage',num2str(i_cage),'/',Mice,'/msg.txt']);
                    end
                    
                    set(handles.hbutton_open(i_cage),'String','open');
                    set(handles.htext_trailNum(i_cage), 'String', 'Trial No.');
                    set(handles.htext_perf100(i_cage), 'String', '0%');
                    set(handles.htext_Protocol(i_cage), 'String', 'iProt - iTrial - 00%');
                    set_background_color(i_cage,handles.default_color); % set default background color
                    
                    disp('Close a Serial Port - Seiral Timer ...');
                    disp(e);
                end
            end
        end
        %disp('serialTimer-end');
    end

    function updateGUI(~, ~)
        for i_cage = 1:handles.total_cage_num
            if strcmp(get(handles.hbutton_open(i_cage),'String'), 'close')
                % update days
                set(handles.htext_days(i_cage),'String',sprintf('%2.1f days',now - handles.db_table{i_cage}.startDate));
                
                mRow = handles.localdata{i_cage}.tRow;
                % update background color
                try
                    trialData_time = handles.localdata{i_cage}.trial_time(1:mRow,2);
                    current_time = now;
                    [trialData_time_24,~] = find(trialData_time>current_time-1);
                    if ~isempty(trialData_time_24)
                        mRow=trialData_time_24(end);
                        num_trial_24hr = numel(trialData_time_24);
                        set(handles.htext_trailNum24hr(i_cage),'String',[num2str(num_trial_24hr),' (24hr)']);
                        if num_trial_24hr > 640
                            num_trial_24hr = 640;
                        end
                        set_background_color(i_cage,handles.mymap(ceil(num_trial_24hr/80),:));
                    end
                catch e
                    disp('Update GUI... trial num in 24 hr');
                    disp(e);
                    set_background_color(i_cage,handles.mymap(1,:));
                end
                clear trialData_time trialData_time_24;
                
                % update earlylick
                if mRow > 100
                    try
                        earlylick = handles.localdata{i_cage}.trial_time(mRow-99:mRow,5);
                        set(handles.htext_earlylick(i_cage),'String',[num2str(sum(earlylick == 1)),'%']);
                        if earlylick(end) == 2
                            set(handles.htext_earlylick(i_cage),'String','NaN %');
                        end
                        % end
                    catch e
                        disp('Update GUI... earlylick');
                        disp(e);
                    end
                    clear earlylick;
                end
            end
        end
    end

%% Close Window Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  Close Window Callbacks   %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function closeGUI_Callback(hObject, ~)
        % handles = guidata(hObject);
        selection = questdlg('Close This Figure?',...
            'Close Request Function',...
            'Yes','No','No');
        switch selection
            case 'Yes'
                disp('HomeCage GUI window closing...');
                try
                    stop(handles.update_timer); % stop timely update
                    stop(handles.serial_update_timer);
                    delete(timerfind);
                    for i_cage = 1:handles.total_cage_num
                        %disp(['closeGUI' num2str(i_cage)]);
                        if strcmp(get(handles.hbutton_open(i_cage),'String'), 'close')
                            try
                                fclose(handles.fileID(i_cage)); % close file object
                                delete(handles.s{i_cage});  % close serial port
                                Mice = get(handles.hedit_mice(i_cage),'String');
                                if exist(['./Data/Cage',num2str(i_cage),'/',Mice,'/allmat.mat'], 'file') == 2
                                    eval(['delete ','./Data/Cage',num2str(i_cage),'/',Mice,'/allmat.mat']);
                                end
                                if exist(['./Data/Cage',num2str(i_cage),'/',Mice,'/msg.txt'], 'file') == 2
                                    eval(['delete ','./Data/Cage',num2str(i_cage),'/',Mice,'/msg.txt']);
                                end
                            catch e
                                disp(e);
                            end
                        end
                    end
                    delete(hObject);
                catch e
                    disp(e);
                    delete(hObject);
                end
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
        set(handles.htext_Protocol(ind),'BackgroundColor',color);            
        set(handles.htext_early(ind),'BackgroundColor',color);
        set(handles.htext_earlylick(ind),'BackgroundColor',color);
    end

end