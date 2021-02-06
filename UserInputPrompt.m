function [wave_type,targets]=UserInputPrompt()
% Written by Samuel Viegas %
% This function is used so that the user can input the targets initial 
% velocities and positions
while true
    options={'Rectangular Pulse Waveform','LinearFM Pulse Waveform','Load Saved Parameters'};
    [wave_type,tf]=listdlg('PromptString',{'Select an option'},'SelectionMode','Single','ListString',options);
    if tf~=0
        break;
    end
end
if wave_type==3
    % Load saved data
    targets=zeros(2,4);
    x=false;
else
    x=true;
end
while x
    targets_init=inputdlg({'Target 1 distance (0-1500)[m]',...
                         'Target 2 distance (0-1500)[m]',...
                         'Target 3 distance (0-1500)[m]',...
                         'Target 4 distance (0-1500)[m]',...
                         'Target 1 speed (-50-50)',...
                         'Target 2 speed (-50-50)',...
                         'Target 3 speed (-50-50)',...
                         'Target 4 speed (-50-50)'},...
                         'Targets parameters');
    targets(1,1)=str2double(targets_init(1,1));
    targets(1,2)=str2double(targets_init(2,1));
    targets(1,3)=str2double(targets_init(3,1));
    targets(1,4)=str2double(targets_init(4,1));
    targets(2,1)=str2double(targets_init(5,1));
    targets(2,2)=str2double(targets_init(6,1));
    targets(2,3)=str2double(targets_init(7,1));
    targets(2,4)=str2double(targets_init(8,1)); 
    %targets(2,:)=targets(2,:)*(-1);
    if (targets(1,1)>1000||targets(1,2)>1000||targets(1,3)>1000||targets(1,4)>1000||targets(2,1)>50||targets(2,2)>50||targets(2,3)>50||targets(2,4)>50||targets(2,1)<-50||targets(2,2)<-50||targets(2,3)<-50||targets(2,4)<-50)
        disp('Input values exceed the maximum values accepted! Try again!');
    else
        break;
    end
end