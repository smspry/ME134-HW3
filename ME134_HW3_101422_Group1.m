%% HOMEWORK 3
clear all; clc;

% this is the name of the made server page
% it is the ESP32 IP followed by the name given in the server creation
% section
site_comms = "http://192.168.4.1/wifi_comms";  

%% commands to the start and stop of the balancing robot
% prompts user for either a 'start' or 'stop'

while(1)
    
    % prompts the user to input the start and end of the process
    % in the MATLAB command window
    prompt_robot = "Please enter 'start' to start the balancing robot and 'stop' to end the balancing robot";
    txt = input(prompt_robot, "s"); 
    
    % sends the chosen value to the server
    if(txt == "start")
        sending = 1;
        response = webwrite(site_comms, sending);
        fprintf('The robot has began balancing! %s\n',response)
    elseif (txt == "stop")
        sending = 0;
        response = webwrite(site_comms, sending);
        fprintf('The robot has stopped balancing! %s\n',response)
    elseif(txt == "out")
    	break
    end
    
    % to pause the while loop from prompting without stopping
    pause
    
end
