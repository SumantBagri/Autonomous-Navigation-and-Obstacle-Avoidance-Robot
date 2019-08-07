function [port_obj] = initializearduino(comPortNumber)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
port=strcat('COM',num2str(comPortNumber));
out=instrfind('Port',port);
if (~isempty(out))
   disp('WARNING:  port in use.  Closing.')
   if (~strcmp(get(out(1), 'Status'),'open'))  % Is it open?
        delete(out(1)); % If not, delete
    else  % is open
        fclose(out(1));
        delete(out(1)); 
   end
end
port_obj=serial(port);
set(port_obj,'BaudRate', 115200);
%set(port_obj,'TimeOut',1);
%set(port_obj, 'OutputBufferSize', floor(389*1.5) );
%set(port_obj, 'InputBufferSize',  floor(389*1.5) );
fopen(port_obj);
pause(3);
disp('Arduino Initialized')
