function close_port(comPortNumber)
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
end

