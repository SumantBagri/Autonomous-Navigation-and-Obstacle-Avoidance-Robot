function [data] = read_arduino(Port_obj)
data=nan;

%% IF THERE IS NO DATA?
if (get(Port_obj, 'BytesAvailable')==0)
    disp('Data not avail yet.   Try again or check arduino.')
    return
end
 
%% IF THERE IS DATA
 try
    % read until terminator
    data = fscanf(Port_obj, '%s');
 
    
  % or the sentence wasn't long enough to fill in lat and lon

 
  
catch ERR_MSG
      % if something didn't work correctly the error message displays
      disp('Error Reading Data! Check Unit')
    
end
 
end
