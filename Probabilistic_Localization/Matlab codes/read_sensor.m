function [auto_heading] = read_sensor()
global SenVal s;
while s.bytesAvailable==0
end
while(s.bytesAvailable~=0)
    temp=fscanf(s);
end
 %   if temp(end)~='\n'
  %      temp=temp+fscanf(s);
    i=1;
    temp=strsplit(temp,",");
    if size(temp,2)~=10
        read_sensor;
    else
   %     temp2=splitlines(temp(8));
   % temp(8)=temp2(1);
    while(i<=8)
        if str2double(temp(i))>50.8
            SenVal(i)=1;
        else
            SenVal(i)=0;
        end
        i=i+1;
    end
    auto_heading=str2double(temp(9));
    end
 %   flushinput(s);
end


