function [bool,flag] = write_heading(s,heading,target)
    global p;
    dir=-1;
    flag=0;
    if max(max(max(p)))>0.8
        temp=p==max(max(max(p)));
        for x=1:1:15
        for y=1:1:7
            for z=1:1:4
                if(temp(y,x,z)==1)
                    dir=z;
                end
                if target(1)==y && target(2)==x
    %                bool=1;
                    flag=1;
                    break

                end
           
            end
            if flag==1
                break
            end
        end
        if flag==1
            break
        end

        end
    bool=1;
    %heading
    %dir
    if flag==0
    temp=mod((heading-dir+1),4)
    fprintf(s,int2str(temp));
    else 
        fprintf(s,int2str(-1));
    end
    else
        bool=0;
    end
   
end