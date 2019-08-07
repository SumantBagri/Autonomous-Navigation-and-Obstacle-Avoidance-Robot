function heading = solve_maze(target)
    import java.util.LinkedList
    global mask u bfs_fields
     global p
     p_temp=p(:,:,1)+p(:,:,2)+p(:,:,3)+p(:,:,4);
    temp=p_temp==max(max((p_temp)));
%    curr=[7,4];
    for x=1:1:15
        for y=1:1:7
             if(temp(y,x)==1)
                 curr=[y x];
            end
            bfs_fields(y,x).color="white";
            bfs_fields(y,x).d=-1;
            bfs_fields(y,x).pi=[0,0];
            bfs_fields(y,x).current=[y,x];
        end
    end
    %using target as source
    bfs_fields(target(1),target(2)).color="gray";
    bfs_fields(target(1),target(2)).d=0;
    bfs_fields(target(1),target(2)).pi=-1;
    q=LinkedList();
    q.add([target(1),target(2)]);
    while size(q)>=1
        u=q.remove();
        bfs_field=[];
        adjacency=[u(1)-1 u(2);u(1)+1 u(2); u(1) u(2)-1; u(1) u(2)+1];
        for i=1:1:4
            if adjacency(i,1)>0 && adjacency(i,1)<8 && adjacency(i,2)>0 && adjacency(i,2)<16
                if mask(adjacency(i,1), adjacency(i,2))==1
                    bfs_field=[bfs_field [adjacency(i,1);adjacency(i,2)]];
                end
            end
        end
        for v=bfs_field
            if bfs_fields(v(1),v(2)).color=="white"
                bfs_fields(v(1),v(2)).color="gray";
                bfs_fields(v(1),v(2)).d=bfs_fields(u(1),u(2)).d+1;
                bfs_fields(v(1),v(2)).pi=u;
                q.add([v(1),v(2)]);
            end
        end
        bfs_fields(u(1),u(2)).color="black";
    end
    a=bfs_fields(curr(1),curr(2));
    tempa=a.pi;
    tempb=tempa;
    while tempa(1)~=target(1) || tempa(2)~=target(2)
        temp_index=bfs_fields(tempa(1),tempa(2)).pi;
        tempa=[temp_index(1),temp_index(2)];
    end
    if tempb(1)<curr(1)
        heading=2;
    elseif tempb(1)>curr(1)
        heading=4;
    elseif tempb(2)<curr(2)
        heading=3;
    elseif tempb(2)>curr(2)
        heading=1;
    end
end
