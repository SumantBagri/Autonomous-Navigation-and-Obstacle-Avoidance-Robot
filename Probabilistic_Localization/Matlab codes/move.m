function pnew = move(p, heading)
    K = [0.1 0.1 0.1; 0.1 0.8 0.1; 0.1 0.1 0.1];
    if(heading==-1)
        %First run
        pnew=p;
        return
    end
    global mask
    k1=1/25;
    k2=1-k1;
    heading_mask=p;
    %heading_mask=heading_mask(:,:,1)+heading_mask(:,:,2)+heading_mask(:,:,3)+heading_mask(:,:,4);
    pnew=ones(size(p));
    for i=1:1:4
        p(:,:,i) = conv2(p(:,:,i), K,'same');
        if(heading==1)
            heading_mask(:,:,i)=[0.1+zeros(7,1) p(:,1:end-1,i)];
        elseif (heading==2)
            heading_mask(:,:,i)=[p(2:end,:,i); 0.1+zeros(1,15)];
        elseif (heading==3)
            heading_mask(:,:,i)=[p(:,2:end,i) 0.1+zeros(7,1)];
        else
            heading_mask(:,:,i)=[0.1+zeros(1,15);p(1:end-1,:,i)];
        end
        pnew(:,:,i)=k2*p(:,:,i)+k1*heading_mask(:,:,i);
    end
   %masking matrix
   pnew = pnew.*mask;

   %normalization
   pnew = pnew./sum(sum(sum(pnew)));
end
