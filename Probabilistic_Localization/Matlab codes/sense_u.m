function pnew = sense_u(p, SenVal)
  global world mask p_fig mult_fig bool_check
  %models sensor reading
  pHit = 0.6; %default = 0.9
  pMiss = 0.2; %default = 0.1
  if bool_check==1
    p_fig=figure;
    mult_fig=figure;
    bool_check = 0;
  end
  mult=zeros(7,15,4);
  for x=1:1:15
      for y=1:1:7
          for z=1:1:4
              if(world(z,:,y,x)==SenVal)
                  mult(y,x,z)=pHit;
              else
                  mult(y,x,z)=pMiss;
              end
          end
      end
  end
  p(:,:,1) = p(:,:,1).*mask;
  p(:,:,2) = p(:,:,2).*mask;
  p(:,:,3) = p(:,:,3).*mask;
  %3D multiplication step
  p = p.*mult;
  %normalization
  p = p./sum(sum(sum(p)));
  figure(p_fig);
  pnew1=p(:,:,1)+p(:,:,2)+p(:,:,3)+p(:,:,4);
  imagesc(pnew1);
  title("Probability Map");
  mult1=mult(:,:,1)+mult(:,:,2)+mult(:,:,3)+mult(:,:,4);
  figure(mult_fig);
  imagesc(mult1);
  pnew=p;
end
