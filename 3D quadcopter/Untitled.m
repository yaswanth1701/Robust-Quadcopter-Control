waypoints = [0    0   0;
              1    1   1;
             2    0   2;
              3    -1  1;
              4    0   0]'.*2;
b=zeros(1,32);

for i=1:4
    b(1,i)=waypoints(i);
end 
wayx=waypoints(1,:);
size(wayx,2)

    