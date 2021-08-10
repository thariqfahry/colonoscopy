raw = csvread('colon_coordinates.csv',1,0);
raw = [raw;raw(end,:)-0.1]; % add dummy

rawsize=size(raw);

resolution = 5;
i=1;
j=1;
proc = []; %#ok<NASGU>
curr = [];

proc = raw(1,:);
proc = [proc;raw(i+1,:)];

i=2;
while i < rawsize(1,1)
    
  while  (pdist([proc(end,:);raw(i,:)],'euclidean') > resolution || pdist([proc(end,:);proc(end-1,:)],'euclidean') > resolution)
    
    while pdist([proc(end,:);proc(end-1,:)],'euclidean') > resolution
    x1=proc(end-1,1);
    y1=proc(end-1,2);
    x2=proc(end,1);
    y2=proc(end,2);
    
    curr = [x1,y1;x2,y2];
    %dist=pdist(curr,'euclidean');
        
       %if dist > resolution
            mp = [(x1+x2)/2, (y1+y2)/2];
        
            proc(end,:)=[];
            proc = [proc;mp]; %#ok<AGROW>
        
            %disp(mp);
       %else
           %proc = [proc;raw(i,:)]; %#ok<AGROW>
       %end
    end
    proc = [proc;raw(i,:)];
  end
  i=i+1;
  proc = [proc;raw(i,:)];
end



csvwrite('smooth_coordinates.csv',proc);
%abc