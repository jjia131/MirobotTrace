clear variables
close all
%s = serialport("COM3",115200); %serial port COM+# to connect with robotic arm
imdata = imread("catCopy.png");
imdata = imresize(imdata,[140 NaN]);
imdata = im2bw(imdata);  %recommends imbinarize instead of im2bw
imdata = ~imdata;

data = bwmorph(imdata,'skel'); %need image processing
data = bwmorph(data,'thin',Inf);
imshow(data);
data = bwmorph(data,'spur');
figure
imshow(data);
title("thin");
B = double(imdata);
[row, col] = find(B == 1);
[test]= find(B==1);

X1 = zeros(round((length(row)-1)/4),1); Y1 = zeros(size(X1));
a = 1;
for i = 1:4:length(row)    %get correct X Y coordinate for robotic arm
    X1(a)= 280 - row(i);
    Y1(a) = 75 - col(i);
    a = a+1;
end


X1 = num2str(X1);       %convert number to string
Y1 = num2str(Y1);

%string concatenation to G-Code instruction
%ZABC coordinate does not change, keep end effector vertically down
%F2000 = speed 2000
code1 = strcat('M20 G90 G00 X' , X1 , ' Y',  Y1 , ' Z140 A0.00 B0.00 C0.00 F2000.00');

%write(s,'M3S500','char') %Turn on the gripper
%pause(1)
%Write to robotic amr with zero coordinate G-Code instruction
%write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')

%Search for start
[rowmax,colmax]= size(B);
a = 0; b = 0;
for m=rowmax:-1:1
    for n=1:colmax
        if B(m,n)==1
            a = m;
            b = n; %found start, to be simple assume all connected etc
            break
        end
    end
    if B(m,n)==1
        a = m;
        b = n;%found start, to be simple assume all connected etc
        break
    end
end
%a&b are our start
%get the arm to a,b




%write(s, '','char') asdfasdfsdf %error tells where robot info is needed





B(a,b)=-1;           %-1 tells we already checked it
%checks if all points are found
hold=(140*b)-(140-a);
j=find(test==hold);
test(j)=0;
k=find(test);
if(~isempty(k))  %if not empty
    %c is our completion check
    c=1;
    [B,e]=recursion(a,b,c,B,test,rowmax, colmax);
else
    c=1;
    %do nothing, the picture is done
end

%Put arm back at 0,0,0
%write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')


function [B,c] = recursion(a,b,c,B,test,rowmax, colmax)
while c~=0
    if a-1>=0 && b-1>=0 && a+1<=rowmax && b+1<=colmax
        %left
        if B(a,b-1)==1
            b=b-1;
            break
        end
        %up
        if B(a-1,b)==1
            a=a-1;
            break
        end
        %right
        if B(a,b+1)==1
            b=b+1;
            break
        end
        %down
        if B(a+1,b)==1
            a=a+1;
            break
        end
        %upleft
        if B(a-1,b-1)==1
            a=a-1;
            b=b-1;
            break
        end
        %upright
        if B(a-1,b+1)==1
            a=a-1;
            b=b+1;
            break
        end
        
        %downright
        if B(a+1,b+1)==1
            a=a+1;
            b=b+1;
            break
        end
        
        %downleft
        if B(a+1,b-1)==1
            a=a+1;
            b=b-1;
            break
        end
        %if it makes it through each direction without a 1, is a spur (has no
        %connecting spots so it must 'backtrack')
        %backtrack means going to places already marked -1 (will make thicker
        %line) but will find the next unmarked (1) spot. We know there are more
        %since there is a completion check when each new spot is found.
        %downleft
        
        if B(a+1,b-1)==-1
            a=a+1;
            b=b-1;
            break
        end
        %downright
        if B(a+1,b+1)==-1
            a=a+1;
            b=b+1;
            break
        end
        %upright
        if B(a-1,b+1)==-1
            a=a-1;
            b=b+1;
            break
        end
        %upleft
        if B(a-1,b-1)==-1
            a=a-1;
            b=b-1;
            break
        end
        %down
        if B(a+1,b)==-1
            a=a+1;
            break
        end
        %right
        if B(a,b+1)==-1
            b=b+1;
            break
        end
        %left
        if B(a,b-1)==-1
            b=b-1;
            break
        end
        %up
        if B(a-1,b)==-1
            a=a-1;
            break
        end
        
    end %if end
    
    %if surrounded by 0s and -2s or c>10
   % if c>=10
        %move arm up and goto next unplotted spot
        %arm down and continue
        code2 = strcat('M20 G90 G00 X0 Y0 Z40 A0.00 B0.00 C0.00 F2000.00');
        %pause(1)
        k=find(test);
        if(isempty(k))
            c=0; %it is completed
        else
            x=test(k(1));
            b=ceil(x/140);
            a=140-((140*b)-x);
            %write(s, '','char')  %goes to it
            %pause(1)
            test(k(1))=0;   %mark it
            code3 = strcat('M20 G90 G00 X0 Y0 Z140 A0.00 B0.00 C0.00 F2000.00');
        end
   % end
end

%break goes here

while c~=0
    
    
    
    
    
    %write(s, '','char') asdfasdfsdf %error tells where robot info is needed
    %pause(1)
    %tells arm to move
    
    
    
    
    if(B(a,b)==-1)%this is a safety to prevent infinite loop
        B(a,b)=-2;   %-2 prevents checking same -1
        c=c+1;  %counts every -1 since lsat 1
       
    if c>=10 %been through a lot of -1s
        %move arm up and goto next unplotted spot
        %arm down and continue
        code2 = strcat('M20 G90 G00 X0 Y0 Z40 A0.00 B0.00 C0.00 F2000.00');
        %pause(1)
        k=find(test);
        if(isempty(k))
            c=0; %it is completed
        else
            x=test(k(1));
            b=ceil(x/140);
            a=140-((140*b)-x);
            %write(s, '','char')  %goes to it
            %pause(1)
            test(k(1))=0;   %mark it
            code3 = strcat('M20 G90 G00 X0 Y0 Z140 A0.00 B0.00 C0.00 F2000.00');
        end
    end
        
    else
        B(a,b)=-1;
        c=1;
        hold=(140*b)-(140-a);
        j=find(test==hold);
        if(~isempty(j))
            test(j)=0;
        end
        k=find(test);
        m=size(k);
        if(isempty(k)|| m(1)<=10)
            c=0; %it is completed
            break
        end
    end
    
    [B,c]=recursion(a,b,c,B,test,rowmax,colmax);
end
end
