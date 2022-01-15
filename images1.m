%not tested because of the end at 126 and because of robot arm info missing

clear variables
close all
%s = serialport("COM3",115200); %serial port COM+# to connect with robotic arm
imdata = imread("circle.jpg");
imdata = imresize(imdata,[130 NaN]);
imdata = im2bw(imdata);  %recommends imbinarize instead of im2bw
imdata = ~imdata;

data = bwmorph(imdata,'skel'); %need image processing
data = bwmorph(data,'thin',Inf);
imshow(imdata);
%data = bwmorph(data,'endpoints');
figure
imshow(data);
title("thin");
[row, col] = find(data == 1);
min = min(row);
max = max(row);
cutoff = min + (max-min)/2;   %split to two images with (max-min)/2

a = 1; b = 1;
for i = 1:length(row)  %copy to upper and lower half
    if row(i) <= cutoff
        row1(a) = row(i);
        col1(a) = col(i);
        a = a+1;
    else
        row2(b) = row(i);
        col2(b) = col(i);
        b = b+1;
    end
end
B = double(imdata);
half1 = zeros(size(B));
half2 = zeros(size(B));
for i = 1:a-1
    half1(row1(i),col1(i)) = 1;
end
figure
imshow(half1);
title("half1");

for i = 1:b-1
    half2(row2(i),col2(i)) = 1;
end
figure
imshow(half2);
title("half2");    %show image with halfs
X1 = zeros(round((length(row1)-1)/4),1); Y1 = zeros(size(X1));
a = 1;
for i = 1:4:length(row1)    %get correct X Y coordinate for robotic arm
    X1(a)= 280 - row1(i);
    Y1(a) = 75 - col1(i);
    a = a+1;
end

X2 = zeros(round((length(row2)-1)/4),1); Y2 = zeros(size(X2));
b = 1;
for i = 1:4:length(row2)    %get correct X Y coordinate for robotic arm
    X2(b)= 280 - row2(i);
    Y2(b) = 75 - col2(i);
    b = b+1;
end

X1 = num2str(X1);       %convert number to string
Y1 = num2str(Y1);
X2 = num2str(X2);
Y2 = num2str(Y2);
%string concatenation to G-Code instruction
%ZABC coordinate does not change, keep end effector vertically down
%F2000 = speed 2000
code1 = strcat('M20 G90 G01 X' , X1 , ' Y',  Y1 , ' Z140 A0.00 B0.00 C0.00 F2000.00');
code2 = strcat('M20 G90 G01 X' , X2 , ' Y',  Y2 , ' Z140 A0.00 B0.00 C0.00 F2000.00');
%write(s,'M3S500','char')
%pause(1)
%Write to robotic amr with zero coordinate G-Code instruction
write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')


% find the right z coorindate





%Search for start

for m=rowmax:1
    for n=1:colmax
        if half1(m,n)==1
            %found start, to be simple assume all connected etc
            break
        end
        if half1(a,b)==1
            break
        end
    end
end
%a&b are our start
%get the arm to a,b
write(s, '','char') asdfasdfsdf %error tells where robot info is needed
half1(m,n)=-1;           %-1 tells we already checked it

%c is our completion check
c=false;

%this was at the end of the originalcode is it needed for the arm??
%write to robotic arm
%write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')
%pause(1)
%write(s,'M3S0','char')



e,f=recursion1(m,n,c)
c=false;
g,h=recursion2(e,f,c)


%Put arm back at 0,0,0
write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')


end %need an end here so it doesn't go to the recursion now but images.m doesn't have a function ...



function [a,b] = recursion1(a,b,c)
d=false;

while d==false
    if a-1>=0 && b-1>=0
        %upleft
        if half1(a-1,b-1)==1
            a=a-1;
            b=b-1;
            break
        end
        %left
        if half1(a,b-1)==1
            b=b-1;
            break
        end
        %up
        if half1(a-1,b)==1
            a=a-1;
            break
        end
        
        
        if a+1<=rowmax && b+1<=colmax
            %upright
            if half1(a-1,b+1)==1
                a=a-1;
                b=b+1;
                break
            end
            %right
            if half1(a,b+1)==1
                b=b+1;
                break
            end
            %downright
            if half1(a+1,b+1)==1
                a=a+1;
                b=b+1;
                break
            end%down
            if half1(a+1,b)==1
                a=a+1;
                break
            end
            %downleft
            if half1(a+1,b-1)==1
                a=a+1;
                b=b-1;
                break
            end
        end
    end
    %if it makes it through each direction without a 1, it has reached the end
    d=true;
    c=true;
end

%break goes here

while c==false
    write(s, '','char') asdfasdfsdf %error tells where robot info is needed
    %pause(1)
    %tells arm to move
    half1(a,b)=-1;
    recursion1(a,b,c)
end
%if c is true a and b stayed the same
end

function [a,b] = recursion2(a,b,c)
d=false;

while d==false
    if a+1<=rowmax && b+1<=colmax
        %upright
        if half2(a-1,b+1)==1
            a=a-1;
            b=b+1;
            break
        end
        %right
        if half2(a,b+1)==1
            b=b+1;
            break
        end
        %downright
        if half2(a+1,b+1)==1
            a=a+1;
            b=b+1;
            break
        end%down
        if half2(a+1,b)==1
            a=a+1;
            break
        end
        %downleft
        if half2(a+1,b-1)==1
            a=a+1;
            b=b-1;
            break
        end
        
        
        if a-1>=0 && b-1>=0
            %upleft
            if half2(a-1,b-1)==1
                a=a-1;
                b=b-1;
                break
            end
            %left
            if half2(a,b-1)==1
                b=b-1;
                break
            end
            %up
            if half2(a-1,b)==1
                a=a-1;
                break
            end
        end
    end
    %if it makes it through each direction without a 1, it has reached the end
    d=true;
    c=true;
end

%break goes here

while c==false
    write(s, '','char') asdfasdfsdf %error tells where robot info is needed
    %pause(1)
    %tells arm to move
    half2(a,b)=-1;
    recursion2(a,b,c)
end
%if c is true a and b stayed the same
end

