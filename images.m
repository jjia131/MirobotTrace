clear variables
close all
s = serialport("COM3",115200);
imdata = imread("circle.jpg");
imdata = imresize(imdata,[130 NaN]);
imdata = im2bw(imdata);
imdata = ~imdata;

data = bwmorph(imdata,'skel');
data = bwmorph(data,'thin',Inf);
imshow(imdata);
%data = bwmorph(data,'endpoints');
figure
imshow(data);
title("thin");
[row, col] = find(data == 1);
min = min(row);
max = max(row);
cutoff = min + (max-min)/2;

a = 1; b = 1;
for i = 1:length(row)
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
title("half2");
X1 = zeros(round((length(row1)-1)/4),1); Y1 = zeros(size(X1));
a = 1;
for i = 1:4:length(row1)
    X1(a)= 280 - row1(i);
    Y1(a) = 75 - col1(i);
    a = a+1;
end

X2 = zeros(round((length(row2)-1)/4),1); Y2 = zeros(size(X2));
b = 1;
for i = 1:4:length(row2)
    X2(b)= 280 - row2(i);
    Y2(b) = 75 - col2(i);
    b = b+1;
end

X1 = num2str(X1);
Y1 = num2str(Y1);
X2 = num2str(X2);
Y2 = num2str(Y2);
code1 = strcat('M20 G90 G01 X' , X1 , ' Y',  Y1 , ' Z140 A0.00 B0.00 C0.00 F2000.00');
code2 = strcat('M20 G90 G01 X' , X2 , ' Y',  Y2 , ' Z140 A0.00 B0.00 C0.00 F2000.00');
write(s,'M3S500','char')
pause(1)
write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')
for i = 1:length(code1(:,1))
    write(s,code1(i,:),'char')
    pause(1)
end

for i = length(code2(:,1)):-1:1
    write(s,code2(i,:),'char')
    pause(1)
end
write(s,'M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000','char')
pause(1)
write(s,'M3S0','char')