#if 0
%%
%%
%%
%%
%%
%%
%%
function [D C] = DoubleWavelet(X,Y,varargin)

%Default Values
display = 1;
FirstDamping = 0.7; %noise immunity
secondwidth = 6; %curve matching
treshold = 1.75;
offset = 3.0521;
margin = 0.05;
numwidth=2;
confidence = 2*numwidth+1;
%optionnal parameters
nbarg = nargin-2;  
for i=1:2:nbarg
    name = varargin{i};
    value= varargin{i+1};
    switch name
        case 'display'
            if strcmp(value,'off')
                display = 0;
            elseif strcmp(value,'on')
                display = 1;
            end
        case 'offset'
            offset = value;
        case 'treshold'
            treshold = value;
        case 'firstdamping'
            FirstDamping = value;
        case 'secondwidth'
            secondwidth = value;
    end
end

%match size...
length = min(size(X,2),size(Y,2));
Y = Y(1,1:length);
X = X(1,1:length)-offset;
if display
    figure('Name','Successive wavelet detection','NumberTitle','on')
    subplot(4,4,1:4)
    plot(X,Y);
    ylabel('signal input', 'fontweight', 'bold');
end
%tic;
% compute wavelet number one
FIRST = cat(2,lognpdf(5:-1:1,0,FirstDamping),0,-lognpdf(1:5,0,FirstDamping));
FIRST = FIRST/ sum(lognpdf([1:5],0,FirstDamping));
YPRIME = conv(Y,FIRST);
YPRIME = YPRIME(6:length+5);
if display
    subplot(4,4,5);
    plot(-5:5,FIRST);
    ylabel('first convolution', 'fontweight', 'bold');
    subplot(4,4,6:8)
    plot(X,YPRIME);
end

%compute wavelet number two
%perform three time to match different width parameter
D=[];
numfound=[];
if display
    subplot(4,4,9)
    ylabel('second convolution', 'fontweight', 'bold');
end
for w=-numwidth:numwidth
    width = secondwidth+w;
    SECOND = sin(-width:width*pi/width);

    YSECOND = conv(YPRIME,SECOND(1,:)); %convolution
    YSECOND = YSECOND(1+width:length+width);%size adjustment
    YSECOND = YSECOND.^2; %always positive, use RMS to set treshold

    if display   
        subplot(4,4,9)
        hold all;
        plot(-width:width,SECOND);
        subplot(4,4,10:12)
        hold all;
        plot(X,YSECOND);    
    end

    list = [];
    thres = treshold*rms(YSECOND);

    for i=1:max(size(YSECOND))
        if YSECOND(i)>thres,
            list= cat(1,list,i);
        end
    end
    n = size(list,1);
    tempD=[];
    for i=1:n-1
        if list(i)==list(i+1)-1 %two consecutive good values
            if YPRIME(list(i+1))<0 && YPRIME(list(i))>=0
                tempD=cat(1,tempD,X(list(i))+(X(list(i+1))-X(list(i)))*(YPRIME(list(i))/(YPRIME(list(i))-YPRIME(list(i+1)))));
            end
        end
    end
    f=size(tempD,1);
    numfound = cat(2,numfound,f);
    [dx dy]=size(D);
    replacementD = zeros(max(dx,f),dy+1);
    replacementD(1:dx,1:dy) = D;
    replacementD(1:f,dy+1)=tempD;
    D=replacementD;
end

%at this point we have a Nxnumwidth array of detected obstacle and
% an array of 1xnumwidth of number of obstacle in each list.
if display
    subplot(4,4,13:14);
    barh(D);
    set(gca,'XLim',[0 max(X)]);
    ylabel('detections', 'fontweight', 'bold');
end
% lets sort the values in an array of Nx2 where we have a distance and a
% confidence level
C=[-10 -10];%dummy value
for i=1:size(D,2)
    for j=1:numfound(i)
        for k=1:size(C,1)
            if D(j,i) < C(k,1)-margin %insert before
                C=cat(1,C(1:k-1,:),[D(j,i) 1],C(k:size(C,1),:));
                break;
            elseif D(j,i) > C(k,1)+margin %continue
            else %match with somebody
                C(k,2)=C(k,2)+1;
                break;
            end;
        end;
        if k==size(C,1)%insert at the end
            C=cat(1,C,[D(j,i) 1]);
        end;
    end;
end;
C=C(2:size(C,1),:);%eliminate dummy value...
if display
   subplot(4,4,15:16);
   stem(C(:,1),C(:,2));
   set(gca,'XLim',[0 max(X)]);
end
%prune results
D=[];
for i=1:size(C,1)
    if C(i,2) >= confidence
       D= cat(2,D,C(i,1));
    end;
end;
%toc
#endif
