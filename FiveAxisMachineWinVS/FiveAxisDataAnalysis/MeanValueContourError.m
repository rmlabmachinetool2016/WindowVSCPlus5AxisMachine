%% Mean contour error Circle R15mm, F= 3.14mm/s. Time 15s
% plot(vec_refr_1Coarse(1,1:25),vec_ViscousCoarse(1,1:25),'-','Color','g');
clf;

TickFontSize = 14;
LegendFontSize = 14;
set(gca,'XTick',1:1:10,'FontSize',14)
set(gca,'YTick',0:1:5,'FontSize',12)
% hold all;
x = linspace(1,10,10);% simple set of x values
y = [2.62,2.60,2.54  ,2.49 ,2.49 ,2.49, 2.48,2.81,2.46,2.47]; % random y values for each x value
x1 = linspace(1,10,10);% simple set of x values
y1 = [0.39, 0.42, 0.39, 0.38, 0.37, 0.69, 0.40, 0.42, 0.42, 0.41]; % random y values for each x value
%Default
% x = [0,0.3 ,1.2 ,2  ,4   ,5.2 ,8.6 ,9.2 ,10.2,10.4,10.5]; % simple set of x values
% y = [5,3.33,1.25,0.8,0.45,0.35,0.25,0.28,0.45,0.55,0.7]; % random y values for each x value
xx = linspace(0,10.5,1000); % many x values for plotting
yy = spline(x,y,xx); % generate y values based on spline interpolation
plot(x,y,'b*'); % red data points
hold on;
plot(x1,y1,'rpentagram'); % red data points

h =legend('Conventional','Proposed');
set(h,'FontSize',LegendFontSize); 
hFig = figure(1);
set(gca,'position',[.05,.08,.4,.3]);
% set(hFig, 'Position', [0 0 505 505])
% set(gca,'XTickLabel',1:1:10,'FontSize',TickFontSize);
% set(gca,'YTickLabel',0:1:4,'FontSize',TickFontSize);
xlabel('Trial','FontSize',TickFontSize); 
ylabel({['Mean of contour error'],['magnitude[\mum]']},'FontSize',TickFontSize)
axis([1 10 0 4])
% set(gca,'XTickLabel',1:10:100);
% set(gca,'XTick',0:2:10,'FontSize',13)
% set(gca,'XTickLabel',{'1',' ',' ',' ',' ',' ',' ',' ',' ','10'},'FontSize',TickFontSize)
%  set(gca,'XTickLabel',['0';' ';'1';' ';'2';' ';'3';' ';'10'])
% set(gca,'XTickLabel',['1';' ';' ';' ';' ';' ';' ';' ';' ';'10'])
%% Mean contour error Parallelgram  R4mm, F= 3.14mm/s. Time 18s
% plot(vec_refr_1Coarse(1,1:25),vec_ViscousCoarse(1,1:25),'-','Color','g');
clf;
TickFontSize = 14;
LegendFontSize = 14;
set(gca,'XTick',1:1:10,'FontSize',14)
set(gca,'YTick',0:1:5,'FontSize',12)
% hold all;
x = linspace(1,10,10);% simple set of x values
y = [2.34,2.31,2.41  ,2.44 ,2.35 ,2.38, 2.39,2.41,2.41,2.40]; % random y values for each x value
x1 = linspace(1,10,10);% simple set of x values
y1 = [1.09, 0.43, 0.40, 0.54, 0.52, 0.40, 0.53, 0.81, 0.48, 0.43]; % random y values for each x value
%Default
% x = [0,0.3 ,1.2 ,2  ,4   ,5.2 ,8.6 ,9.2 ,10.2,10.4,10.5]; % simple set of x values
% y = [5,3.33,1.25,0.8,0.45,0.35,0.25,0.28,0.45,0.55,0.7]; % random y values for each x value
xx = linspace(0,10.5,1000); % many x values for plotting
yy = spline(x,y,xx); % generate y values based on spline interpolation
plot(x,y,'b*'); % red data points
hold on;
plot(x1,y1,'rpentagram'); % red data points

h =legend('Conventional','Proposed');
set(h,'FontSize',LegendFontSize); 
hFig = figure(1);
set(gca,'position',[.05,.08,.4,.3]);
% set(hFig, 'Position', [0 0 505 505])
xlabel('Trial','FontSize',TickFontSize); 
ylabel({['Mean of contour error'],['magnitude[\mum]']},'FontSize',TickFontSize)
axis([1 10 0 4])
% set(gca,'XTickLabel',1:10:100);
set(gca,'XTickLabel',{'1',' ',' ',' ',' ',' ',' ',' ',' ','10'})
%  set(gca,'XTickLabel',['0';' ';'1';' ';'2';' ';'3';' ';'10'])
% set(gca,'XTickLabel',['1';' ';' ';' ';' ';' ';' ';' ';' ';'10'])

%% Mean contour error Triangle contour R4mm, F= 3.14mm/s. Time 25.7s
% plot(vec_refr_1Coarse(1,1:25),vec_ViscousCoarse(1,1:25),'-','Color','g');
clf;
TickFontSize = 14;
LegendFontSize = 14;
set(gca,'XTick',1:1:10,'FontSize',14)
set(gca,'YTick',0:1:5,'FontSize',12)
% hold all;
x = linspace(1,10,10);% simple set of x values
y = [2.54,3.11,2.54  ,2.55 ,2.62 ,2.59, 2.57,2.65,2.58,3.35]; % random y values for each x value
x1 = linspace(1,10,10);% simple set of x values
y1 = [1.79, 0.75, 0.51, 0.52, 0.52, 0.59, 0.50, 0.67, 0.60, 1.55]; % random y values for each x value
%Default
% x = [0,0.3 ,1.2 ,2  ,4   ,5.2 ,8.6 ,9.2 ,10.2,10.4,10.5]; % simple set of x values
% y = [5,3.33,1.25,0.8,0.45,0.35,0.25,0.28,0.45,0.55,0.7]; % random y values for each x value
xx = linspace(0,10.5,1000); % many x values for plotting
yy = spline(x,y,xx); % generate y values based on spline interpolation
plot(x,y,'b*'); % red data points
hold on;
% plot(0:4,0:4)
plot(x1,y1,'rpentagram'); % red data points

h =legend('Conventional','Proposed');
set(h,'FontSize',LegendFontSize); 
hFig = figure(1);
set(gca,'position',[.05,.08,.4,.3]);
% set(hFig, 'Position', [0 0 505 505])
xlabel('Trial','FontSize',TickFontSize); 
ylabel({['Mean of contour error'],['magnitude[\mum]']},'FontSize',TickFontSize)
axis([1 10 0 4])
% set(gca,'XTickLabel',1:10:100);
set(gca,'XTickLabel',{'1',' ',' ',' ',' ',' ',' ',' ',' ','10'})
%  set(gca,'XTickLabel',['0';' ';'1';' ';'2';' ';'3';' ';'10'])
% set(gca,'XTickLabel',['1';' ';' ';' ';' ';' ';' ';' ';' ';'10'])
%% Velocity Viscous Graph
clf;
plot(0:4,0:4)
set(gca,'XLim',[0 4])
set(gca,'XTick',[0:0.5:4])
set(gca,'XTickLabel',['0';' ';'1';' ';'2';' ';'3';' ';'4'])