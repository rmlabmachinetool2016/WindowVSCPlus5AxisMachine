  %% Read Two file data to compare
%EXPERIMENTGRAPH Summary of this function goes here
    %   Detailed explanation goes here
    clc
    clear all;
    FontSize = 24;
    TickFontSize = 20;
%     FileInput1 = 'RMechDataFileSimulR5Time10MKnown.rme';
%     FileInput1 = 'R5Time10W0KcConstKv40Kp60.rme';
    FileInput1 = 'R30F200Two.rme';%SimulR5Time10W0KcChangeMax10.rme';%'R5Time10W0.rme';%'R5Time10W0KcChange.rme';%'R5Time10W0.rme';
%     FileInput2 = 'R5Time10W0KcChange2Kv40Kp60.rme';%'RMechDataFileSimulR5Time10Fc.rme';
    FileInput2 = 'R30F200Two.rme';
    disp('Start DrawGraph from File '); 
    disp(FileInput1);
    disp(FileInput2);
    TempNum_samp = 10000;
    num_samp = TempNum_samp;
    m_TimeTotal = 0.0;
    ZoonCoefficient = 10;
    TranslateX = 0;
    TranslateY = 0;
    RefMinX = 0;RefMinY = 0;
    RefMaxX = 0;RefMaxY = 0;
    
    %File 1 Data
    m_TimeNow = zeros(1,num_samp);
    vec_realxExp = zeros(2,num_samp);
    vec_realx = zeros(2,num_samp);
	vec_realx_1 = zeros(2,num_samp);% Not use 
    vec_realx_2 = zeros(2,num_samp);% Not use
	vec_refr = zeros(2,num_samp);
	vec_refr_1 = zeros(2,num_samp);%Not use
    vec_refr_2 = zeros(2,num_samp);%Not use
	vec_el = zeros(2,num_samp);
	vec_ew = zeros(2,num_samp);
	vec_u = zeros(2,num_samp);
	vec_eVoltSup = zeros(2,num_samp);
	vec_Abso_ec = zeros(2,num_samp);
    %File 2 Data
    m_TimeNowF2 = zeros(1,num_samp);
    vec_realxExpF2 = zeros(2,num_samp);
    vec_realxF2 = zeros(2,num_samp);
	vec_realx_1F2 = zeros(2,num_samp);% File 2
    vec_realx_2F2 = zeros(2,num_samp);% File 2
    vec_elF2 = zeros(2,num_samp);
	vec_ewF2 = zeros(2,num_samp);
    vec_uF2 = zeros(2,num_samp);
	vec_eVoltSupF2 = zeros(2,num_samp);
    vec_Abso_ecF2 = zeros(2,num_samp);
    
%     num_variable = 14;
    num_variable = 12;
    vec_temp = zeros(1,num_variable);
    %Open file  1 to read
    FID_in = fopen(FileInput1,'r'); 
%     m_TimeTotal = fread(FID_in, 1,'double');
    
  %Start read data to variable 
    for i=1:TempNum_samp
    vec_temp = fread(FID_in,num_variable,'double'); 
    if (numel(vec_temp)>0)
        m_TimeNow(i) = vec_temp(1);
        vec_temp(3) = vec_temp(3)+TranslateX;
        vec_temp(4) = vec_temp(4)+TranslateY;
        vec_temp(5) = vec_temp(5)+TranslateX;
        vec_temp(6) = vec_temp(6)+TranslateY;
%         vec_realx(1,i) =  vec_temp(2);vec_realx(2,i) =  vec_temp(3);
         vec_refr(1,i) =  vec_temp(3);vec_refr(2,i) =  vec_temp(4);
             if (i>1)
                vec_refr_1(1,i) = (vec_refr(1,i)-vec_refr(1,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
                vec_refr_1(2,i) =  (vec_refr(2,i)-vec_refr(2,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
            end;
            if (i>2)
                vec_refr_2(1,i) = (vec_refr_1(1,i)-vec_refr_1(1,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
                vec_refr_2(2,i) =  (vec_refr_1(2,i)-vec_refr_1(2,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
            end;
           if (RefMinX>vec_refr(1,i))
                RefMinX= vec_refr(1,i);
            end; 
             if (RefMinY>vec_refr(2,i))
                RefMinY= vec_refr(2,i);
            end; 
            if (RefMaxX<vec_refr(1,i))
                RefMaxX= vec_refr(1,i);
            end; 
             if (RefMaxY<vec_refr(2,i))
                RefMaxY= vec_refr(2,i);
            end; 
            
           TempRadius = sqrt(vec_temp(3)*vec_temp(3)+vec_temp(4)*vec_temp(4));
         TempRealRadius = sqrt(vec_temp(5)*vec_temp(5)+vec_temp(6)*vec_temp(6));
         TempRealRadiusZoom = TempRadius+ (TempRealRadius-TempRadius)*ZoonCoefficient;
         vec_realxExp(1,i) = vec_temp(5)*TempRealRadiusZoom/TempRealRadius;
         vec_realxExp(2,i) = vec_temp(6)*TempRealRadiusZoom/TempRealRadius; 
          vec_realx(1,i) = vec_temp(5);
          vec_realx(2,i) = vec_temp(6); 
          if (i>1)
                vec_realx_1(1,i) = (vec_realx(1,i)-vec_realx(1,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
                vec_realx_1(2,i) =  (vec_realx(2,i)-vec_realx(2,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
            end;
            if (i>2)
                vec_realx_2(1,i) = (vec_realx_1(1,i)-vec_realx_1(1,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
                vec_realx_2(2,i) =  (vec_realx_1(2,i)-vec_realx_1(2,i-1))/(m_TimeNow(i)-m_TimeNow(i-1));
            end;
          
          vec_el(1,i) =  vec_temp(7)*1000;vec_el(2,i) =  vec_temp(8)*1000;
        
%         vec_ew(1,i) =  vec_temp(8)*1000;vec_ew(2,i) =  vec_temp(9)*1000;
%         vec_u(1,i) =  vec_temp(10);vec_u(2,i) =  vec_temp(11);
%         vec_eVoltSup(1,i) =  vec_temp(12);vec_eVoltSup(2,i) =  vec_temp(13);
%         vec_Abso_ec(1,i) =  vec_temp(14)*1000;        
        
        vec_ew(1,i) =  vec_temp(7)*1000;vec_ew(2,i) =  vec_temp(8)*1000;
        vec_u(1,i) =  vec_temp(9);vec_u(2,i) =  vec_temp(10);
        vec_eVoltSup(1,i) =  vec_temp(11);
        vec_eVoltSup(2,i) =  vec_temp(12);
%         vec_Abso_ec(1,i) =  vec_temp(16)*1000;
    elseif (num_samp==TempNum_samp)
            num_samp = i;
             m_TimeTotal =  m_TimeNowF2(i-1);
        end;
    end; 
    fclose(FID_in);   
    
    %Open file  2 to read
    FID_in = fopen(FileInput2,'r'); 
%     m_TimeTotal = fread(FID_in,1,'double');
    
  %Start read data to variable 
%    num_variable = 14;
    num_variable = 12;
    for i=1:num_samp
    vec_temp = fread(FID_in,num_variable,'double'); 
    if (numel(vec_temp)>0)
        m_TimeNowF2(i) = vec_temp(1);
        
        vec_temp(3) = vec_temp(3)+TranslateX;
        vec_temp(4) = vec_temp(4)+TranslateY;
        vec_temp(5) = vec_temp(5)+TranslateX;
        vec_temp(6) = vec_temp(6)+TranslateY;
%         vec_realxF2(1,i) =  vec_temp(2);vec_realxF2(2,i) =  vec_temp(3);
         vec_refr(1,i) =  vec_temp(3);vec_refr(2,i) =  vec_temp(4);
         TempRadius = sqrt(vec_refr(1,i)*vec_refr(1,i)+vec_refr(2,i)*vec_refr(2,i));
         TempRealRadius = sqrt(vec_temp(5)*vec_temp(5)+vec_temp(6)*vec_temp(6));
         TempRealRadiusZoom = TempRadius+ (TempRealRadius-TempRadius)*ZoonCoefficient;
         vec_realxExpF2(1,i) = vec_temp(5)*TempRealRadiusZoom/TempRealRadius;
         vec_realxExpF2(2,i) = vec_temp(6)*TempRealRadiusZoom/TempRealRadius;
         vec_realxF2(1,i) = vec_temp(5);
          vec_realxF2(2,i) = vec_temp(6); 
          if (i>1)
                vec_realx_1F2(1,i) = (vec_realxF2(1,i)-vec_realxF2(1,i-1))/(m_TimeNowF2(i)-m_TimeNowF2(i-1));
                vec_realx_1F2(2,i) =  (vec_realxF2(2,i)-vec_realxF2(2,i-1))/(m_TimeNowF2(i)-m_TimeNowF2(i-1));
            end;
            if (i>2)
                vec_realx_2F2(1,i) = (vec_realx_1F2(1,i)-vec_realx_1F2(1,i-1))/(m_TimeNowF2(i)-m_TimeNowF2(i-1));
                vec_realx_2F2(2,i) =  (vec_realx_1F2(2,i)-vec_realx_1F2(2,i-1))/(m_TimeNowF2(i)-m_TimeNowF2(i-1));
            end;
         
         
         vec_elF2(1,i) =  vec_temp(7)*1000;vec_elF2(2,i) =  vec_temp(8)*1000;
        
        
         vec_ewF2(1,i) =  vec_temp(7)*1000;vec_ewF2(2,i) =  vec_temp(8)*1000;
        vec_uF2(1,i) =  vec_temp(9);vec_uF2(2,i) =  vec_temp(10);
        vec_eVoltSupF2(1,i) =  vec_temp(11);
        vec_eVoltSupF2(2,i) =  vec_temp(12);
%         vec_Abso_ecF2(1,i) =  vec_temp(16)*1000;
        
%         vec_ewF2(1,i) =  vec_temp(8)*1000;vec_ewF2(2,i) =  vec_temp(9)*1000;
%         vec_uF2(1,i) =  vec_temp(10);vec_uF2(2,i) =  vec_temp(11);
%         vec_eVoltSupF2(1,i) =  vec_temp(12);vec_eVoltSupF2(2,i) =  vec_temp(13);
%         vec_Abso_ecF2(1,i) =  vec_temp(14)*1000;
%          end; 
        elseif (m_TimeTotal<0.01)
             m_TimeTotal =  m_TimeNowF2(i-1);
        end; % // End of if (numel(vec_temp)>0)
%          end; 
    end; % // End of for
    fclose(FID_in);  
    if (m_TimeTotal<0.01)
             m_TimeTotal =  m_TimeNowF2(i-1);
       end;  
       
 XTickDistance2 =  (RefMaxX-RefMinX)/10;
 YTickDistance2 =  (RefMaxY-RefMinY)/10;
%     num_samp = num_samp-5;
 %% Reference Contour and Real Contour
%Draw Contour
clf;
FontSize = 10;
TickFontSize = 22;
NotGoodStart = 0;
NotGoodEnd = 0;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
YMax = 14;
XMax = 14;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(vec_refr(1,1+NotGoodStart:num_samp-NotGoodEnd),vec_refr(2,1+NotGoodStart:num_samp-NotGoodEnd),'-','Color','g','LineWidth',2);

set(gca,'XTick',RefMinX:XTickDistance2:RefMaxX,'FontSize',TickFontSize)
set(gca,'YTick',RefMinY:YTickDistance2:RefMaxY,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
% plot(vec_realx(1,1+NotGoodStart:num_samp),vec_realx(2,1+NotGoodStart:num_samp),'-','Color','b','LineWidth',2);
% plot(vec_realxF2(1,1+NotGoodStart:num_samp-NotGoodEnd),vec_realxF2(2,1+NotGoodStart:num_samp-NotGoodEnd),'-','Color','m','LineWidth',2);

plot(vec_realxExp(1,1+NotGoodStart:num_samp),vec_realxExp(2,1+NotGoodStart:num_samp),'-','Color','b','LineWidth',2);
plot(vec_realxExpF2(1,1+NotGoodStart:num_samp-NotGoodEnd),vec_realxExpF2(2,1+NotGoodStart:num_samp-NotGoodEnd),'-','Color','m','LineWidth',2);
set(gca,'position',[.1,.1,.45,.85]);
% set(gca,'position',[.1,.1,.45,.85]);
% h = legend('Reference Contour $r$','Updated Dynamics Real Contour $x$','Original Dynamics Real Contour $x$','Reference $\dot{r_2}$');
h = legend('Reference Contour $r$','Real File1','Real File2','Reference $\dot{r_2}$');
set(h,'interpreter','latex','FontSize',FontSize,'Location','NorthEast')
% legend('Reference r_{1}','Real x_1','Reference r_{2}','Real x_2');
xlabel('X1 [\mum]','FontSize',TickFontSize); ylabel( 'X2 [\mum]','FontSize',TickFontSize)
% axis([-8 8 -7 9])
axis([RefMinX-XTickDistance2 RefMaxX+XTickDistance2 RefMinY-YTickDistance2 RefMaxY+YTickDistance2])
% set(gca,'XTickLabel',-50:10:50);
% set(gca,'YTickLabel',-40:10:60);
 %% Original Dynamics and Updated Dynamics Erro e_l1
%Draw Contour
clf;
FontSize = 28;
TickFontSize = 22;
NotGood = 0;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
YMax = 15;
XMax = GraRadius;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNow(1,1:num_samp),vec_el(1,1:num_samp),'-','Color','b','LineWidth',1);
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize)
set(gca,'YTick',-100:5:100,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_elF2(1,1:num_samp-NotGood),'-','Color','r','LineWidth',1);
% h = legend('Reference Contour r','Updated Dynamics Real Contour x','Original Dynamics Real Contour x','Reference $\dot{r_2}$');
line([0,0],[-50,50],'Color','b')
line([-50,50],[0,0],'Color','b')
h = legend('File 1','File 2');
set(h,'interpreter','latex','FontSize',FontSize,'Location','NorthEast')
% legend('Reference r_{1}','Real x_1','Reference r_{2}','Real x_2');
% legend('Reference ','Chao');
xlabel('Time[s]','FontSize',TickFontSize); ylabel({['Tangential error'],['e_l_1 [\mum]']},'FontSize',TickFontSize)
axis([0 GraTime+0.1 -2000 2000])
 %% Original Dynamics and Updated Dynamics Erro e_l2
%Draw Contour
clf;
FontSize = 28;
TickFontSize = 22;
NotGood = 0;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
YMax = 10;
XMax = GraRadius;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNow(1,1:num_samp),vec_el(2,1:num_samp),'-','Color','b','LineWidth',1);
set(gca,'XTick',-100:1:100,'FontSize',TickFontSize)
set(gca,'YTick',-100:5:100,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_elF2(2,1:num_samp-NotGood),'-','Color','r','LineWidth',1);
% h = legend('Reference Contour r','Updated Dynamics Real Contour x','Original Dynamics Real Contour x','Reference $\dot{r_2}$');
line([0,0],[-50,50],'Color','b')
line([-50,50],[0,0],'Color','b')
h = legend('Conventional','Proposed');
set(h,'interpreter','latex','FontSize',FontSize,'Location','NorthEast')
% legend('Reference r_{1}','Real x_1','Reference r_{2}','Real x_2');
% legend('Reference ','Chao');
xlabel('Time[s]','FontSize',TickFontSize); ylabel({['Normal error'],['e_l_2 [\mum]']},'FontSize',TickFontSize)
axis([0 GraTime+0.1 -YMax YMax])
 %% Reference velocity and Real Velocity
%Draw Contour
clf;
FontSize = 24;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
YMax = 1000;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNowF2(1,1:num_samp),vec_realx_1F2(1,1:num_samp),'-','Color','g','LineWidth',1);

set(gca,'XTick',0:0.5:GraTime,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:0.5:YMax,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNowF2(1,1:num_samp),vec_realx_1F2(2,1:num_samp),'-','Color','c','LineWidth',1);
plot(m_TimeNow(1,1:num_samp),vec_refr_1(1,1:num_samp),'-','Color','b','LineWidth',1);
plot(m_TimeNow(1,1:num_samp),vec_refr_1(2,1:num_samp),'-','Color','m','LineWidth',1);
h = legend('Real $\dot{x_1}$','Real $\dot{x_2}$','Reference $\dot{r_1}$','Reference $\dot{r_2}$');
set(h,'interpreter','latex','FontSize',FontSize,'Location','SouthEast')
% legend('Reference r_{1}','Real x_1','Reference r_{2}','Real x_2');
xlabel('Second','FontSize',FontSize); ylabel('Velocity mm/s','FontSize',FontSize)
axis([0 GraTime+0.1 -YMax YMax])
 %% Reference Acceleration and Real Acceleration 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
YMax = 1500;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNowF2(1,1:num_samp),vec_realx_2F2(1,1:num_samp),'-','Color','g');
set(gca,'XTick',0:0.5:GraTime,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:50:YMax,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNowF2(1,1:num_samp),vec_realx_2F2(2,1:num_samp),'-','Color','c');
plot(m_TimeNow(1,1:num_samp),vec_refr_2(1,1:num_samp),'-','Color','b','LineWidth',2);
plot(m_TimeNow(1,1:num_samp),vec_refr_2(2,1:num_samp),'-','Color','m','LineWidth',2);
h = legend('Real $\ddot{x_1}$','Real $\ddot{x_2}$','Reference $\ddot{r_1}$','Reference $\ddot{r_2}$');
set(h,'interpreter','latex','FontSize',FontSize)
% legend('Reference r_{1}','Real x_1','Reference r_{2}','Real x_2');
xlabel('Second','FontSize',FontSize); ylabel('Acceleration mm/s^2','FontSize',FontSize)
axis([0 GraTime+0.1 -YMax YMax])
 %% Graph Plot  Nu1_Alpha1_Delta1
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
TickFontSize = 20;
NotGood = 0;
YMax = 36;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_uF2(1,1:num_samp-NotGood),'-','Color','b');
set(gca,'XTick',0:1:GraTime,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:0.5:YMax,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_uF2(1,1:num_samp-NotGood)- vec_eVoltSupF2(1,1:num_samp-NotGood),'-','Color','g','LineWidth',2);
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_eVoltSupF2(1,1:num_samp-NotGood),'-','Color','c');
plot(m_TimeNow(1,1:num_samp-NotGood),vec_u(1,1:num_samp-NotGood),'-','Color','r');
 line([0,0],[-50,50],'Color','b')
  line([-50,50],[0,0],'Color','b')
h = legend('Input F2 $\nu _1$','Disturbance value $\delta _1$','Estimated value $\alpha _1$', 'Input F1 ');
set(h,'interpreter','latex','FontSize',24,'Location','SouthEast')
xlabel('Second','FontSize',24); ylabel('Voltage(v)','FontSize',24)
axis([0 GraTime+0.1 -YMax YMax])
 %% Graph Plot  Nu2_Alpha2_Delta2
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
NotGood = 0;
m_GraphFactor = 1000.0;
YMax = 36;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_uF2(2,1:num_samp-NotGood),'-','Color','m');
set(gca,'XTick',0:0.5:GraTime,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:5:YMax,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_uF2(2,1:num_samp-NotGood)- vec_eVoltSupF2(2,1:num_samp-NotGood),'-','Color','b','LineWidth',2);
plot(m_TimeNowF2(1,1:num_samp-NotGood),vec_eVoltSupF2(2,1:num_samp-NotGood),'-','Color','c');
plot(m_TimeNow(1,1:num_samp-NotGood),vec_u(2,1:num_samp-NotGood),'-','Color','r');
line([0,0],[-50,50],'Color','b')
line([-50,50],[0,0],'Color','b')
h = legend('Input F2$\nu _1$','Disturbance value $\delta _1$','Estimated value $\alpha _1$','Input F1');
set(h,'interpreter','latex','FontSize',24,'Location','SouthEast')
xlabel('Second','FontSize',24); ylabel('Voltage(v)','FontSize',24)
axis([0 GraTime+0.1 -YMax YMax])
 %% Graph Plot  Nu1_Alpha1_Delta1 In velocity field X axis
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
NotGood = 0;
YMax = 36;
XMax = 250;

% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(vec_realx_1F2(1,1+NotGood:num_samp-NotGood),vec_uF2(1,1+NotGood:num_samp-NotGood),'-','Color','b');
set(gca,'XTick',-XMax:5:XMax,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:0.5:YMax,'FontSize',TickFontSize)

% set(gca, 'ColorOrder', jet(3));
hold all;
plot(vec_realx_1F2(1,1+NotGood:num_samp-NotGood),vec_uF2(1,1+NotGood:num_samp-NotGood)- vec_eVoltSupF2(1,1+NotGood:num_samp-NotGood),'-','Color','g','LineWidth',2);
plot(vec_realx_1F2(1,1+NotGood:num_samp-NotGood),vec_eVoltSupF2(1,1+NotGood:num_samp-NotGood),'-','Color','c','LineWidth',1);
 plot(vec_realx_1(1,1:num_samp),vec_u(1,1:num_samp),'-','Color','r','LineWidth',2);
 plot(vec_refr_1(1,1:num_samp),vec_refr_2(1,1:num_samp),'-','Color','m');


% axes(AX(2),[-XMax XMax -YMax YMax])
h = legend('Input F2 $\nu _1$','Disturbance F2 $\delta _1$',...
'Estimated F2 $\alpha _1$','Estimated F1 $\alpha _1$','Reference F1 $\ddot{r_1}$');
set(h,'interpreter','latex','Location','SouthEast','FontSize',FontSize,'LineWidth',2)
 line([0,0],[-6,6],'Color','b')
 line([-11,11],[0,0],'Color','b')
xlabel('Velocity [mm/s]','FontSize',FontSize); ylabel('Voltage [v], Accelerator [mm/s^2]','FontSize',FontSize)
ylabel('Voltage [v]','FontSize',FontSize)

% h = text('units','inch', 'position',[10 4], ...
%     'fontsize',14, 'interpreter','latex', 'string',...
%     ['Accelerator $[mm/s^2]$']); 
% set(h,'interpreter','latex','FontSize',FontSize)

axis([-XMax XMax -YMax YMax])
 %% Graph Plot  Nu1_Alpha1_Delta1 In velocity field 2 Y axis
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
NotGood = 0;
YMax = 36;
XMax = 250;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
[AX,H1,H2] = plotYY(vec_realx_1F2(1,1+NotGood:num_samp-NotGood),vec_uF2(1,1+NotGood:num_samp-NotGood),vec_refr_1(1,1:num_samp),vec_refr_2(1,1:num_samp),'plot');

set(get(AX(1),'Ylabel'),'String','Voltage [v]','FontSize',FontSize,'Color','b') 
set(get(AX(2),'Ylabel'),'String','Accelerator [mm/s^2]','FontSize',FontSize,'Color','m') 
set(H1,'LineStyle','-','Color','b')
set(H2,'LineStyle','-','Color','m')
axes(AX(2));
set(AX(2),'YTick',-YMax:0.5:YMax,'FontSize',TickFontSize,'Color','m')
axis([-XMax XMax -YMax YMax])
% set(AX,'YTick',-5:1:5,'FontSize',TickFontSize)
axes(AX(1));
hold all;
set(gca,'XTick',-XMax:5:XMax,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:0.5:YMax,'FontSize',TickFontSize)
axis([-XMax XMax -YMax YMax])
plot(vec_realx_1F2(1,1+NotGood:num_samp-NotGood),vec_uF2(1,1+NotGood:num_samp-NotGood)- vec_eVoltSupF2(1,1+NotGood:num_samp-NotGood),'-','Color','g','LineWidth',2);
plot(vec_realx_1F2(1,1:num_samp),vec_eVoltSupF2(1,1:num_samp),'-','Color','c');
plot(vec_realx_1(1,1:num_samp),vec_u(1,1:num_samp),'-','Color','r','LineWidth',2);
plot(vec_refr_1(1,1:num_samp),vec_refr_2(1,1:num_samp),'-','Color','m');
% 
% % axes(AX(2),[-XMax XMax -YMax YMax])
h = legend('Input F2 $\nu _2$','Disturbance F2 $\delta _2$',...
'Estimated F2 $\alpha _2$','Estimated F1 $\alpha _2$','Reference F1 $\ddot{r_2}$');
set(h,'interpreter','latex','Location','SouthEast','FontSize',FontSize,'LineWidth',2)
 line([0,0],[-35,35],'Color','b')
 line([-50,50],[0,0],'Color','b')
xlabel('Velocity [mm/s]','FontSize',FontSize); 

 %% Graph Plot  Nu2_Alpha2_Delta2 In velocity field 2 Y axis
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
NotGood = 5;
YMax = 3;
XMax = 4;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
[AX,H1,H2] = plotYY(vec_realx_1F2(2,1+NotGood:num_samp-NotGood),vec_uF2(2,1+NotGood:num_samp-NotGood),vec_refr_1(2,1:num_samp),vec_refr_2(2,1:num_samp),'plot');

set(get(AX(1),'Ylabel'),'String','Voltage [v]','FontSize',FontSize,'Color','b') 
set(get(AX(2),'Ylabel'),'String','Accelerator [mm/s^2]','FontSize',FontSize,'Color','m') 
set(H1,'LineStyle','-','Color','b')
set(H2,'LineStyle','-','Color','m')
axes(AX(2));
set(AX(2),'YTick',-YMax:1:YMax,'FontSize',TickFontSize,'Color','m')
axis([-XMax XMax -YMax YMax])
% set(AX,'YTick',-5:1:5,'FontSize',TickFontSize)
axes(AX(1));
hold all;
set(gca,'XTick',-XMax:1:XMax,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:1:YMax,'FontSize',TickFontSize)
axis([-XMax XMax -YMax YMax])
plot(vec_realx_1F2(2,1+NotGood:num_samp-NotGood),vec_uF2(2,1+NotGood:num_samp-NotGood)- vec_eVoltSupF2(2,1+NotGood:num_samp-NotGood),'-','Color','g','LineWidth',2);
plot(vec_realx_1F2(2,1:num_samp),vec_eVoltSupF2(2,1:num_samp),'-','Color','c');
plot(vec_realx_1(2,1:num_samp),vec_u(2,1:num_samp),'-','Color','r','LineWidth',2);
plot(vec_refr_1(2,1:num_samp),vec_refr_2(2,1:num_samp),'-','Color','m');
% 
% % axes(AX(2),[-XMax XMax -YMax YMax])
h = legend('Control input $\nu _2$','Current $\alpha _2$',...
'Augmented PD value $\delta _2$','Propose $\alpha _2$','Reference Accelerator $\ddot{r_2}$');
set(h,'interpreter','latex','Location','SouthEast','FontSize',FontSize,'LineWidth',2)
 line([0,0],[-6,6],'Color','b')
 line([-11,11],[0,0],'Color','b')
xlabel('Velocity [mm/s]','FontSize',FontSize); 
%% Graph
clf;
X = [1:10]; Y = X;
A = plotyy(X,Y,X,Y);
set(A(2),'ytick',[],'yticklabel',[]);

B = axes;
set(B,'yaxislocation','right','ytick',[],'xtick',[])
ylabel(B,'Right Label New')
ylabel(A(1),'Left Label')
ylabel(A(2),'Right Label')
h = text('units','inch', 'position',[2.5 1], ...
    'fontsize',14, 'interpreter','latex', 'string',...
    ['$$New  \dot r_1 $$']); 
set(h, 'rotation', 60)
 %% Graph Plot  Nu2_Alpha2_Delta2 In velocity field
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
NotGood = 5;
YMax = 8;
XMax = 10;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(vec_realx_1F2(2,1:num_samp),vec_uF2(2,1:num_samp),'-','Color','b');
set(gca,'XTick',-XMax:0.5:XMax,'FontSize',TickFontSize)
set(gca,'YTick',-YMax:0.5:YMax,'FontSize',TickFontSize)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(vec_realx_1F2(2,1:num_samp),vec_uF2(2,1:num_samp)- vec_eVoltSupF2(2,1:num_samp),'-','Color','g','LineWidth',2);
plot(vec_realx_1F2(2,1:num_samp),vec_eVoltSupF2(2,1:num_samp),'-','Color','c');
plot(vec_realx_1(2,1:num_samp),vec_u(2,1:num_samp),'-','Color','r','LineWidth',2);
plot(vec_refr_1(2,1:num_samp),vec_refr_2(2,1:num_samp),'-','Color','m');
h = legend('Control input $\nu _2$','Estimated value $\alpha _2$',...
'Disturbance value $\delta _2$','Proposed Estimated value $\alpha _2$','Reference Accelerator $\ddot{r_2}$');
set(h,'interpreter','latex','Location','SouthEast','FontSize',24,'LineWidth',2)
 line([0,0],[-6,6],'Color','b')
 line([-11,11],[0,0],'Color','b')
xlabel('Velocity mm/s','FontSize',24); ylabel('Voltage(v), Accelerator mm/s^2','FontSize',24)
axis([-XMax XMax -YMax YMax])

 %% Graph Plot  Two file data to compare model and real 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
% Draw vec_uX, vec_uY independent Error ////////////////////////////////////
plot(m_TimeNow(1,1:num_samp),vec_u(1,1:num_samp),'-','Color','g');
set(gca,'XTick',0:0.5:GraTime)
set(gca,'YTick',-GraRadius:0.5:GraRadius)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNow(1,1:num_samp),vec_eVoltSupF2(2,1:num_samp),'-','Color','c');
plot(m_TimeNowF2(1,1:num_samp),vec_uF2(1,1:num_samp),'-','Color','b');
plot(m_TimeNowF2(1,1:num_samp),vec_uF2(2,1:num_samp),'-','Color','m');
legend('\nu_{m0}','\nu_{m1}','\nu_{r0}','\nu_{r1}');
xlabel('Second'); ylabel('Voltage out (v)')
axis([0 GraTime+0.1 -3 3])
%% Graph Plot  Two file data to calculate magnitude of disturbance 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
plot(m_TimeNow(1,1:num_samp),vec_u(1,1:num_samp),'-','Color','g');

set(gca,'XTick',0:0.5:GraTime)
set(gca,'YTick',-GraRadius:0.5:GraRadius)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNow(1,1:num_samp),vec_u(2,1:num_samp),'-','Color','c');
plot(m_TimeNowF2(1,1:num_samp-50),vec_uF2(1,1:num_samp-50),'-','Color','b');
plot(m_TimeNowF2(1,1:num_samp-50),vec_uF2(2,1:num_samp-50),'-','Color','m');
plot(m_TimeNowF2(1,1:num_samp-50),vec_realx_1F2(1,1:num_samp-50),'-','Color','b');
plot(m_TimeNowF2(1,1:num_samp-50),vec_realx_1F2(2,1:num_samp-50),'-','Color','m');

text('units','inch', 'position',[2.5 1], ...
    'fontsize',14, 'interpreter','latex', 'string',...
    ['$$ \dot r_1 $$']); 
text('units','inch', 'position',[5 1], ...
    'fontsize',14, 'interpreter','latex', 'string',...
    ['$$ \dot r_2 $$']); 

text('units','inch', 'position',[2.5 1.6], ...
    'fontsize',14, 'interpreter','latex', 'string',...
    ['$$ \delta_{1} $$']); 
text('units','inch', 'position',[5 1.6], ...
    'fontsize',14, 'interpreter','latex', 'string',...
    ['$$ \delta_{2} $$']); 
% hz ='anv'; %text('interpreter','latex', 'string','anv'); 
legend('\delta_{1}','\delta_{2}',' r_1 ','r_2');
xlabel('Second'); ylabel('Voltage out (v)')
axis([0 GraTime+0.1 -11 11])
%% Graph Plot  calculate friction of axial 1 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
plot(vec_realx_1(1,1:num_samp),vec_uF2(1,1:num_samp)-vec_u(1,1:num_samp),'-','Color','g');
set(gca,'XTick',-11:0.5:11)
set(gca,'YTick',-GraRadius:0.5:GraRadius)
% set(gca, 'ColorOrder', jet(3));
hold all;
%  plot(vec_realx_1(2,1:num_samp),vec_uF2(2,1:num_samp)-vec_u(2,1:num_samp),'-','Color','m');
A1 = vec_realx_1(1,490); A2 = vec_uF2(1,490)-vec_u(1,490);
B1 = vec_realx_1(1,984 ); B2 = vec_uF2(1,984)-vec_u(1,984);
C1 = vec_realx_1(1,1964); C2 = vec_uF2(1,1964)-vec_u(1,1964);
D1 = vec_realx_1(1,1474); D2 = vec_uF2(1,1474)-vec_u(1,1474);
FColumbFW = -C1*(C2-D2)/(C1-D1)+C2;
FColumbBW = -A1*(A2-B2)/(A1-B1)+A2;
ViscousFrictionFW = (D2-C2)/(D1-C1);
ViscousFrictionBW = (B2-A2)/(B1-A1);

plot(A1,A2,'ro','Color','r');
plot(B1,B2,'ro','Color','r');
plot(C1,C2,'ro','Color','r');
plot(D1,D2,'ro','Color','r');
plot(0,FColumbFW,'ro','Color','r');
plot(0,FColumbBW,'ro','Color','r');
strNumberFormat = '%6.2f';
text(A1, A2,[' \leftarrow A(' num2str(A1,strNumberFormat) ',' num2str(A2,strNumberFormat) ')'],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
 text(B1, B2,['B(' num2str(B1,strNumberFormat) ',' num2str(B2,strNumberFormat) ')' '\rightarrow '],'HorizontalAlignment','right',...
     'FontSize',16,'Color','b') 
 text(D1, D2,['D(' num2str(D1,strNumberFormat) ',' num2str(D2,strNumberFormat) ')' '\rightarrow '],'HorizontalAlignment','right',...
     'FontSize',16,'Color','b') 
 text(C1, C2,[' \leftarrow C(' num2str(C1,strNumberFormat) ',' num2str(C2,strNumberFormat) ')'],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b')  
 text(0, FColumbBW,[' \leftarrow \eta _B_W_1_0  =' num2str(FColumbBW)],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
  text(0, FColumbFW,['\eta _F_W_1_0  =' num2str(FColumbFW) '\rightarrow'],'HorizontalAlignment','right',...
     'FontSize',16,'Color','b') 
 
 text(2, 0,[' \eta _F_W_1_2 =' num2str(ViscousFrictionFW)],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
 text(-2, 0,[' \eta _B_W_1_2=' num2str(ViscousFrictionBW)],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
% plot(x,y,'ro'); % red data points
% hold on;
% plot(xx,yy,'-','Color','r'); % Red spline curve
%plot(xx1,yy1,'-','Color','b'); % Red spline curve

% AB1= linspace(-3.5,0,200);
% AB2 = (AB1-A1)
% CD1= linspace(0,3.5,200);
 line([A1,0],[A2,FColumbBW])
 line([0,D1],[FColumbFW,D2])
 line([0,0],[-6,6],'Color','m')
  line([-11,11],[0,0],'Color','m')
% hz ='anv'; %text('interpreter','latex', 'string','anv'); 
legend('\delta_{1}');
xlabel('Velocity mm/s'); ylabel('Voltage out (v)')
axis([-7 7 -0.6 0.6])
%% Graph Plot  calculate friction of axial 2 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
plot(vec_realx_1(2,1:num_samp),vec_uF2(2,1:num_samp)-vec_u(2,1:num_samp),'-','Color','m');
set(gca,'XTick',-11:0.5:11)
set(gca,'YTick',-GraRadius:0.5:GraRadius)
% set(gca, 'ColorOrder', jet(3));
hold all;
%  plot(vec_realx_1(2,1:num_samp),vec_uF2(2,1:num_samp)-vec_u(2,1:num_samp),'-','Color','m');
C1 = vec_realx_1(2,490);C2 = vec_uF2(2,490)-vec_u(2,490);
A1 = vec_realx_1(2,984 ); A2 = vec_uF2(2,984)-vec_u(2,984);
D1 = vec_realx_1(2,1964); D2 = vec_uF2(2,1964)-vec_u(2,1964);
B1 = vec_realx_1(2,1474); B2 = vec_uF2(2,1474)-vec_u(2,1474);
FColumbFW = -C1*(C2-D2)/(C1-D1)+C2;
FColumbBW = -A1*(A2-B2)/(A1-B1)+A2;
ViscousFrictionFW = (D2-C2)/(D1-C1);
ViscousFrictionBW = (B2-A2)/(B1-A1);

plot(A1,A2,'ro','Color','r');
plot(B1,B2,'ro','Color','r');
plot(C1,C2,'ro','Color','r');
plot(D1,D2,'ro','Color','r');
plot(0,FColumbFW,'ro','Color','r');
plot(0,FColumbBW,'ro','Color','r');
strNumberFormat = '%6.2f';
text(A1, A2,[' \leftarrow A(' num2str(A1,strNumberFormat) ',' num2str(A2,strNumberFormat) ')'],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
 text(B1, B2,['B(' num2str(B1,strNumberFormat) ',' num2str(B2,strNumberFormat) ')' '\rightarrow '],'HorizontalAlignment','right',...
     'FontSize',16,'Color','b') 
 text(D1, D2,['D(' num2str(D1,strNumberFormat) ',' num2str(D2,strNumberFormat) ')' '\rightarrow '],'HorizontalAlignment','right',...
     'FontSize',16,'Color','b') 
 text(C1, C2,[' \leftarrow C(' num2str(C1,strNumberFormat) ',' num2str(C2,strNumberFormat) ')'],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b')  
 text(0, FColumbBW,[' \leftarrow \eta _B_W_1_0  =' num2str(FColumbBW)],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
  text(0, FColumbFW,['\eta _F_W_1_0  =' num2str(FColumbFW) '\rightarrow'],'HorizontalAlignment','right',...
     'FontSize',16,'Color','b') 
 
 text(2, 0,[' \eta _F_W_1_2 =' num2str(ViscousFrictionFW)],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
 text(-2, 0,[' \eta _B_W_1_2=' num2str(ViscousFrictionBW)],'HorizontalAlignment','left',...
     'FontSize',16,'Color','b') 
% plot(x,y,'ro'); % red data points
% hold on;
% plot(xx,yy,'-','Color','r'); % Red spline curve
%plot(xx1,yy1,'-','Color','b'); % Red spline curve

% AB1= linspace(-3.5,0,200);
% AB2 = (AB1-A1)
% CD1= linspace(0,3.5,200);
 line([A1,0],[A2,FColumbBW])
 line([0,D1],[FColumbFW,D2])
 line([0,0],[-6,6],'Color','b')
  line([-11,11],[0,0],'Color','b')
 
% hz ='anv'; %text('interpreter','latex', 'string','anv'); 
legend('\delta_{2}');
xlabel('Velocity mm/s'); ylabel('Voltage out (v)')
% axis([-3.5 3.5 -2.5 2.5])
axis([-7 7 -0.6 0.6])

%% Graph Plot Compare Control output and predicted output on  axial 1 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
m_GraphFactor = 1000.0;
plot(vec_realx_1(1,1:num_samp),vec_u(1,1:num_samp),'-','Color','g');
set(gca,'XTick',-11:0.5:11)
set(gca,'YTick',-GraRadius:0.5:GraRadius)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(vec_realx_1F2(1,1:num_samp),vec_uF2(1,1:num_samp),'-','Color','m');
%  plot(vec_realx_1(2,1:num_samp),vec_uF2(2,1:num_samp)-vec_u(2,1:num_samp),'-','Color','m');
  line([0,0],[-6,6],'Color','b')
  line([-11,11],[0,0],'Color','b')
% hz ='anv'; %text('interpreter','latex', 'string','anv'); 
legend('Predicted \nu_{m1}','Control \nu_{r1}');
xlabel('Velocity mm/s'); ylabel('Voltage out (v)')
% axis([-3.5 3.5 -2.5 2.5])
axis([-7 7 -3.5 3.5])
%% Graph Plot  Compare Control output and predicted output on axial 2 
%Draw Contour
clf;
GraRadius = double(TempRadius);
GraTime=  double(m_TimeTotal);
NotGood=0;
m_GraphFactor = 1000.0;
plot(vec_realx_1(2,1:num_samp-NotGood),vec_u(2,1:num_samp-NotGood),'-','Color','b');
set(gca,'XTick',-11:0.5:11)
set(gca,'YTick',-GraRadius:0.5:GraRadius)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(vec_realx_1F2(2,1:num_samp-NotGood),vec_uF2(2,1:num_samp-NotGood),'-','Color','m');
plot(vec_refr_1(2,1:num_samp-NotGood),vec_refr_2(2,1:num_samp-NotGood),'-','Color','m');
%  plot(vec_realx_1(2,1:num_samp),vec_uF2(2,1:num_samp)-vec_u(2,1:num_samp),'-','Color','m');
 line([0,0],[-6,6],'Color','b')
  line([-11,11],[0,0],'Color','b')
% hz ='anv'; %text('interpreter','latex', 'string','anv'); 
legend('Predicted \nu_{m2}','Control \nu_{r2}');
xlabel('Velocity mm/s'); ylabel('Voltage out (v)')
axis([-3.5 3.5 -2.5 2.5])
% axis([-7 7 -3.5 3.5])
%% Plot Velocity Out To calculate Viscous Friction and Plot Spline of Vout
clf;
plot(m_TimeNow(1,1:num_samp),vec_realx_1(1,1:num_samp),'-','Color','g');
set(gca,'XTick',0:0.005:20)
set(gca,'YTick',-11:0.005:11)
% set(gca, 'ColorOrder', jet(3));
hold all;
plot(m_TimeNow(1,1:num_samp),vec_realx_1(2,1:num_samp),'-','Color','c');
% plot(m_TimeNow(1,1:num_samp),vec_refr_1F2(1,1:num_samp),'-','Color','b');
% plot(m_TimeNow(1,1:num_samp),vec_refr_1F2(2,1:num_samp),'-','Color','m');
%  plot(m_TimeNow(1,490),vec_realx_1(2,490),'-','Color','r'); % Red spline curve

legend('Velocity_X','Velocity_Y','vecu_XF2','vecu_YF2');
xlabel('Second'); ylabel('Velocity mm/s')
axis([0 21 -12 12])


