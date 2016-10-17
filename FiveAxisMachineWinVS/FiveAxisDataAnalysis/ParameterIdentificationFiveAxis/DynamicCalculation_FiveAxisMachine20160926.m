%% Calculate dynamic of the five axis machine tool
% NC post-processor for 5-axis milling machine of table rotatingtilting YH Jung - ?2002 - ?Cited by 125.pdf
syms ThetaC ThetaA Tx Ty Tz Dy Dz Px Py Pz;


T_CT = [1 0 0 Tx; 0 1 0 Ty+Dy; 0 0 1 Tz-Dz; 0 0 0 1]
T_BC = [1 0 0 0; 0 cos(ThetaA) -sin(ThetaA) 0; 0 sin(ThetaA) cos(ThetaA) 0; 0 0 0 1]
T_AB = [1 0 0 0; 0 1 0 -Dy; 0 0 1 +Dz; 0 0 0 1]


T_WA = [cos(ThetaC) -sin(ThetaC) 0 0; sin(ThetaC) cos(ThetaC) 0 0; 0 0 1 0; 0 0 0 1]
T_WT = T_WA*T_AB*T_BC*T_CT


T_AW =  [cos(ThetaC) sin(ThetaC) 0 cos(ThetaC)*Px+sin(ThetaC)*Py; -sin(ThetaC) cos(ThetaC) 0 -sin(ThetaC)*Px+cos(ThetaC)*Py; 0 0 1 Pz; 0 0 0 1]
T_BA = [1 0 0 0; 0 1 0 Dy; 0 0 1 -Dz; 0 0 0 1]
T_CB = [1 0 0 0; 0 cos(ThetaA) sin(ThetaA) 0; 0 -sin(ThetaA) cos(ThetaA) 0; 0 0 0 1]
T_GC = [1 0 0 0; 0 1 0 -Dy; 0 0 1 Dz; 0 0 0 1]

T_GW = T_GC*T_CB*T_BA*T_AW

%%