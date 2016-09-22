function [ m_bDrawResult] = ExperimentGraph( FileInput )
    %EXPERIMENTGRAPH Summary of this function goes here
    %   Detailed explanation goes here
    disp('Start DrawGraph from File '+FileInput); 
    num_samp = 5001;
    m_TimeTotal = 0.0;
    m_TimeNow = zeros(1,num_samp);
    vec_realx = zeros(1,num_samp);
	vec_realx_1 = zeros(1,num_samp);
	vec_refr = zeros(1,num_samp);
	vec_refr_1 = zeros(1,num_samp);
	vec_el = zeros(1,num_samp);
	vec_ew = zeros(1,num_samp);
	vec_u = zeros(1,num_samp);
	vec_eVoltSup = zeros(1,num_samp);
	vec_Abso_ec = zeros(1,num_samp);
    FID_in = fopen('R5Time10W0.rme','r'); 
    m_TimeTotal = fread(FID_in,1,'double');
    
  %Start read data to variable 
    for i=1:num_samp 
    m_TimeNow(i) = fread(FID_in,1,'double'); 
    end 
    fclose(FID_in);     
    
end

