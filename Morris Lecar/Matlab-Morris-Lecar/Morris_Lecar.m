
%Morris Lecar model 
%Written Stanislav Mircic
%2023 Backyard Brains


%Time step
dt = 1;


 %initial values 
 Iapp = 65;
 V = 0;
 n = 0;


%Reversal potentials
 ECa = 120; %mV
 EK = -84; %mV
 EL = -60; %mV
 Cm = 20; %microF/cm2



 gL = 2; %mS/cm2
 gK = 8;
 gCa = 4;
 gKCa = 0.75;

 V1 = -1.2;
 V2 = 18;
 
 
 %SNLC
 V3 = 12;
 V4 = 17.4;
 phi = 0.067;
 gCa = 4;
 

%  %Hopf
%  V3 = 2;
%  V4 = 30;
%  phi = 0.04;
%  gCa = 4.4;



%  %Homoclinic
%  V3 = 12;
%  V4 = 17.4;
%  phi = 0.23;
%  gCa = 4;
%  
 
 randomCurrent = 0;


 spike = false;
 photodiodeInputCurrent=0;



voltage = [];
input = [];
 for i=1:10000
    Iapp = 0;

    analogInput = 0;
    %get current from pot
    if(i>500)
%         if(mod(i,10000)<6000)
%             analogInput = 60;
%         else
%             analogInput = 0;
%         end
        analogInput = 60;
        
    end
    Iapp = Iapp + analogInput;
    input = [input analogInput];

   
    minf = 0.5*(1+tanh((V-V1)/V2));
    tau_n = 1.0/cosh((V-V3)/(2*V4));
    ninf = 0.5*(1+tanh((V-V3)/V4));
    
    
    V = V + ((-gL*(V-EL) - gK*n*(V-EK) - gCa*minf*(V-ECa)+Iapp)/Cm)*dt;
    n = n + phi*((ninf-n)/tau_n)*dt;
    
    
    voltage = [voltage V];
 end
    plot(voltage)
    hold on 
    plot(input)