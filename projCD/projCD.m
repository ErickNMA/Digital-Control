%Sistema contínuo, em malha aberta:
G = tf([5], conv([1 0.1], [1 0.5])); %função de transferência
[z,p,k] = zpkdata(G); %formato zpk
Gzpk = zpk(z,p,k,'DisplayFormat','timeconstant'); %formato em constante de tempo
%figure
%step(G); %degrau em malha aberta
Ginfo = stepinfo(G); %características do transitório
disp(Ginfo);

%Especificações de projeto:
disp('Especificações do Projeto:');
ST = 0.75*Ginfo.SettlingTime;
fprintf('ST < %g\n', ST); %redução do tempo de acomodação em 25% da malha aberta
disp('OS < 5%'); %overshoot ótimo de 5%
T = 1/5*max(abs(p{1}));
fprintf('T = %g\n', T); %intervalo de amostragem: 1/5 da constante de tempo mais rápida

%Discretização do sistema, com ZOH:
Gz = c2d(G, T, 'zoh');

%Projeto de controlador Proporcional (overshoot):
%zetawn = 4/ST;
%fprintf('zeta*wn > %g\n', zetawn);
OS = 0.05;
zeta = (-log(OS)/sqrt((pi^2)+(log(OS)^2)));
fprintf('zeta > %g\n', zeta);
%figure
%rlocus(Gz); %LGR do ramo direto
%grid
kp1 = 0.025; %ganho obtido pelo LGR
Tp = feedback(kp1*Gz, 1); %malha fechada do controlador proporcional
%figure
%step(Tp);

%Projeto de controlador Proporcional-Integral:
%sisotool(Gz);
load('Cpi.mat');
Tpi = feedback(Cpi*Gz, 1); %malha fechada do controlador PI