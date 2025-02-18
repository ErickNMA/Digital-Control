%Configurações:
sympref('FloatingPointOutput', true);
syms z kp ki kd

%Sistema contínuo, em malha aberta:
G = tf([1], conv([1 2], [1 0.2 1])); %função de transferência

%Discretização do sistema, com ZOH:
T = 1;
Gz = c2d(G, T, 'zoh');
[gzero, gpole, ggain] = zpkdata(Gz);

%Zeros desejados iguais aos polos da planta:
polos = pole(Gz);
zds = expand((z-polos(1))*(z-polos(2)));
coefszds = coeffs(zds);
coefzcont0 = ((2*kd)/(2*kp*T + ki*T^2 + 2*kd));
coefzcont1 = ((ki*T^2 - 2*kp*T - 4*kd)/(2*kp*T + ki*T^2 + 2*kd));
eq1 = coefzcont0 - coefszds(1);
eq2 = coefzcont1 - coefszds(2);

%Ajuste do ganho k' pelo LGR:
ramodireto = Gz*tf(conv([1 -polos(1)], [1 -polos(2)]), conv([1 0], [1 -1]), T);
%rlocus(ramodireto);
%grid

%Relação do ganho obtido do LGR, com k':
klgr = 0.937;
%Valter:
klinha = ((ki*T/2) + kd/T + kp);
%Erick e Robson:
%klinha = ((2*ki*T) + kd*T + kp);
eq3 = klinha - klgr;

%Resolvendo o sistema de equações lineares:
sol = solve(eq1, eq2, eq3, kp, ki, kd);
Kp = double(sol.kp);
Ki = double(sol.ki);
Kd = double(sol.kd);
fprintf('\n \t Kp = %g \t Ki = %g \t Kd = %g \n', Kp, Ki, Kd);

%Função de transferência PID:
Gpid = (Kp + (tf([1 1], [1 -1], T)*Ki*T/2) + (tf([1 -1], [1 0], T)*Kd/T));
save Gpid.mat Gpid

%Malha Fechada:
MF = feedback(Gz*Gpid, 1);
disp(stepinfo(MF));
step(MF);
