%Configurações:
sympref('FloatingPointOutput', true);
syms z kp ki kd

%Sistema contínuo, em malha aberta:
G = tf([75], conv([1 10], [1 15])); %função de transferência

%Discretização do sistema, com ZOH:
T = 11e-3;
Gz = c2d(G, T, 'zoh');
[gzero, gpole, ggain] = zpkdata(Gz);

%Zeros desejados iguais aos polos da planta:
polos = pole(Gz);
zds = expand((z-polos(1))*(z-polos(2)));
coefszds = coeffs(zds);
%Valter:
coefzcont0 = ((2*kd)/(2*kp*T + ki*T^2 + 2*kd));
coefzcont1 = ((ki*T^2 - 2*kp*T - 4*kd)/(2*kp*T + ki*T^2 + 2*kd));
%Erick e Robson:
%coefzcont0 = ((kd*T)/(kp + 2*ki*T + kd*T));
%coefzcont1 = ((2*ki*T - kp - 2*kd*T)/(kp + 2*ki*T + kd*T));
eq1 = coefzcont0 - coefszds(1);
eq2 = coefzcont1 - coefszds(2);

%Ajuste do ganho k' pelo LGR:
ramodireto = Gz*tf(conv([1 -polos(1)], [1 -polos(2)]), conv([1 0], [1 -1]), T);
%rlocus(ramodireto);
%grid

%Relação do ganho obtido do LGR, com k':
klgr = 63.5;
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
%Gpid = tf(pid(Kp, Ki, Kd,'Ts',T,'IFormula','Trapezoidal', 'DFormula','BackwardEuler'));
%a0 = double(subs(coefzcont0, {kp, ki, kd}, {Kp, Ki, Kd}));
%a1 = double(subs(coefzcont1, {kp, ki, kd}, {Kp, Ki, Kd}));
%kconta = double(subs(klinha, {kp, ki, kd}, {Kp, Ki, Kd}));
%Gpid = kconta*tf([1 a1 a0], conv([1 0], [1 -1]), T);

%Controlador 1 (PI):
C1 = (Kp + (tf([1 1], [1 -1], T)*Ki*T/2));

%Controlador 2 (D):
C2 = (tf([1 -1], [1 0], T)*Kd/T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Análise frequencial:

%Bode do sistema:
[mag, phase, omega] = bode(G);
mag_dB = 20*log10(squeeze(mag));
phase = squeeze(phase);
omega = squeeze(omega);

%Frequência de corte do sistema:
fcsys = omega(find(mag_dB <= (max(mag_dB) - 3), 1));

%Frequência de Nyquist:
nyq = pi/T;

%Plot da resposta em frequência:
figure
subplot(2,1,1);
semilogx(omega, mag_dB, 'b', 'LineWidth', 1);
hold
plot([fcsys fcsys], [-100 20], 'c--', 'LineWidth', 1);
plot([nyq nyq], [-100 20], 'g--', 'LineWidth', 1);
text(fcsys+1, -90, sprintf('\\omega_s = %.2f rad/s', fcsys), 'HorizontalAlignment', 'left', 'Color', 'c');
text(nyq+20, -90, sprintf('\\omega_{nyq} = %.2f rad/s', nyq), 'HorizontalAlignment', 'left', 'Color', 'g');
grid
xlabel('Frequência (rad/s)');
ylabel('Magnitude (dB)');
subplot(2,1,2);
semilogx(omega, phase, 'b', 'LineWidth', 1);
hold
plot([fcsys fcsys], [-200 0], 'r--', 'LineWidth', 1);
plot([nyq nyq], [-200 0], 'g--', 'LineWidth', 1);
grid
xlabel('Frequência (rad/s)');
ylabel('Fase (°)');

%Projeto do filtro de segunda ordem:
 w0 = 10*nyq;
 a0 = w0^2;
 Q = 1;
 F = tf([a0], [1 w0/Q w0^2]);
 
%Bode do filtro:
[fmag, fphase, fomega] = bode(F);
fmag = 20*log10(squeeze(fmag));
fphase = squeeze(fphase);
fomega = squeeze(fomega);

%Frequência de corte do filtro:
fcfil = fomega(find(fmag <= (max(fmag) - 3), 1));

%Plot resposta em frequência:
subplot(2,1,1);
semilogx(fomega, fmag, 'r', 'LineWidth', 1);
plot([fcfil fcfil], [-100 20], 'm--', 'LineWidth', 1);
text(fcfil-1, -90, sprintf('\\omega_f = %.2f rad/s', fcfil), 'HorizontalAlignment', 'right', 'Color', 'm');
subplot(2,1,2);
semilogx(fomega, fphase, 'r', 'LineWidth', 1);
plot([fcfil fcfil], [-200 0], 'm--', 'LineWidth', 1);

%Malha fechada PID:
MFPID = feedback(Gpid*Gz, 1);

%Malha fechada PI + D:
MF = ((C1*Gz)/(1 + C1*Gz + C2*Gz));