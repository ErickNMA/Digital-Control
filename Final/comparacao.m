% Sistema:
G = tf(1, conv([1 2], [1 0.2 1]));

%Discretização:
T = 1;
Gz = c2d(G, T, 'zoh');

%Espaço de estados:
n = size(Gz.den{1});
n = n(2)-1;
%Matriz A:
A = zeros(n,n);
flipden = fliplr(Gz.den{1});
for i=[1:1:n]
    if(i~=n)
        A(i,i+1) = 1;
    end
    A(n,i) = -flipden(i);
end
%Matriz B:
B = zeros(n,1);
B(n,1) = 1;
%Matriz C:
num = Gz.num{1};
C = num(2:end);
%Matriz D:
D = 0;

%Importando os controladores projetados:
load("Gpid.mat");
load("Gpol.mat");
load("P.mat");
load("K.mat");

%Síntese Direta:
MFPID = feedback(Gpid*Gz, 1);
disp("PID");
stepinfo(MFPID)
%Bode sistema:
[mag, phase, omega] = bode(MFPID);
mag_dB = 20*log10(squeeze(mag));
phase = squeeze(phase);
omega = squeeze(omega);
figure
subplot(2,1,1);
semilogx(omega, mag_dB, 'b', 'LineWidth', 1.5);
grid
xlabel('Frequência (rad/s)');
ylabel('Magnitude (dB)');
subplot(2,1,2);
semilogx(omega, phase, 'b', 'LineWidth', 1);
grid
xlabel('Frequência (rad/s)');
ylabel('Fase (°)');
exportgraphics(gcf, 'bode_pid.eps', 'ContentType', 'vector');

%Polinomial:
MFPOL = P*feedback(Gpol*Gz, 1);
disp("POLINOMIAL");
stepinfo(MFPOL)
%Bode sistema:
[mag, phase, omega] = bode(MFPOL);
mag_dB = 20*log10(squeeze(mag));
phase = squeeze(phase);
omega = squeeze(omega);
figure
subplot(2,1,1);
semilogx(omega, mag_dB, 'b', 'LineWidth', 1.5);
grid
xlabel('Frequência (rad/s)');
ylabel('Magnitude (dB)');
subplot(2,1,2);
semilogx(omega, phase, 'b', 'LineWidth', 1);
grid
xlabel('Frequência (rad/s)');
ylabel('Fase (°)');
exportgraphics(gcf, 'bode_pol.eps', 'ContentType', 'vector');

%Realimentação de Estados:
Amf = A;
Amf(n,:) = A(n,:)-K;
MFSS = ss(Amf,0.84*B,C,D, T);
disp("Realimentação de Estados");
stepinfo(MFSS)
%Bode sistema:
[mag, phase, omega] = bode(MFSS);
mag_dB = 20*log10(squeeze(mag));
phase = squeeze(phase);
omega = squeeze(omega);
figure
subplot(2,1,1);
semilogx(omega, mag_dB, 'b', 'LineWidth', 1.5);
grid
xlabel('Frequência (rad/s)');
ylabel('Magnitude (dB)');
subplot(2,1,2);
semilogx(omega, phase, 'b', 'LineWidth', 1);
grid
xlabel('Frequência (rad/s)');
ylabel('Fase (°)');
exportgraphics(gcf, 'bode_ss.eps', 'ContentType', 'vector');