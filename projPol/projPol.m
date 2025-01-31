% Sistema:
G = tf(1, conv([1 2], [1 0.2 1]));
T = 1;
Gz = c2d(G, T, 'zoh');

% Especificações de projeto:
ts = 10;
os = 0.1;

% Parâmetros de projeto do sistema:
tau = ts/4;
R = exp(-T/tau);
zeta = sqrt((log(os)^2)/((pi^2)+(log(os)^2)));
theta = sqrt(((log(R)^2) - ((zeta^2)*(log(R)^2)))/(zeta^2));

% Polos desejados:
a = R*cos(theta);
b = R*sin(theta);
pd1 = complex(a, b);
pd2 = complex(a, -b);

% Polinômio característico:
%polmf = conv([1 -pd1], [1 -pd2]);
%polmf = conv(polmf, [1 -pd2]);
polmf = [1 -2*a (a^2 + b^2)];
polmf = conv(polmf, [1 0]);
polmf = conv(polmf, [1 0]);
polmf = conv(polmf, [1 0]);

% Definindo o vetor D:
D = fliplr(polmf)';

% Definindo a matriz E:
A = Gz.den{1};
B = Gz.num{1};
n = size(Gz.den{1});
n = n(2)-1;
e11 = zeros(n,n);
e12 = zeros(n,n);
e21 = zeros(n,n);
e22 = zeros(n,n);
for i=[1:1:n]
    for j=[1:1:n]
        if(j<=i)
            k = (i-j);
            e11(i,j) = A(n-k+1);
            e12(i,j) = B(n-k+1);
        end
        if(j>=i)
            k = (j-i);
            e21(i,j) = A(k+1);
            e22(i,j) = B(k+1);
        end
    end
end
E = [e11 e12; e21 e22];

% Cáclulo dos controladores:
M = inv(E)*D;
alpha = fliplr(M(1:n)');
beta = fliplr(M(n+1:2*n)');

% Função de transferência do controlador:
C = tf(beta, alpha, T);

% Malha fechada:
MF = feedback(C*Gz, 1);
refy = feedback(C*Gz, 1)/dcgain(MF);
%Sinal de controle:
refu = feedback(C, Gz)/dcgain(MF);

%Plot:
figure
subplot(2,1,1);
[y,t] = step(refy);
plot(t, y, 'b', 'LineWidth', 1.5);
grid
xlabel('Tempo [s]');
ylabel('Saída do Sistema');
xlim([0,20]);
subplot(2,1,2);
[u,t] = step(refu);
plot(t, u, 'r', 'LineWidth', 1.5);
grid
xlabel('Tempo [s]');
ylabel('Sinal de Controle');
xlim([0,20]);
exportgraphics(gcf, 'mf.eps', 'ContentType', 'vector');

%Bode controlador:
[mag, phase, omega] = bode(C);
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
exportgraphics(gcf, 'bode_cont.eps', 'ContentType', 'vector');

%Bode sistema:
[mag, phase, omega] = bode(Gz);
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
exportgraphics(gcf, 'bode_sys.eps', 'ContentType', 'vector');

%Bode ref -> y:
[mag, phase, omega] = bode(refy);
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
exportgraphics(gcf, 'bode_mf.eps', 'ContentType', 'vector');

%Bode ref -> u:
[mag, phase, omega] = bode(refu);
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
exportgraphics(gcf, 'bode_refu.eps', 'ContentType', 'vector');