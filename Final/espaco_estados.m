% Sistema:
G = tf(1, conv([1 2], [1 0.2 1]));

%Discretização:
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
polmf = [1 -2*a (a^2 + b^2)];
polmf = conv(polmf, [1 0]);

%Ganhos de realimentação de estado:
K = polmf - Gz.den{1};
K = fliplr(K(2:end));
disp(K);
save K.mat K