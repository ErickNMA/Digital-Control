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
%pd1 = complex(a, b);
%pd2 = complex(a, -b);
%pd3 = complex(0, 0);

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

%Equação a diferenças do controlador:


% Função de transferência do controlador:
C = tf(beta, alpha, T);

% Malha fechada:
MF = feedback(C*Gz, 1);
refy = feedback(C*Gz/dcgain(MF), dcgain(MF));
figure
hold on
%step(MF)
step(refy)
hold off

%Sinal de controle:
refu = feedback(C/dcgain(MF), Gz*dcgain(MF));
figure
step(refu)

%Bode controlador:
figure
bode(C)

%Bode sistema:
figure
bode(Gz)

%Bode ref -> y:
figure
bode(refy)

%Bode ref -> u:
figure
bode(refu)