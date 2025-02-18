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
Gpol = tf(beta, alpha, T);
save Gpol.mat Gpol

% Malha fechada:
MF = feedback(Gpol*Gz, 1);
P = 1/dcgain(MF);
save P.mat P
refy = P*MF;

%Step:
stepinfo(refy)
step(refy);