%Limpando Workspace e Command Window:
clear
clc

%Função de transferência da planta:
Gps = tf([10], [5 1]);
Gps

%Transformada Z com ZOH:
Gpz = c2d(Gps, 0.5, 'zoh');
Gpz

%Expressão do analítica do erro para um controle proporcional:
syms z
Gsym = vpa(poly2sym(cell2mat(Gpz.num),z)/poly2sym(cell2mat(Gpz.den),z));
syms k
Es = (1/(1+k*Gsym));
E = limit(Es, z, 1);
disp('Expressão do erro em função do ganho: ')
disp(vpa(simplify(E), 5))
ek = symfun(E, k);

%Plot do erro em função do ganho:
figure('Name', 'Erro X Ganho')
kvals = linspace(0, 2);
plot(kvals, ek(kvals))
grid
ylabel('Erro')
xlabel('Ganho')

%Lugar das raízes:
figure('Name', 'LGR')
rlocus(Gpz)

%Ganho proporcional:
kp = 0.951;

%Malha fechada:
Tz = feedback(kp*Gpz, 1);
Tz

%Comparação entre a malha aberta e fechada:
figure('Name', 'Malha Aberta x Malha Fechada')
hold
grid
[y, t] = step(Gpz);
plot(t, y)
disp('Erro MA: ')
disp(1-y(end))
[y, t] = step(Tz);
plot(t, y)
legend('Malha Aberta', 'Malha Fechada')
xlabel('Tempo [s]')
disp('Erro MF: ')
disp(1-y(end))