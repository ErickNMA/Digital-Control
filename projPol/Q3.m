%Controlador discreto projetado:
T = 1;
C = tf([-1.6347 0.1839 0.0168], [1 0.1331 0.005], T);

figure('Position', [100, 100, 600, 400]);
hold
grid
legend('Location', 'southeast');
%Variação do parâmetro P:
plim = 0.7399;
p0 = -plim/2;
pf = plim/2;
for p=linspace(p0,pf,11)
    G = tf([1], conv([1 2+p], [1 0.2 1]));
    Gd = c2d(G, T, 'zoh');
    MF = -0.5873*feedback(C*Gd,1);
    [y,t] = step(MF);
    if(p<0)
        plot(t, y, '--', 'LineWidth', 1.5, 'DisplayName', sprintf("p = %.g",p));
    end
    if(p>0)
        plot(t, y, 'LineWidth', 1.5, 'DisplayName', sprintf("p = %.g",p));
    end
    if(p==0)
        plot(t, y, '-.k', 'LineWidth', 1.5, 'DisplayName', sprintf("p = %.g",p));
    end
    
end
xlabel('Tempo [s]');
ylabel('Saída do Sistema');
exportgraphics(gcf, 'pvar1.eps', 'ContentType', 'vector');

figure('Position', [100, 100, 600, 400]);
hold
grid
legend('Location', 'southeast');
%Variação do parâmetro P:
plim = 0.7399;
p0 = -plim/2;
pf = plim/2;
for p=linspace(p0,pf,11)
    G = tf([1], conv([1 2+p], [1 0.2 1]));
    Gd = c2d(G, T, 'zoh');
    MF = -0.5873*feedback(C*Gd,1);
    k = 1/dcgain(MF);
    MF = k*MF;
    [y,t] = step(MF);
    if(p<0)
        plot(t, y, '--', 'LineWidth', 1.5, 'DisplayName', sprintf("p = %.g | k = %g",p,k));
    end
    if(p>0)
        plot(t, y, 'LineWidth', 1.5, 'DisplayName', sprintf("p = %.g | k = %g",p,k));
    end
    if(p==0)
        plot(t, y, '-.k', 'LineWidth', 1.5, 'DisplayName', sprintf("p = %.g | k = %g",p,k));
    end
    
end
xlabel('Tempo [s]');
ylabel('Saída do Sistema');
exportgraphics(gcf, 'pvar2.eps', 'ContentType', 'vector');