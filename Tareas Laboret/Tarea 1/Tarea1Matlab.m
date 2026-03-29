% Tarea N°1 Laboret
% Sanchez Busso, Juan Pablo

% Especificaciones particulares
p1=0; 
p2=-2; 
cero=-10; 
ganancia=5;
sobrepaso=10;
%tiempo2%=3; 
error=0;
Tm=0.09;

% --- A LAZO ABIERTO ---

% Obtener la funcion de transferencia continua G(s)
G = zpk(cero, [p1 p2], ganancia);

% Hallar la FT de lazo abierto Gd(s)
Gd = c2d(G, Tm, 'zoh');

% Dibujar el mapa de polos y ceros
figure;
pzmap(G);
title('Mapa de Polos y Ceros - Continuo');

figure;
pzmap(Gd);
title('Mapa de Polos y Ceros - Discreto (Tm = 0.09)');

% Que ocurre con el mapa si se multiplica por 10 el periodo de muestreo?
Gd1 = c2d(G, 10*Tm, 'zoh');
figure;
pzmap(Gd1);
title('Mapa de Polos y Ceros - Discreto (Tm x 10 = 0.9)');

% Respuesta al escalon del sistema discreto y continuo
figure;
step(G);
title('Respuesta al Escalon - Continuo');

figure;
step(Gd);
title('Respuesta al Escalon - Discreto');

% Determinar Kp y error ante un escalon
Kp = dcgain(Gd);
error_escalon = 1 / (1 + Kp); % Calculo formal del error teorico

disp(['Constante Kp: ', num2str(Kp)]);
disp(['Error en estado estacionario (escalon): ', num2str(error_escalon)]);

% Verificar mediante respuesta al escalon a lazo cerrado
F = feedback(Gd, 1);
figure;
step(F);
title('Respuesta al Escalon - Lazo Cerrado');

% Verificacion del error ante una rampa de entrada
t = 0:Tm:100*Tm; % genera rampa
figure;
lsim(F, t, t);
title('Respuesta a Entrada Rampa - Lazo Cerrado');


% --- A LAZO CERRADO CON REALIMENTACION UNITARIA ---

% Graficar el lugar de raices del sistema continuo G(s) y discreto Gd(s)
figure;
rlocus(G);
title('Lugar de las Raices - Continuo G(s)');

figure;
rlocus(Gd);
title('Lugar de las Raices - Discreto Gd(s)');

% Que ocurre con la estabilidad relativa si se aumenta 10 veces el Tm?
figure;
rlocus(Gd1);
title('Lugar de las Raices - Discreto (Tm x 10)');
