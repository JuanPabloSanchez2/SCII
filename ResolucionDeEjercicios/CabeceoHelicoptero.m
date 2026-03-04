% Resolucion del ejercicio utilizando Gemini IA
% La resolucion requirio varias iteraciones hasta lograr los requerimiento del ejercicio (overshoot y ts).

% --- Solución: Cancelación Polo-Cero en Lazo Cerrado ---

% 1. Definición del sistema
num = [0 0 6.27 0.015366]; 
den = [1 0.4348 -0.007656 0.10878];
G_s = tf(num, den);
disp('Función de Transferencia del Helicóptero:');
G_s
[A, B, C, D] = tf2ss(num, den);
sys_ss = ss(A, B, C, D);
disp('Matrices del Espacio de Estados:');
A, B, C, D

% 2. Identificación del "Cero Lento" problemático
% Raíz del numerador: 6.27*s + 0.015366 = 0
cero_planta = -0.015366 / 6.27; 

% 3. Selección de Polos
% Colocamos el primer polo EXACTAMENTE sobre el cero para cancelarlo
p1 = cero_planta; 

% Colocamos los otros dos polos para cumplir: ts < 1.5s y Overshoot < 20%
% Usaremos wn = 5 y zeta = 0.7 (asegura un overshoot ~5% y ts ~0.8s)
zeta = 0.7;
wn = 5;
p2 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
p3 = -zeta*wn - 1i*wn*sqrt(1-zeta^2);

polos_deseados = [p1, p2, p3];

% 4. Cálculo de la matriz K y la ganancia Nbar
K = place(A, B, polos_deseados);
A_cl = A - B*K;

sys_cl_temp = ss(A_cl, B, C, D);
Nbar = 1 / dcgain(sys_cl_temp); % Ahora Nbar será un valor pequeño y razonable

% 5. Sistema Final
sys_final = ss(A_cl, B*Nbar, C, D*Nbar);

% --- Simulación ---
figure;
step(sys_final);
title('Respuesta al Escalón: Cancelación Polo-Cero');
xlabel('Tiempo (segundos)');
ylabel('Amplitud (Ángulo de Cabeceo)');
grid on;

% Verificar que se cumplen estrictamente los 3 requisitos
disp('Características de la respuesta final:');
stepinfo(sys_final)
