% Actividad Practica N°1 Representación de sistemas y controladores
% Caso de estudio 1. Sistema de dos variables de estado
% Item[1]


% --- Parámetros del circuito ---
R = 2200;       % Resistencia en Ohms
L = 500e-3;     % Inductancia en Henry
C = 10e-6;      % Capacitancia en Farads

% --- Matrices de Espacio de Estados ---
A = [-R/L, -1/L; 
      1/C,    0];
  
B = [1/L; 
       0];
   
% Usamos el nombre 'C_mat' para no sobrescribir la variable del capacitor 'C'
C_mat = [R, 0]; 

D = 0;

% Creación del modelo LTI en espacio de estados
sys = ss(A, B, C_mat, D);

% --- Configuración de la Simulación ---
% Vector de tiempo: Simulamos 100 ms para observar 5 ciclos completos
dt = 1e-5; 
t = 0:dt:0.1;

% Señal de entrada u(t): 12V cambiando de signo cada 10 ms (0.01 s).
% Esto equivale a un periodo de 20 ms, es decir, una frecuencia de 50 Hz.
frecuencia = 1 / 0.02; 
% Usamos sign(sin(...)) para generar la onda cuadrada sin necesitar el Signal Processing Toolbox
u = 12 * sign(sin(2 * pi * frecuencia * t));

% Simulación lineal del sistema
[y, t_out, x] = lsim(sys, u, t);

% --- Gráficas ---
figure('Name', 'Dinámica del Sistema RLC', 'Position', [100, 100, 800, 600]);

% 1. Gráfica de Entrada vs Salida
subplot(2,1,1);
plot(t_out, u, 'b--', 'LineWidth', 1.5); hold on;
plot(t_out, y, 'r', 'LineWidth', 1.5);
title('Respuesta del Sistema: Entrada v_e(t) vs Salida v_r(t)');
xlabel('Tiempo (s)');
ylabel('Tensión (V)');
legend('Entrada v_e(t)', 'Salida v_r(t) [Tensión en R]', 'Location', 'best');
grid on;

% 2. Gráfica de las Variables de Estado internas
subplot(2,1,2);
% Estado x1 (Corriente en mA)
yyaxis left;
plot(t_out, x(:,1) * 1000, 'g', 'LineWidth', 1.5); 
ylabel('Corriente i(t) [mA]');

% Estado x2 (Tensión en el capacitor)
yyaxis right;
plot(t_out, x(:,2), 'm', 'LineWidth', 1.5);
ylabel('Tensión V_c(t) [V]');

title('Dinámica de las Variables de Estado (x_1 y x_2)');
xlabel('Tiempo (s)');
legend('Estado x_1 (Corriente)', 'Estado x_2 (Tensión Cap.)', 'Location', 'best');
grid on;
