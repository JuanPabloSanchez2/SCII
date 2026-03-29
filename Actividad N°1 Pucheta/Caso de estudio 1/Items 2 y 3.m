% Actividad Practica N°1 Representación de sistemas y controladores
% Caso de estudio 1. Sistema de dos variables de estado
% Items[2 y 3]


% Importamos los datos
nombre_archivo = 'Curvas_Medidas_RLC_2026.xls';

% Leemos los datos de la Hoja 1. 
datos = readmatrix(nombre_archivo, 'Sheet', 1);

% Separamos las columnas en vectores
t_medido = datos(:, 1);     % Vector de tiempo
i_medido = datos(:, 2);     % Vector de corritente
vc_medido = datos(:, 3);    % Vector de tensión en el capacitor
vin_medido = datos(:, 4);   % Vector de tensión de entrada
vr_medido = datos(:, 5);    % Vector de tensión de salida

% --- Visualización inicial ---
figure('Name', 'Respuesta Medida del Sistema RLC', 'Position', [100, 100, 800, 500]);
plot(t_medido, vc_medido, 'b', 'LineWidth', 1.5);
title('Tensión en el Capacitor frente a una Entrada Escalón');
xlabel('Tiempo (s)');
ylabel('Tensión V_c (V)');
grid on;

% Obtención de los valores de RLC 

% Resistencia (Ley de Ohm)
R_calc = i_medido \ vr_medido;

% Capacitancia (Integral de la corriente)
% cumtrapz hace la integral numérica punto a punto
integral_i = cumtrapz(t_medido, i_medido); 
C_calc = vc_medido \ integral_i; % Resolvemos C * Vc = integral(i)

% Inductancia (Integral de la tensión del inductor)
% Primero calculamos V_L por Kirchhoff
vL_medido = vin_medido - vr_medido - vc_medido; 
integral_vL = cumtrapz(t_medido, vL_medido);
L_calc = i_medido \ integral_vL; % Resolvemos L * i = integral(vL)

% Cálculo de a y b
a_calc = R_calc / L_calc;
b_calc = 1 / (L_calc * C_calc);

fprintf('--- RESULTADOS POR LEYES FUNDAMENTALES ---\n');
fprintf('a (R/L) = %.2f\n', a_calc);
fprintf('b (1/LC) = %.2f\n', b_calc);

fprintf('--- RESULTADOS DE LOS COMPONENTES ---\n');
fprintf('El valor calculado de la Resistencia (R) es: %.2f Ohms\n', R_calc);
fprintf('El valor calculado de la Inductancia (L) es: %.4f Henry (%.2f mH)\n', L_calc, L_calc*1000);
fprintf('El valor calculado de la Capacitancia (C) es: %.2e Farads (%.2f uF)\n', C_calc, C_calc*1e6);


% Item [3]

% 1. Definir la función de transferencia de la corriente
% La relación I(s)/Vin(s) en un circuito RLC serie es:
% I(s) / Vin(s) = (s/L) / (s^2 + (R/L)s + 1/LC)
num_I = [1/L_calc, 0];
den_I = [1, a_calc, b_calc]; 
sys_I = tf(num_I, den_I);

% 2. Simular la respuesta de la corriente usando la entrada medida
% (Simulamos todo el vector de tiempo para que la dinámica transitoria sea correcta)
i_simulado = lsim(sys_I, vin_medido, t_medido);

% 3. Encontrar el índice donde el tiempo es >= 0.05 segundos
idx_val = find(t_medido >= 0.05);

% Recortar los vectores de tiempo y corrientes usando el índice
t_val = t_medido(idx_val);
i_val_medido = i_medido(idx_val);
i_val_simulado = i_simulado(idx_val);

% 4. Superponer las gráficas para validar el resultado
figure('Name', 'Validación de Corriente', 'Position', [150, 150, 800, 500]);
plot(t_val, i_val_medido, 'b', 'LineWidth', 2);
hold on;
plot(t_val, i_val_simulado, 'r--', 'LineWidth', 2);
title('Validación de la Serie de Corriente (t \geq 0.05 s)');
xlabel('Tiempo (s)');
ylabel('Corriente i(t) (A)');
legend('Corriente Medida', 'Corriente Simulada (Modelo RLC)');
grid on;
hold off;
