% Resolucion del ejercicio utilizando Gemini IA

% --- 1. Definición de la Planta ---
s = tf('s');
% Función de transferencia sin retardo (Ecuación 1-36)
G_sin_retardo = 7031250 / ((s + 0.667)*(s + 5)*(s + 50)); 

% --- 2. Aproximación de Padé para el retardo ---
T_retardo = 0.2;
% Generamos la aproximación de Padé de 1er orden
[num_pade, den_pade] = pade(T_retardo, 1);
G_pade = tf(num_pade, den_pade);

% Planta aproximada para diseñar el controlador
G_aprox = G_sin_retardo * G_pade;

% --- 3. Diseño del Controlador PI ---
% Un PI asegura un error de estado estacionario de 0% (cumple < 10%)
% Usamos pidtune para encontrar parámetros estables automáticamente
opts = pidtuneOptions('DesignFocus', 'reference-tracking');
C_pi = pidtune(G_aprox, 'PI', opts);

disp('Parámetros del Controlador PI (Notarás valores muy bajos debido a la alta ganancia de la planta):');
C_pi

% --- 4. Simulación Discreta con Buffer Circular ---
% Aplicamos el "Tip" del documento para modelar el retardo real

h = 0.01;                  % Tiempo de integración / muestreo
t = 0:h:25;                % Vector de tiempo de simulación
N = length(t);

% Discretización de la planta en Espacio de Estados (para estabilidad numérica en el bucle)
sys_d = c2d(ss(G_sin_retardo), h, 'zoh');
Ad = sys_d.A; Bd = sys_d.B; Cd = sys_d.C; Dd = sys_d.D;

% Discretización del Controlador PI (Método Tustin)
C_d = c2d(C_pi, h, 'tustin');
[num_c, den_c] = tfdata(C_d, 'v');

% Inicialización del Buffer Circular para retener la acción de control 'u'
R_U = zeros(1, round(T_retardo/h)); 

% Variables de estado iniciales
x = [0; 0; 0];      
y_sim = zeros(1, N);
u_c_k1 = 0;         % Memoria de control (k-1)
e_k1 = 0;           % Memoria de error (k-1)
ref = 1;            % Referencia (escalón unitario)

for i = 2:N
    % Medición y cálculo de error
    e = ref - y_sim(i-1);
    
    % Ley de control PI (Ecuación en diferencias)
    u = u_c_k1 + num_c(1)*e + num_c(2)*e_k1;
    
    % Actualización de la memoria del controlador
    u_c_k1 = u;
    e_k1 = e;
    
    % --- Ejecución del Buffer Circular ---
    R_U = circshift(R_U, [0, -1]);  % Desplazamiento
    R_U(end) = u;                   % Entra el nuevo cálculo de control
    u_retardada = R_U(1);           % Sale el valor retardado 0.2s después
    
    % Dinámica de la Planta Discreta
    y_sim(i) = Cd * x + Dd * u_retardada;
    x = Ad * x + Bd * u_retardada;
end

% --- 5. Comparación y Gráficas ---
% Respuesta teórica usando la aproximación de Padé
sys_cl_pade = feedback(C_pi * G_aprox, 1);
[y_pade, t_pade] = step(sys_cl_pade, t);

figure;
plot(t_pade, y_pade, 'b', 'LineWidth', 1.5); hold on;
% Límites para el eje X (Tiempo)
xlim([0 10]); % Por ejemplo, para ver solo los primeros 10 segundos
% Límites para el eje Y (Amplitud)
ylim([-0.2 1.3]); % Ajusta para ver bien el pico y el pequeño bajón inicial
stairs(t, y_sim, 'r--', 'LineWidth', 1.5);
yline(1, 'k:');
legend('Lazo Cerrado con Padé', 'Retardo Real (Buffer Circular)', 'Referencia');
title('Control TCP/IP RED: Respuesta al Escalón');
xlabel('Tiempo (s)');
ylabel('Longitud de la Pila (y_t)');
grid on;
