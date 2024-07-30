% Parámetros del convertidor Buck no ideal
V_g = 20; % Voltaje de entrada [V]
C = 0.1e-3; % Capacitancia [F]
L = 1e-3; % Inductancia [H]
R_C = 0.01; % Resistencia interna del capacitor [Ohm]
R_L = 0.1; % Resistencia interna del inductor [Ohm]
R_Load = 10; % Resistencia de carga [Ohm]

% Parámetros del controlador PID
k_P = 0.8; % Ganancia proporcional inicial
k_I = 0.4; % Ganancia integral inicial
k_D = 0.1; % Ganancia derivativa inicial

% Parámetros de la red neuronal
gamma = 0.0001; % Reducir la tasa de aprendizaje para estabilidad
delta_1 = 0.5; % Parámetro del Lyapunov
delta_2 = 0.5; % Parámetro del Lyapunov

% Límites para las señales de control
u_min = -1000;
u_max = 1000;

% Límites para los pesos de la red neuronal
w_min = -10;
w_max = 10;

% Simulación
T = 0.001; % Reducir el paso de tiempo para mayor precisión
t_final = 3; % Tiempo final de la simulación
t = 0:T:t_final; % Vector de tiempo

% Variables de estado
i_L = zeros(size(t));
v_c = zeros(size(t));
v_out = zeros(size(t));
e = zeros(size(t)); % Error
u = zeros(size(t)); % Señal de control
w_k = [k_P; k_I; k_D]; % Pesos de la red neuronal

% Entrada deseada
v_ref = 3.3; % Voltaje de referencia [V]

% Simulación
for k = 3:length(t)
    % Error
    e(k) = v_ref - v_out(k-1);

    % Control PID incremental
    delta_u = w_k(1)*(e(k) - e(k-1)) + w_k(2)*e(k) + w_k(3)*(e(k) - 2*e(k-1) + e(k-2));
    u(k) = min(max(u(k-1) + delta_u, u_min), u_max); % Limitar señal de control

    % Actualización de la red neuronal
    y_k = u(k);
    d_k = v_ref;
    
    % Regla delta para actualizar los pesos
    for i = 1:length(w_k)
        w_k(i) = min(max(w_k(i) + gamma * (d_k - y_k) * y_k, w_min), w_max); % Actualización de pesos con límites
    end
    
    % Actualización del Lyapunov
    Psi_k = 2*delta_1*delta_2*(e(k) + delta_u) + 2*delta_2*w_k(i);
    Phi_k = delta_1*(2*e(k) + delta_u) + 2*delta_1*delta_2*w_k(i);
    w_k = min(max(w_k - gamma/delta_2 * (Psi_k - Phi_k * y_k), w_min), w_max); % Actualización de Lyapunov con límites
    
    % Actualización de las ganancias PID
    k_P = w_k(1);
    k_I = w_k(2);
    k_D = w_k(3);

    % Dinámica del convertidor Buck
    di_L = (V_g - v_c(k-1) - R_L*i_L(k-1))/L;
    dv_c = (i_L(k-1) - v_c(k-1)/R_Load - v_c(k-1)/R_C)/C;

    % Integración
    i_L(k) = i_L(k-1) + T*di_L;
    v_c(k) = v_c(k-1) + T*dv_c;

    % Salida del convertidor
    v_out(k) = v_c(k) + R_C*(i_L(k) - v_c(k)/R_Load);
end

% Gráficas de resultados
figure;
subplot(3,1,1);
plot(t, v_out);
title('Voltaje de Salida');
xlabel('Tiempo [s]');
ylabel('Voltaje [V]');
xlim([0 0.03]); % Limitar eje X
ylim([0, 10]); % Limitar eje Y para visualizar mejor

subplot(3,1,2);
plot(t, i_L);
title('Corriente en el Inductor');
xlabel('Tiempo [s]');
ylabel('Corriente [A]');
xlim([0 0.03]); % Limitar eje X
ylim([-100, 100]); % Limitar eje Y para visualizar mejor

subplot(3,1,3);
plot(t, u);
title('Señal de Control');
xlabel('Tiempo [s]');
ylabel('u(t)');
xlim([0 0.03]); % Limitar eje X
ylim([u_min, u_max]); % Limitar eje Y

% Gráfica del error
figure;
plot(t, e);
title('Error de Seguimiento');
xlabel('Tiempo [s]');
ylabel('Error [V]');
xlim([0 0.03]); % Limitar eje X

% Simulación bajo variaciones de voltaje de entrada
V_g_var = 10; % Voltaje de entrada variado [V]
i_L_var = zeros(size(t));
v_c_var = zeros(size(t));
v_out_var = zeros(size(t));
e_var = zeros(size(t)); % Error
u_var = zeros(size(t)); % Señal de control
w_k_var = [k_P; k_I; k_D]; % Pesos de la red neuronal

for k = 3:length(t)
    % Error
    e_var(k) = v_ref - v_out_var(k-1);

    % Control PID incremental
    delta_u = w_k_var(1)*(e_var(k) - e_var(k-1)) + w_k_var(2)*e_var(k) + w_k_var(3)*(e_var(k) - 2*e_var(k-1) + e_var(k-2));
    u_var(k) = min(max(u_var(k-1) + delta_u, u_min), u_max); % Limitar señal de control

    % Actualización de la red neuronal
    y_k = u_var(k);
    d_k = v_ref;
    
    % Regla delta para actualizar los pesos
    for i = 1:length(w_k_var)
        w_k_var(i) = min(max(w_k_var(i) + gamma * (d_k - y_k) * y_k, w_min), w_max); % Actualización de pesos con límites
    end
    
    % Actualización del Lyapunov
    Psi_k = 2*delta_1*delta_2*(e_var(k) + delta_u) + 2*delta_2*w_k_var(i);
    Phi_k = delta_1*(2*e_var(k) + delta_u) + 2*delta_1*delta_2*w_k_var(i);
    w_k_var = min(max(w_k_var - gamma/delta_2 * (Psi_k - Phi_k * y_k), w_min), w_max); % Actualización de Lyapunov con límites
    
    % Actualización de las ganancias PID
    k_P = w_k_var(1);
    k_I = w_k_var(2);
    k_D = w_k_var(3);

    % Dinámica del convertidor Buck
    di_L = (V_g_var - v_c_var(k-1) - R_L*i_L_var(k-1))/L;
    dv_c = (i_L_var(k-1) - v_c_var(k-1)/R_Load - v_c_var(k-1)/R_C)/C;

    % Integración
    i_L_var(k) = i_L_var(k-1) + T*di_L;
    v_c_var(k) = v_c_var(k-1) + T*dv_c;

    % Salida del convertidor
    v_out_var(k) = v_c_var(k) + R_C*(i_L_var(k) - v_c_var(k)/R_Load);
end

% Gráficas de resultados bajo variaciones de voltaje de entrada
figure;
subplot(3,1,1);
plot(t, v_out_var);
title('Voltaje de Salida con V_g = 10V');
xlabel('Tiempo [s]');
ylabel('Voltaje [V]');
xlim([0 0.03]); % Limitar eje X
ylim([0, 10]); % Limitar eje Y para visualizar mejor

subplot(3,1,2);
plot(t, i_L_var);
title('Corriente en el Inductor con V_g = 10V');
xlabel('Tiempo [s]');
ylabel('Corriente [A]');
xlim([0 0.03]); % Limitar eje X
ylim([-100, 100]); % Limitar eje Y para visualizar mejor

subplot(3,1,3);
plot(t, u_var);
title('Señal de Control con V_g = 10V');
xlabel('Tiempo [s]');
ylabel('u(t)');
xlim([0 0.03]); % Limitar eje X
ylim([u_min, u_max]); % Limitar eje Y

% Gráfica del error bajo variaciones de voltaje de entrada
figure;
plot(t, e_var);
title('Error de Seguimiento con V_g = 10V');
xlabel('Tiempo [s]');
ylabel('Error [V]');
xlim([0 0.03]); % Limitar eje X

% Simulación con perturbación en la entrada
perturb = 0.1; % Perturbación en voltaje [V]
v_out_perturb = v_out + perturb;

% Gráficas de resultados con perturbación
figure;
subplot(3,1,1);
plot(t, v_out_perturb);
title('Voltaje de Salida con Perturbación');
xlabel('Tiempo [s]');
ylabel('Voltaje [V]');
xlim([0 0.03]); % Limitar eje X
ylim([0, 10]); % Limitar eje Y para visualizar mejor

subplot(3,1,2);
plot(t, i_L);
title('Corriente en el Inductor con Perturbación');
xlabel('Tiempo [s]');
ylabel('Corriente [A]');
xlim([0 0.03]); % Limitar eje X
ylim([-100, 100]); % Limitar eje Y para visualizar mejor

subplot(3,1,3);
plot(t, u);
title('Señal de Control con Perturbación');
xlabel('Tiempo [s]');
ylabel('u(t)');
xlim([0 0.03]); % Limitar eje X
ylim([u_min, u_max]); % Limitar eje Y

% Gráfica del error con perturbación
figure;
plot(t, v_out_perturb - v_ref);
title('Error de Seguimiento con Perturbación');
xlabel('Tiempo [s]');
ylabel('Error [V]');
xlim([0 0.03]); % Limitar eje X
