% Limpieza de pantalla
clear all
close all
clc

% 1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 240;            % Aumentamos el tiempo para que alcance a recorrer todos los puntos
ts = 0.1;           
t = 0:ts:tf;        
N = length(t);      

% 2 PUNTOS DESEADOS (TRAYECTORIA) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definimos la matriz de puntos [x, y]
puntos = [-3,-2; -3,-1.5; -2,-1; -2.5,-0.5; -3,0.5; -2.5,1.5; -1.5,1.5; ...
          -0.5,1; 0,1.5; -0.5,2; 0,1.5; 0.5,2; 0,1.5; 0.5,1; 1.5,1.5; ...
          2.5,-0.5; 2,-1; 3,-1.5; 2,-1; 3,-1.5; 3,-2; 2,-2.5; 1.5,-2; ...
          0.5,-1.5; 0,-2; -0.5,-1.5; -1.5,-2; -2.5,-2.5; -3,-2];

indice_punto = 1;      % Empezamos por el primer punto
num_puntos = size(puntos, 1);
tolerancia = 0.25;     % Radio de cercanía para cambiar al siguiente punto

% 3 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1(1) = -3;  
y1(1) = -2;  
phi(1) = pi/2; 

hx(1) = x1(1);  
hy(1) = y1(1);  

% 4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    % Actualizar punto deseado actual
    hxd = puntos(indice_punto, 1);
    hyd = puntos(indice_punto, 2);

    % a) Errores de control
    hxe(k) = hxd - hx(k);
    hye(k) = hyd - hy(k);
    
    % Magnitud del error
    Error(k) = sqrt(hxe(k)^2 + hye(k)^2);

    % LÓGICA DE CAMBIO DE PUNTO:
    % Si el error es menor a la tolerancia, pasamos al siguiente punto
    if Error(k) < tolerancia && indice_punto < num_puntos
        indice_punto = indice_punto + 1;
    end

    % b) Matriz Jacobiana
    J = [cos(phi(k)) -sin(phi(k));... 
         sin(phi(k)) cos(phi(k))];

    % c) Matriz de Ganancias (ajustada para mayor suavidad)
    K = [3.0 0; 0 0.2];

    % d) Ley de Control
    he = [hxe(k); hye(k)];
    qpRef = pinv(J) * K * he;

    v(k) = qpRef(1);   
    w(k) = qpRef(2);   

    % 5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi(k+1) = phi(k) + w(k) * ts; 
    
    xp1 = v(k) * cos(phi(k));  
    yp1 = v(k) * sin(phi(k));
 
    x1(k+1) = x1(k) + xp1 * ts; 
    y1(k+1) = y1(k) + yp1 * ts; 

    hx(k+1) = x1(k+1);  
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure;  
set(scene, 'Color', 'white');
sizeScreen = get(0, 'ScreenSize');
set(scene, 'position', sizeScreen);
axis equal; grid on; hold on;
xlabel('x(m)'); ylabel('y(m)');

% Graficar todos los puntos de la trayectoria de fondo
plot3(puntos(:,1), puntos(:,2), puntos(:,1)*0, '--k', 'LineWidth', 0.5); 
scatter3(puntos(:,1), puntos(:,2), puntos(:,1)*0, 'b', 'filled');

% Inicialización de gráficos móviles
scale = 1.5; % Ajustado para que se vea mejor en la escala de tus puntos
MobileRobot_5; 
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale);
H2 = plot3(hx(1), hy(1), 0, 'r', 'lineWidth', 2);

step = 2; % Aumentar para que la animación sea más rápida
for k=1:step:N
    delete(H1);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    set(H2, 'XData', hx(1:k), 'YData', hy(1:k), 'ZData', zeros(1,k));
    drawnow; % Más eficiente que pause(ts) para animaciones largas
end