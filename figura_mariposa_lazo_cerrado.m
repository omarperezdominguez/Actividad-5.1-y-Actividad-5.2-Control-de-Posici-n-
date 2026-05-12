%Limpieza de pantalla
clear all
close all
clc

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=120;             % Tiempo de simulación en segundos (s)
ts=0.1;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras

%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1(1)=7;
y1(1)=8;
phi(1)=0;

hx(1)= x1(1);
hy(1)= y1(1);

%3 WAYPOINTS DE LA FIGURA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
waypoints = [
   7,8;
   9,8;
   9,3;
   8,2;
   7,3;
   6.5,8;
   7,3;
   5,2;
   3,1;
   2,2;
   2,3;
   4,4;
   3,5;
   2.16,6.96;
   3,9;
   5,9;
   7,8;
   8,9;
   9,10;
   8,9;
   7,9;
   9,8;
   11,9;
   13,9;
   14,7;
   13,5;
   12,4;
   14, 3;
   14,2;
   12,1;
   11,2;
   10,3;
];

%4 LAZO ABIERTO - Generar v y w desde waypoints %%%%%%%%%%%%%%%%%%
% Las velocidades se calculan UNA SOLA VEZ antes del bucle
% No hay retroalimentacion ni corrección de error

v_ref  = 1.5;
w_giro = 2.0;

v = zeros(1,N);
w = zeros(1,N);

phi_aux = phi(1);
v_total = [];
w_total = [];

for i = 1:size(waypoints,1)-1
    x_ini = waypoints(i,1);   y_ini = waypoints(i,2);
    x_fin = waypoints(i+1,1); y_fin = waypoints(i+1,2);

    dist = sqrt((x_fin-x_ini)^2 + (y_fin-y_ini)^2);
    if dist < 0.001; continue; end

    angle_seg = atan2(y_fin-y_ini, x_fin-x_ini);

    % Fase GIRO (v=0, w≠0)
    d_angle = atan2(sin(angle_seg-phi_aux), cos(angle_seg-phi_aux));
    if abs(d_angle) > 0.01
        n_turn  = max(1, round(abs(d_angle)/w_giro/ts));
        v_total = [v_total, zeros(1,n_turn)];
        w_total = [w_total, sign(d_angle)*w_giro*ones(1,n_turn)];
    end
    phi_aux = angle_seg;

    % Fase TRASLACION (v≠0, w=0)
    n_move  = max(1, round(dist/v_ref/ts));
    v_total = [v_total, v_ref*ones(1,n_move)];
    w_total = [w_total, zeros(1,n_move)];
end

% Ajustar al tamaño N del vector de tiempo
if length(v_total) >= N
    v = v_total(1:N);
    w = w_total(1:N);
else
    v(1:length(v_total)) = v_total;
    w(1:length(w_total)) = w_total;
end

%5 BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N

    % Integración orientación
    phi(k+1)=phi(k)+w(k)*ts;

    % Modelo cinemático
    xp1=v(k)*cos(phi(k));
    yp1=v(k)*sin(phi(k));

    % Integración posición (Euler)
    x1(k+1)=x1(k)+ xp1*ts;
    y1(k+1)=y1(k)+ yp1*ts;

    % Punto de control
    hx(k+1)=x1(k+1);
    hy(k+1)=y1(k+1);

end

%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%

scene=figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');

view([-0.1 35]);
axis([-2 18 -2 18 0 1]);

scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;

H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);

step=5;
for k=1:step:N
    delete(H1);
    delete(H2);
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    drawnow;    
    pause(ts);


end

%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;
set(graph,'position',sizeScreen);
subplot(211)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(212)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');