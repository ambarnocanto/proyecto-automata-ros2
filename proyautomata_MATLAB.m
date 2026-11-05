% Definir resolución (ej. 10 celdas por metro -> 50x50 celdas para 5x5 m)
mapResolution = 10; 
mapWidth = 5.0; 
mapHeight = 5.0;

% Crear una cuadrícula de ocupación vacía (todo libre)
gridMatrix = zeros(mapWidth * mapResolution, mapHeight * mapResolution);

% Rellenar paredes exteriores y bloque central (ejemplos de índices)
% Bloque central en el centro (ej. de 1.0 a 4.0 m en x y y)
gridMatrix(10:40, 10:40) = 1; 

% Crear el objeto de mapa de ocupación de MATLAB
map = binaryOccupancyMap(gridMatrix, mapResolution);
figure;
show(map);
title('Mapa de Ocupación del Circuito de Pasillos');

% Coordenadas [x, y] en metros
base_pos = [1.0, 0.4]; 
puntos_control = [
    1.0, 0.8;  % CP 1 (Índice 2)
    4.0, 0.8;  % CP 2 (Índice 3)
    4.5, 2.5;  % CP 3 (Índice 4)
    4.0, 4.2;  % CP 4 (Índice 5)
    2.5, 4.5;  % CP 5 (Índice 6)
    1.0, 4.2   % CP 6 (Índice 7)
    ];

% Unir todos los puntos (Base + Puntos de Control) en una sola matriz
todos_puntos = [base_pos; puntos_control];
num_puntos = size(todos_puntos, 1);

% Crear el planificador A* basado en el mapa de ocupación
p_astar = plannerAStarGrid(map);

% Inicializar matriz de distancias
matriz_distancias = zeros(num_puntos, num_puntos);
for i = 1:num_puntos
    for j = 1:num_puntos
        if i ~= j
            start_coord = world2grid(map, todos_puntos(i, :));
            goal_coord  = world2grid(map, todos_puntos(j, :));
            % Calcular ruta usando A*
            path = plan(p_astar, start_coord, goal_coord);
            % Calcular la distancia total de la ruta en metros (convertida de celdas)
            if ~isempty(path)
                matriz_distancias(i, j) = sum(vecnorm(diff(path, [], 1), 2, 2)) / mapResolution;
            else
                matriz_distancias(i, j) = Inf; % Si no hay ruta
            end
        end
    end
end

% --- DEFINICIÓN DEL ORDEN DEL LAZO ---
% Recorrido secuencial natural de los puntos de control (índices 2 al 7)
orden_lazo = [2, 3, 4, 5, 6, 7]; 

% --- CÁLCULO DEL ORDEN POR VECINO MÁS CERCANO (Nearest Neighbor) ---
no_visitados = 2:num_puntos;
orden_nn = 1; % Inicia en la base (índice 1)
actual = 1;

while ~isempty(no_visitados)
    [~, idx_min] = min(matriz_distancias(actual, no_visitados));
    siguiente = no_visitados(idx_min);
    orden_nn = [orden_nn, siguiente];
    no_visitados(idx_min) = [];
    actual = siguiente;
end

% Calcular la distancia total del orden del lazo usando la matriz de distancias A*
distancia_lazo = 0;
orden_lazo_completo = [orden_lazo, orden_lazo(1)];
for k = 1:(length(orden_lazo_completo)-1)
    distancia_lazo = distancia_lazo + matriz_distancias(orden_lazo_completo(k), orden_lazo_completo(k+1));
end

% Calcular la distancia total del orden del vecino más cercano
distancia_nn = 0;
orden_nn_completo = [orden_nn, orden_nn(1)];
for k = 1:(length(orden_nn_completo)-1)
    distancia_nn = distancia_nn + matriz_distancias(orden_nn_completo(k), orden_nn_completo(k+1));
end

% Graficar y comparar las rutas
figure;
show(map);
hold on;
% 1. Trazar ruta del orden del lazo (Rojo)
lazo_coords = todos_puntos([orden_lazo, orden_lazo(1)], :);
plot(lazo_coords(:,1), lazo_coords(:,2), 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r', 'DisplayName', sprintf('Orden del Lazo (%.2f m)', distancia_lazo));

% 2. Trazar ruta del vecino más cercano (Azul discontinuo)
nn_coords = todos_puntos([orden_nn, orden_nn(1)], :);
plot(nn_coords(:,1), nn_coords(:,2), 'b--s', 'LineWidth', 1.5, 'MarkerFaceColor', 'b', 'DisplayName', sprintf('Vecino Más Cercano (%.2f m)', distancia_nn));

legend('Location', 'best');
title('Comparación de Órdenes de Patrullaje - Tarea 1.4');
xlabel('X [metros]');
ylabel('Y [metros]');
hold off;

% Abrir archivo para escribir la tabla de retorno
fid = fopen('retorno.yaml', 'w');
fprintf(fid, 'retorno_tabla:\n');
for i = 2:num_puntos % Desde los puntos de control (excluyendo la base en índice 1)
    d_base = matriz_distancias(i, 1); % Distancia al punto 1 (Base)
    sentido = 'continuar'; 
    fprintf(fid, '  - punto_id: %d\n', i-1);
    fprintf(fid, '    coordenadas: [%.2f, %.2f]\n', todos_puntos(i,1), todos_puntos(i,2));
    fprintf(fid, '    d_base: %.2f\n', d_base);
    fprintf(fid, '    sentido_retorno: "%s"\n', sentido);
end
fclose(fid);
disp('Archivo retorno.yaml generado con éxito.');

% Mostrar explícitamente los cálculos en la Command Window
disp('--- MATRIZ DE DISTANCIAS (A*) ---');
disp(matriz_distancias);
disp('--- ORDEN DE PATRULLAJE ---');
fprintf('Orden del Lazo: %s\n', num2str(orden_lazo));
fprintf('Orden Vecino Más Cercano: %s\n', num2str(orden_nn));
disp('--- TABLA DE RETORNO (para retorno.yaml) ---');
for i = 2:num_puntos
    fprintf('Punto Control CP%d -> Distancia a Base: %.2f m\n', i-1, matriz_distancias(i, 1));
end