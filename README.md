# Predicción de Autonomía de Batería mediante Redes LSTM y Supervisión Dinámica en ROS 2

Sistema integral de patrullaje autónomo para un robot **TurtleBot3 Burger** en Gazebo (ROS 2 Humble) que integra planificación global mediante algoritmo **A\***, navegación reactiva con **control lógico difuso tipo Mamdani**, modelado electro-mecánico de batería **Li-ion 2S** y predicción temporal de autonomía mediante una red neuronal **LSTM** para supervisión dinámica de retorno a base.

**Autoras:** Ámbar Luisana Canto Contreras, Laura Carolina Naranjo Aviles — *Ingeniería Mecatrónica, Universidad Tecnológica Ecotec*

---

## Estructura del Repositorio

* `proyautomata_MATLAB.m` / `retorno.yaml`: Planificación global $A^*$ y tabla de distancias óptimas de retorno ($d_{\text{base}}$) desde $CP_1\text{--}CP_6$.
* `src/`: Paquetes ROS 2 (`controlador_pared.py`, `bateria_sim_node.py`, `predictor_autonomia.py`, `nodo_supervisor.py` y mundo `factory_world.world`).
* `generador_datos.py` / `verificar_dataset.py` / `dataset_misiones_bateria.csv`: Generación y validación de 200 misiones simuladas ($244\,964$ muestras a $1\text{ Hz}$).
* `entrenar_final_lstm.py` / `modelos/`: Entrenamiento y pesos de la red recurrente `modelo_lstm.h5` (ventanas de $30\text{ s} \times 5$ variables).
* `resultados/`: Métricas de evaluación (`MAE = 5.38 s`, `RMSE = 8.21 s`), gráficas de convergencia y comparativa de 10 misiones en Gazebo.

## Instalación y Ejecución en ROS 2 Humble

```bash
# Compilar workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

# Lanzar simulación completa con supervisión predictiva LSTM
ros2 launch bateria_sim simulacion_patrullaje.launch.py
