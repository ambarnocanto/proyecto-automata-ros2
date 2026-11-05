import random
import csv
import time

def simular_misiones():
    nombre_archivo = "dataset_misiones_bateria.csv"
    
    # Definimos las cabeceras exigidas por la rúbrica
    headers = [
        "mision_id", "tiempo_s", "voltaje_v", "corriente_a", 
        "velocidad_ms", "carga_kg", "soc_porcentaje", "tiempo_restante_s"
    ]
    
    print(f"Iniciando la generación de 200 misiones de descarga...")
    
    with open(nombre_archivo, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        for mision in range(1, 201):
            # Parámetros aleatorios para variar cada misión
            velocidad = round(random.uniform(0.05, 0.25), 2)  # m/s
            carga = round(random.uniform(0.0, 5.0), 1)         # kg
            
            # Estado inicial de la batería
            soc = 100.0  # %
            voltaje = 8.4  # Voltios iniciales aproximados (batería Li-ion 2S)
            
            # Estimación de duración de la misión en función de la velocidad y carga
            duracion_total_estimada = int(1200 / (velocidad * 0.5 + carga * 0.1 + 0.5))
            tiempo_transcurrido = 0
            
            while soc > 10.0:
                # Cálculo de corriente basada en el esfuerzo (velocidad y carga)
                corriente = round(0.2 + (velocidad * 1.5) + (carga * 0.3) + random.uniform(-0.02, 0.02), 3)
                
                # Tasa de descarga proporcional a la corriente
                tasa_descarga = (corriente / 2.0) * 0.15
                soc = max(10.0, soc - tasa_descarga)
                
                # Descenso simulado del voltaje en función del SoC
                voltaje = round(6.8 + (soc / 100.0) * 1.6, 3)
                
                # Tiempo restante estimado hasta llegar al 10%
                tiempo_restante = max(0, duracion_total_estimada - tiempo_transcurrido)
                
                # Escribir la fila de datos en el CSV
                writer.writerow([
                    mision, tiempo_transcurrido, voltaje, corriente, 
                    velocidad, carga, round(soc, 2), tiempo_restante
                ])
                
                tiempo_transcurrido += 1
                
        print(f"¡Proceso finalizado con éxito! Archivo generado: {nombre_archivo}")

if __name__ == "__main__":
    simular_misiones()
