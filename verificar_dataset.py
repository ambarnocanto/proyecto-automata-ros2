import csv

archivo = 'dataset_misiones_bateria.csv'

total_filas = 0
misiones = set()

with open(archivo, mode='r') as f:
    lector = csv.DictReader(f)
    cabeceras = lector.fieldnames
    for fila in lector:
        total_filas += 1
        misiones.add(int(fila['mision_id']))

print("--- REPORTE DE VERIFICACIÓN DEL PUNTO 1.6 ---")
print(f"Total de registros (filas): {total_filas}")
print(f"Misiones únicas encontradas: {len(misiones)} (debe ser 200)")
print(f"\nColumnas presentes en el archivo:")
for col in cabeceras:
    print(f" - {col}")
