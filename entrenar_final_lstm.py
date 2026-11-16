#!/usr/bin/env python3
import os
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Permite guardar gráficas sin problemas en VM
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Dropout, Input
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping

# Fijar semillas para reproducibilidad
np.random.seed(42)
tf.random.set_seed(42)

# 1. Localizar automáticamente el dataset de 200 misiones
base_dir = os.path.expanduser('~/ros2_ws')
csv_files = glob.glob(os.path.join(base_dir, '**', '*.csv'), recursive=True)
if not csv_files:
    raise FileNotFoundError("No se encontró ningún archivo .csv en ~/ros2_ws. Verifica la ruta.")

# Seleccionar el CSV más pesado (el de las 200 misiones)
dataset_path = max(csv_files, key=os.path.getsize)
print(f"[INFO] Cargando dataset desde: {dataset_path}")
df = pd.read_csv(dataset_path)
print(f"[INFO] Columnas detectadas: {list(df.columns)}")

# 2. Identificar columnas automáticamente según las convenciones del proyecto
col_mision = [c for c in df.columns if 'mision' in c.lower() or 'id' in c.lower()][0]
col_target = 'tiempo_restante_s'

def buscar_col(palabras):
    for c in df.columns:
        if any(p in c.lower() for p in palabras) and c not in [col_mision, col_target]:
            return c
    raise ValueError(f"No se encontró columna para {palabras}")

cols_features = [
    buscar_col(['volt']),
    buscar_col(['corr']),
    buscar_col(['vel']),
    buscar_col(['carg']),
    buscar_col(['soc'])
]
print(f"[INFO] Misión: '{col_mision}' | Features (5): {cols_features} | Objetivo: '{col_target}'")

# 3. División 70 / 15 / 15 por misiones completas (sin mezclar ventanas entre conjuntos)
misiones_unicas = df[col_mision].unique()
np.random.shuffle(misiones_unicas)

n_total = len(misiones_unicas)
n_train = int(0.70 * n_total)
n_val = int(0.15 * n_total)

misiones_train = misiones_unicas[:n_train]
misiones_val = misiones_unicas[n_train:n_train + n_val]
misiones_test = misiones_unicas[n_train + n_val:]

print(f"[INFO] División de misiones -> Train: {len(misiones_train)} | Val: {len(misiones_val)} | Test: {len(misiones_test)}")

df_train = df[df[col_mision].isin(misiones_train)].copy()
df_val = df[df[col_mision].isin(misiones_val)].copy()
df_test = df[df[col_mision].isin(misiones_test)].copy()

# 4. Normalización (estadísticas calculadas SOLO con el conjunto de entrenamiento)
mean_train = df_train[cols_features].mean().values.astype(np.float32)
std_train = df_train[cols_features].std().values.astype(np.float32)
std_train = np.where(std_train == 0, 1.0, std_train)

# Guardar parámetros de normalización para el nodo ROS 2 predictor_autonomia
modelos_dir = os.path.join(base_dir, 'modelos')
resultados_dir = os.path.join(base_dir, 'resultados')
os.makedirs(modelos_dir, exist_ok=True)
os.makedirs(resultados_dir, exist_ok=True)

np.savez(
    os.path.join(modelos_dir, 'scaler_lstm.npz'),
    mean=mean_train,
    std=std_train,
    features=np.array(cols_features)
)

# 5. Construcción de ventanas deslizantes de 30 pasos x 5 variables
PASOS = 30

def construir_ventanas(dataframe):
    X_seq, X_mlp, y_out, soc_out, duracion_out, mision_ids = [], [], [], [], [], []
    for mid, grupo in dataframe.groupby(col_mision, sort=False):
        feats_raw = grupo[cols_features].values.astype(np.float32)
        feats_norm = (feats_raw - mean_train) / std_train
        targets = grupo[col_target].values.astype(np.float32)
        soc_vals = grupo[cols_features[4]].values.astype(np.float32)
        duracion_total = float(targets[0]) if len(targets) > 0 and targets[0] > 0 else float(len(targets))

        if len(feats_norm) < PASOS:
            continue

        for i in range(PASOS - 1, len(feats_norm)):
            X_seq.append(feats_norm[i - PASOS + 1 : i + 1])  # (30, 5)
            X_mlp.append(feats_norm[i])                      # (5,) instante actual
            y_out.append(targets[i])
            soc_out.append(soc_vals[i])
            duracion_out.append(duracion_total)
            mision_ids.append(mid)

    return (
        np.array(X_seq, dtype=np.float32),
        np.array(X_mlp, dtype=np.float32),
        np.array(y_out, dtype=np.float32),
        np.array(soc_out, dtype=np.float32),
        np.array(duracion_out, dtype=np.float32),
        np.array(mision_ids)
    )

X_train_seq, X_train_mlp, y_train, _, _, _ = construir_ventanas(df_train)
X_val_seq, X_val_mlp, y_val, _, _, _ = construir_ventanas(df_val)
X_test_seq, X_test_mlp, y_test, soc_test, dur_test, mid_test = construir_ventanas(df_test)

print(f"[INFO] Ventanas generadas -> Train: {X_train_seq.shape} | Val: {X_val_seq.shape} | Test: {X_test_seq.shape}")

# 6. Definición de Arquitectura Oficial LSTM (Fase 1)
# Entrada (30x5) -> LSTM(64) -> Dropout(0.2) -> Dense(32, ReLU) -> Dense(1, lineal)
modelo_lstm = Sequential([
    Input(shape=(PASOS, len(cols_features))),
    LSTM(64),
    Dropout(0.2),
    Dense(32, activation='relu'),
    Dense(1, activation='linear')
], name='LSTM_Autonomia')

modelo_lstm.compile(
    optimizer=Adam(learning_rate=1e-3),
    loss='mse',
    metrics=['mae']
)
modelo_lstm.summary()

# Modelo de referencia MLP (solo instante actual: 5 variables)
modelo_mlp = Sequential([
    Input(shape=(len(cols_features),)),
    Dense(64, activation='relu'),
    Dropout(0.2),
    Dense(32, activation='relu'),
    Dense(1, activation='linear')
], name='MLP_Referencia')

modelo_mlp.compile(
    optimizer=Adam(learning_rate=1e-3),
    loss='mse',
    metrics=['mae']
)

# 7. Entrenamiento con Early Stopping (Fase 2: 20-40 épocas, lote 64)
early_stop_lstm = EarlyStopping(monitor='val_loss', patience=6, restore_best_weights=True)
early_stop_mlp = EarlyStopping(monitor='val_loss', patience=6, restore_best_weights=True)

print("\n[ENTRENANDO] Red LSTM principal...")
hist_lstm = modelo_lstm.fit(
    X_train_seq, y_train,
    validation_data=(X_val_seq, y_val),
    epochs=40,
    batch_size=64,
    callbacks=[early_stop_lstm],
    verbose=1
)

print("\n[ENTRENANDO] Red MLP de referencia...")
hist_mlp = modelo_mlp.fit(
    X_train_mlp, y_train,
    validation_data=(X_val_mlp, y_val),
    epochs=40,
    batch_size=64,
    callbacks=[early_stop_mlp],
    verbose=1
)

# Guardar modelo LSTM entrenado
ruta_modelo = os.path.join(modelos_dir, 'modelo_lstm.h5')
modelo_lstm.save(ruta_modelo)
print(f"\n[OK] Modelo guardado en: {ruta_modelo}")

# 8. Evaluación y Métricas Detalladas en Conjunto de Prueba (Fase 3)
pred_lstm = modelo_lstm.predict(X_test_seq, verbose=0).flatten()
pred_mlp = modelo_mlp.predict(X_test_mlp, verbose=0).flatten()

def calcular_metricas(y_real, y_est, duraciones):
    mae_s = float(np.mean(np.abs(y_real - y_est)))
    rmse_s = float(np.sqrt(np.mean((y_real - y_est) ** 2)))
    mae_pct = float(np.mean((np.abs(y_real - y_est) / duraciones) * 100.0))
    rmse_pct = float(np.sqrt(np.mean(((y_real - y_est) / duraciones) ** 2)) * 100.0)
    return mae_s, rmse_s, mae_pct, rmse_pct

mae_l, rmse_l, maep_l, rmsep_l = calcular_metricas(y_test, pred_lstm, dur_test)
mae_m, rmse_m, maep_m, rmsep_m = calcular_metricas(y_test, pred_mlp, dur_test)

print("\n" + "="*72)
print("RESULTADOS DE EVALUACIÓN EN CONJUNTO DE PRUEBA (30 MISIONES)")
print("="*72)
print(f"{'Modelo':<18} | {'MAE (s)':<10} | {'RMSE (s)':<10} | {'MAE (% dur)':<12} | {'RMSE (% dur)':<12}")
print("-"*72)
print(f"{'LSTM (30 pasos)':<18} | {mae_l:<10.2f} | {rmse_l:<10.2f} | {maep_l:<12.2f} | {rmsep_l:<12.2f}")
print(f"{'MLP (1 instante)':<18} | {mae_m:<10.2f} | {rmse_m:<10.2f} | {maep_m:<12.2f} | {rmsep_m:<12.2f}")
print("="*72)

# Error por tramo de descarga (Inicio: SoC > 70%, Mitad: 40% < SoC <= 70%, Final: SoC <= 40%)
print("\nDESGLOSE DE ERROR DE LA LSTM POR TRAMO DE DESCARGA:")
print("-"*65)
tramos = [
    ("Inicio (SoC > 70%)", soc_test > 70.0),
    ("Mitad (40% < SoC <= 70%)", (soc_test > 40.0) & (soc_test <= 70.0)),
    ("Final (SoC <= 40%)", soc_test <= 40.0)
]
for nombre_tramo, mask in tramos:
    if np.any(mask):
        t_mae = np.mean(np.abs(y_test[mask] - pred_lstm[mask]))
        t_rmse = np.sqrt(np.mean((y_test[mask] - pred_lstm[mask]) ** 2))
        print(f"{nombre_tramo:<28} | MAE: {t_mae:6.2f} s | RMSE: {t_rmse:6.2f} s")
print("-"*65)

# 9. Generación de Gráficas de Fase 2
# Gráfica 1: Curvas de pérdida (Loss MSE)
plt.figure(figsize=(9, 5))
plt.plot(hist_lstm.history['loss'], label='Entrenamiento (Train Loss)', linewidth=2)
plt.plot(hist_lstm.history['val_loss'], label='Validación (Val Loss)', linewidth=2, linestyle='--')
plt.title('Curvas de Pérdida (MSE) durante el Entrenamiento de la LSTM')
plt.xlabel('Época')
plt.ylabel('Error Cuadrático Medio (MSE)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(resultados_dir, 'curvas_perdida_lstm.png'), dpi=150)
plt.close()

# Gráfica 2: Predicción vs Valor Real en 3 misiones de prueba
misiones_plot = np.unique(mid_test)[:3]
fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
for idx, m_id in enumerate(misiones_plot):
    m_mask = (mid_test == m_id)
    t_eje = np.arange(np.sum(m_mask))
    axes[idx].plot(t_eje, y_test[m_mask], label='Real', color='black', linewidth=2)
    axes[idx].plot(t_eje, pred_lstm[m_mask], label='LSTM', color='#1f77b4', linestyle='--', linewidth=2)
    axes[idx].plot(t_eje, pred_mlp[m_mask], label='MLP', color='#ff7f0e', linestyle=':', linewidth=1.5)
    axes[idx].set_title(f'Misión de Prueba #{m_id}')
    axes[idx].set_xlabel('Tiempo de ventana (s)')
    if idx == 0:
        axes[idx].set_ylabel('Autonomía restante (s)')
    axes[idx].legend()
    axes[idx].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(resultados_dir, 'prediccion_3_misiones.png'), dpi=150)
plt.close()
print(f"[OK] Gráficas guardadas en: {resultados_dir}/")
