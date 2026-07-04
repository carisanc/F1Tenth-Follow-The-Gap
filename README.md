# 🏎️ F1Tenth — Follow The Gap Controller

> **Curso:** Vehículos No Tripulados  
> **Algoritmo:** Follow The Gap (FTG)  
> **Simulador:** [F1Tenth Gym ROS2](https://github.com/f1tenth/f1tenth_gym_ros)  
> **Mapa de prueba:** Budapest

---

## 📋 Tabla de contenidos

1. [Descripción del enfoque](#-descripción-del-enfoque)
2. [Estructura del código](#-estructura-del-código)
3. [Parámetros tunables](#-parámetros-tunables)
4. [Instrucciones de ejecución](#-instrucciones-de-ejecución)
5. [Sistema de vueltas y temporización](#-sistema-de-vueltas-y-temporización)
6. [Resultados](#-resultados)

---

## 🧠 Descripción del enfoque

### ¿Qué es Follow The Gap?

**Follow The Gap (FTG)** es un algoritmo de navegación reactiva que no requiere un mapa previo del entorno. El vehículo toma decisiones en tiempo real a partir de las lecturas del sensor LiDAR, buscando el espacio libre más amplio y dirigiéndose hacia él.

### Pipeline del algoritmo

```
LiDAR scan
     │
     ▼
1. Preprocesamiento
   (limpiar NaN/Inf, recortar al FOV)
     │
     ▼
2. Suavizado
   (media móvil de 5 puntos)
     │
     ▼
3. Detectar obstáculo más cercano
     │
     ▼
4. Aplicar burbuja de seguridad
   (radio adaptativo según distancia)
     │
     ▼
5. Encontrar el gap más largo
     │
     ▼
6. Calcular mejor ángulo
   (80% centro del gap + 20% punto más lejano)
     │
     ▼
7. Filtros anti-oscilación
   (Dead-zone → EMA → Rate limiter)
     │
     ▼
8. Publicar velocidad y ángulo de dirección
```

### Decisiones de diseño clave

| Componente | Decisión | Razón |
|---|---|---|
| **Selección del punto objetivo** | 80% centro del gap + 20% punto más lejano | Equilibrio entre seguridad y velocidad |
| **Burbuja de seguridad** | Radio adaptativo en función de la distancia al obstáculo | A obstáculos cercanos, burbuja más grande |
| **Filtro EMA** | α = 0.30 fijo | Elimina oscilaciones sin introducir lag excesivo |
| **Rate limiter** | 3°/scan máximo | Segunda barrera contra saltos bruscos de ángulo |
| **Dead-zone** | 2° centrales → forzado a 0 | Evita correcciones de ruido en rectas |
| **Velocidad** | Proporcional al ángulo de dirección | Más rápido en rectas, más lento en curvas |

---

## 📁 Estructura del código

```
gap_follow/
├── gap_follow/
│   └── reactive_gap_follow.py   ← Nodo principal
├── resource/
│   └── gap_follow
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

### Estructura interna del nodo

```python
class ReactiveGapFollower(Node):
    │
    ├── __init__()
    │     Inicializa publishers, subscribers,
    │     variables de filtro y contador de vueltas.
    │
    ├── odom_callback()
    │     Recibe posición del vehículo.
    │     Detecta cruce de meta y registra tiempos de vuelta.
    │
    ├── scan_callback()
    │     Pipeline principal FTG:
    │     preprocesa → burbuja → gap → ángulo → filtros
    │
    └── publish_drive()
          Calcula velocidad según ángulo y publica comando.
```

### Lógica de velocidad

```
│ Ángulo │       Velocidad        │
│ < 0.06 rad  │ 8.5 m/s  (recta)         │
│ < 0.10 rad  │ 7.2 m/s  (ligera curva)  │
│ < 0.16 rad  │ 5.6 m/s  (curva suave)   │
│ < 0.25 rad  │ 4.8 m/s  (curva media)   │
│ ≥ 0.25 rad  │ 2.2 m/s  (curva cerrada) │
```

---

## ⚙️ Parámetros tunables

Todos los parámetros están definidos como constantes al inicio del archivo para facilitar el tuning:

```python
ALPHA               = 0.30    # Filtro EMA — más alto = más reactivo, más bajo = más suave
MAX_ANGLE_DELTA_DEG = 3.0     # Rate limiter en °/scan
DEAD_ZONE_DEG       = 2.0     # Dead-zone central en grados
CAR_HALF_WIDTH      = 0.35    # Ancho del vehículo en metros (para la burbuja)
FOV_DEG             = 100.0   # Campo de visión del LiDAR en grados
MAX_STEERING        = 0.42    # Ángulo máximo de dirección en radianes
```

### Guía de tuning rápido

| Síntoma | Ajuste |
|---|---|
| Oscila en rectas | Bajar `ALPHA` o `MAX_ANGLE_DELTA_DEG` |
| Reacciona tarde en curvas | Subir `ALPHA` o `MAX_ANGLE_DELTA_DEG` |
| Roza paredes laterales | Subir `CAR_HALF_WIDTH` |
| Se detiene antes de curvas | Revisar umbrales de velocidad |

---

## 🚀 Instrucciones de ejecución

### Requisitos previos

- Ubuntu 22.04
- ROS2 Humble
- F1Tenth Gym ROS2

### 1. Instalar dependencias del simulador

```bash
# Instalar ackermann_msgs
sudo apt install ros-humble-ackermann-msgs

# Clonar e instalar f110_gym
cd ~/F1Tenth-Repository
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym
pip install -e .

# Fix de compatibilidad numba/coverage si es necesario
pip install coverage==6.5.0
```

### 2. Crear y configurar el paquete

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python gap_follow \
    --dependencies rclpy sensor_msgs nav_msgs ackermann_msgs
```

Copiar el nodo:
```bash
cp reactive_gap_follow.py ~/ros2_ws/src/gap_follow/gap_follow/
```

Editar `setup.py` y agregar el entry point:
```python
entry_points={
    'console_scripts': [
        'reactive_gap_follower = gap_follow.reactive_gap_follow:main',
    ],
},
```

### 3. Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select gap_follow
source install/setup.bash
```

Agregar al `.bashrc` para no repetirlo:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Ejecutar

**Terminal 1 — Simulador:**
```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 — Controlador:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run gap_follow reactive_gap_follower
```

### Verificar que los topics están activos

```bash
ros2 topic list
# Deberías ver:
# /scan
# /drive
# /ego_racecar/odom
```

---

## 🏁 Sistema de vueltas y temporización

El nodo incluye un contador automático de vueltas basado en odometría. Al iniciar, guarda la posición de salida y detecta cada cruce de meta.

### Salida en consola

```
[INFO] Punto de salida: x=12.34  y=-5.67
[INFO] Contador de vuelta armado.
[INFO] Vuelta 1/10 | Tiempo vuelta: 28.43 s | Mejor vuelta: 28.43 s | Tiempo total: 28.43 s
[INFO] Vuelta 2/10 | Tiempo vuelta: 27.91 s | Mejor vuelta: 27.91 s | Tiempo total: 56.34 s
...
[INFO] ¡10 vueltas completadas! Mejor vuelta: 27.12 s | Tiempo total: 284.50 s
```

El vehículo se detiene automáticamente al completar las 10 vueltas.

### Parámetros del contador

```python
MAX_LAPS       = 10     # Número de vueltas
REARM_DISTANCE = 8.0    # Distancia mínima antes de rearmar el contador (metros)
FINISH_RADIUS  = 1.5    # Radio de la zona de meta (metros)
```

---

## 📊 Resultados

### Mapa: Budapest

<img width="517" height="263" alt="Screenshot from 2026-07-04 15-01-54" src="https://github.com/user-attachments/assets/5f808c18-c439-4603-ae46-4f96ecc0a624" />


---

## 🗺️ Mapa utilizado

<img width="1160" height="632" alt="Screenshot from 2026-07-04 13-14-45" src="https://github.com/user-attachments/assets/27eb7932-a6a6-4b80-9114-5b914002f026" />


El controlador fue desarrollado y probado en el mapa **Budapest** del simulador F1Tenth, un circuito con curvas variadas que exige un buen balance entre velocidad y seguridad.

---

*Proyecto desarrollado para el curso de Vehículos No Tripulados.*
