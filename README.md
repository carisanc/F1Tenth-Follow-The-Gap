# 🏎️ F1Tenth — Follow The Gap con Obstáculos

> **Curso:** Vehículos No Tripulados  
> **Algoritmo:** Follow The Gap (FTG)  
> **Simulador:** [F1Tenth Gym ROS2](https://github.com/f1tenth/f1tenth_gym_ros)  
> **Mapa de prueba:** Budapest (con obstáculos estáticos y dinámico)

---

## 📋 Tabla de contenidos

1. [Descripción general del reto](#-descripción-general-del-reto)
2. [Parte 1 — Obstáculos estáticos](#-parte-1--obstáculos-estáticos)
3. [Parte 2 — Obstáculo dinámico](#-parte-2--obstáculo-dinámico)
4. [Modificaciones al controlador principal](#-modificaciones-al-controlador-principal)
5. [Nodo del carro oponente](#-nodo-del-carro-oponente)
6. [Instrucciones de ejecución](#-instrucciones-de-ejecución)
7. [Resultados](#-resultados)

---

## 🧠 Descripción general del reto

Esta segunda parte del proyecto añade dos capas de dificultad sobre el circuito de Budapest:

- **5 obstáculos estáticos** pintados directamente en el mapa del simulador con GIMP
- **1 carro oponente** (obstáculo dinámico) que circula el mismo circuito a velocidad reducida

El objetivo en ambos casos es completar **10 vueltas consecutivas sin colisión**.

---

## 🗺️ Parte 1 — Obstáculos estáticos

### Cómo se agregaron al mapa

Los obstáculos estáticos se añadieron editando la imagen del mapa directamente con **GIMP** (GNU Image Manipulation Program):

1. Abrir el archivo del mapa:
```
~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Budapest_map.png
```

2. En GIMP, usar la herramienta **Lápiz** o **Pincel** con color **negro puro (#000000)** para pintar bloques rectangulares sobre la pista en las zonas deseadas. El simulador interpreta los píxeles negros como paredes infranqueables.

3. Guardar el archivo en formato .png.

```

> **Importante:** después de editar el mapa hay que recompilar y relanzar el simulador para que los cambios surtan efecto.

```bash
cd ~/F1Tenth-Repository
colcon build
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Adaptaciones del controlador para obstáculos estáticos

Para que el carro principal no chocara con los conos, se ajustaron tres parámetros respecto a la versión original:

| Parámetro | Valor original | Valor con obstáculos | Efecto |
|---|---|---|---|
| `radio_burbuja` | 20 índices | 52 índices | Burbuja más grande tapa mejor los obstáculos puntuales |
| `rango_max` | `scan.range_max` | 6.8 m | El carro "ve" solo hasta 6.8 m, reaccionando antes a obstáculos cercanos |
| `vel_curva` | 3.0 m/s | 2.5 m/s | Menor velocidad mínima en curvas para tener más margen de reacción |

---

## 🤖 Parte 2 — Obstáculo dinámico

### Configuración del segundo carro en `sim.yaml`

Para activar el segundo carro hay que editar el archivo de configuración del simulador:

```
~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

Cambiar el parámetro `num_agent` de 1 a 2 y definir la pose inicial del oponente:

```yaml
# opponent parameters
num_agent: 2

# ego starting pose on map
sx: 0.0
sy: 0.0
stheta: 0.0

# opp starting pose on map
sx1: 36.35
sy1: 66.50
stheta1: 3.14    
```

Los campos `sx1`, `sy1` ubican al oponente en un punto del circuito suficientemente alejado del carro principal para que no arranquen juntos en la misma curva.

### Objetivo del carro oponente

El segundo carro (`opponent_gap_follow.py`) tiene dos propósitos:

1. **Ser un obstáculo dinámico realista** — circula el circuito completo con el mismo algoritmo FTG pero a velocidad reducida (~40% de la velocidad del carro principal)
2. **Completar sus propias 10 vueltas** — también tiene contador de vueltas y se detiene al terminar

### Topics del oponente

El simulador con `num_agent: 2` publica topics separados para cada carro:

| Topic | Carro principal | Oponente |
|---|---|---|
| LiDAR | `/scan` | `/opp_scan` |
| Comandos | `/drive` | `/opp_drive` |
| Odometría | `/ego_racecar/odom` | `/opp_racecar/odom` |

---

## 🔧 Modificaciones al controlador principal

Respecto a la Parte 1, el controlador principal (`reactive_gap_follow.py`) incorpora:

### Parámetros clave

```python
self.vel_recta = 7.5      # m/s en rectas
self.vel_curva = 2.5      # m/s en curvas cerradas
self.rango_max = 6.8      # recorte del LiDAR a 6.8 m
self.radio_burbuja = 52   # burbuja grande para tapar al oponente + rendija lateral
self.umbral_gap = 1.7     # m — umbral mínimo para considerar un gap como libre
self.alpha_suavizado = 0.50  # peso del ángulo nuevo en el filtro EMA
```

### Velocidad dinámica continua

En vez de escalones fijos de velocidad, se usa una interpolación lineal continua basada en el ángulo de giro:

```python
factor_giro = abs(steering_angle) / 0.41
speed = vel_recta - (vel_recta - vel_curva) * factor_giro
```

Esto evita cambios bruscos de velocidad al salir de curvas, que causaban pérdida de control.

---

## 📄 Nodo del carro oponente

El archivo `opponent_gap_follow.py` es un FTG independiente que:

- Se suscribe a `/opp_scan` y `/opp_racecar/odom`
- Publica en `/opp_drive`
- Usa el mismo algoritmo que el carro principal pero con velocidades reducidas:

```python
SPEED_STRAIGHT = 5.0   # m/s (vs 7.5 del principal)
SPEED_MEDIUM   = 2.0   # m/s
SPEED_CURVE    = 1.8   # m/s
SPEED_DANGER   = 0.8   # m/s
```

---

## 🚀 Instrucciones de ejecución

### Requisitos

- Ubuntu 22.04
- ROS2 Humble
- F1Tenth Gym ROS2 instalado y compilado
- Paquete `gap_follow` compilado con ambos nodos

### Agregar el nodo oponente al paquete

Copiar el archivo:
```bash
cp opponent_gap_follow.py ~/ros2_ws/src/gap_follow/gap_follow/
```

Editar `setup.py` para incluir ambos entry points:
```python
entry_points={
    'console_scripts': [
        'reactive_gap_follower = gap_follow.reactive_gap_follow:main',
        'opponent_gap_follower = gap_follow.opponent_gap_follow:main',
    ],
},
```

Recompilar:
```bash
cd ~/ros2_ws
colcon build --packages-select gap_follow
source install/setup.bash
```

### Lanzar los 3 nodos

Abrir **3 terminales separadas**:

**Terminal 1 — Simulador:**
```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 — Carro principal:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run gap_follow reactive_gap_follower
```

**Terminal 3 — Carro oponente:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run gap_follow opponent_gap_follower
```

### Verificar que los topics están activos

```bash
ros2 topic list
# Deberías ver:
# /scan
# /drive
# /ego_racecar/odom
# /opp_scan
# /opp_drive
# /opp_racecar/odom
```

### Salida esperada en consola

**Carro principal:**
```
[INFO] Punto de salida: x=0.00  y=0.00 | ¡Cronómetro iniciado!
[INFO] Contador de vuelta armado.
[INFO] Vuelta 1/10 | Tiempo vuelta: 32.14 s | Mejor vuelta: 32.14 s | Tiempo total: 32.14 s
...
[INFO] ¡10 vueltas completadas! Mejor tiempo: 30.87 s | Tiempo total de carrera: 318.42 s
```

**Carro oponente:**
```
[OPP] Punto de salida: x=36.35  y=66.50
[OPP] Vuelta 1/10 | Tiempo vuelta: 58.23 s | ...
```

---

## 📊 Resultados

***Mapa editado***

<img width="1160" height="632" alt="Screenshot from 2026-07-04 13-14-45" src="https://github.com/user-attachments/assets/24f893d9-6c78-4add-b007-df992c6e1a79" />

***Resultados finales del carro principal***

<img width="517" height="263" alt="Screenshot from 2026-07-04 15-01-54" src="https://github.com/user-attachments/assets/b4f172a9-e9fa-49be-8756-4f741fb7b1bc" />

***Video de funcionamiento***

- Link: https://youtu.be/eK8mGlzf1Xg?si=rpoa6shmL2zfKJg_

---

*Proyecto desarrollado para el curso de Vehículos No Tripulados.*
