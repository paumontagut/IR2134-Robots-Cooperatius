# Large Project — ESTCE Open-RMF

Simulacion del edificio ESTCE (Escola Superior de Tecnologia i Ciencies Experimentals)
con Open-RMF (ROS 2 Jazzy + Gazebo Harmonic). Tres flotas de robots: patrulla, limpieza y entrega.

---

## Estado actual del proyecto

### Completado
- [x] Mapa del edificio ESTCE con 2 plantas (L1, L2) en el Traffic Editor
- [x] 3 flotas configuradas: patrol (4 robots), cleaner (2 robots), delivery (2 robots)
- [x] Waypoints con propiedades correctas (chargers, spawn, pickup/dropoff, cleaning zones)
- [x] Nav graphs corregidos y regenerados (graphs 0, 1, 3)
- [x] World SDF regenerado con ~1580 modelos de Fuel
- [x] Swap de 16 GB configurado para evitar OOM
- [x] Fleet configs y fleet adapters funcionando
- [x] Dispenser (coke_dispenser) e ingestor (coke_ingestor) configurados
- [x] Simulacion arranca correctamente (Gazebo + RViz + fleet adapters)
- [x] Dashboard web funciona y muestra el mapa con los robots
- [x] Problema de DDS entre contenedores resuelto (CycloneDDS + LOCALHOST)
- [x] Los 8 robots se spawnean en sus chargers correctamente
- [x] Compilacion exitosa dentro del contenedor

### Pendiente
- [ ] Anadir mas robots a las flotas (mas patrolBots, cleanerBots o deliveryBots)
- [ ] Grabar video de la simulacion funcionando para la entrega
- [ ] Probar todas las tareas (patrol, clean, delivery) y grabarlas

---

## Requisitos previos

### Imagenes Docker

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest
docker pull ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
docker pull ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

### Swap de 16 GB (necesario para los ~1580 modelos)

```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Rocker (para lanzar contenedores con GPU + display)

```bash
pip install rocker
```

### Descargar modelos de Fuel y crear symlinks

> Solo la primera vez o si se borra el cache `~/.gz/fuel/`.
> Se ejecuta **dentro del contenedor**.

```bash
# 1. Descargar modelos (~40 modelos de OpenRobotics)
cd ~/rmf-ws
bash download_fuel_models.sh

# 2. Crear symlinks CamelCase -> cache Fuel
bash setup_fuel_symlinks.sh
```

### Compilar el workspace (primera vez)

> Se ejecuta **dentro del contenedor**.
> IMPORTANTE: usar `source ~/rmf_ws/install/setup.bash` (no `/opt/ros/jazzy/setup.bash`
> directamente) para que `install/setup.bash` se genere con la cadena completa:
> Jazzy → rmf_demos_ws → rmf_ws → rmf-ws.

```bash
source ~/rmf_ws/install/setup.bash
cd ~/rmf-ws
colcon build --symlink-install \
  --packages-select project_assets project_config project_fleet_adapter \
  project_maps project_simulation \
  --paths src/project/*
```

---

## Guia rapida de comandos

> Referencia rapida de que ejecutar, donde y en que orden.

### Lanzar la simulacion completa (3 terminales)

**IMPORTANTE:** Respetar el orden. API server primero, simulacion segundo, dashboard tercero.

**IMPORTANTE:** La simulacion y el API server deben usar el mismo middleware DDS.
Ambos deben usar `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Si no coinciden, el dashboard no mostrara
el mapa ni los robots.

| Terminal | Que lanza | Donde se ejecuta | Comando |
|----------|-----------|-------------------|---------|
| 1 | API Server | Host (Docker) | Ver seccion 4.1 |
| 2 | Simulacion (Gazebo + RMF) | Dentro del contenedor rocker | Ver seccion 4.2 |
| 3 | Dashboard web | Host (Docker) | Ver seccion 4.3 |
| 4 (opcional) | Despachar tareas | Dentro del contenedor rocker | Ver seccion 5 |

---

## 1. Editar el mapa en el Traffic Editor

> Solo cuando se quiera modificar el edificio, waypoints, lanes o nav graphs.
> Se ejecuta en el **host** (abre una ventana GUI).

```bash
rocker --nvidia=gpus --x11 --name traffic-editor --user --home \
  -- ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest \
  traffic-editor
```

Abrir el fichero:
```
~/rmf-ws/src/project/project_maps/maps/estce/estce.building.yaml
```

### Propiedades de waypoints por flota

**Fleet Patrol — Graph 0 (TinyRobot, 4 robots)**

| Waypoint | Propiedades |
|----------|-------------|
| patrol_charger_1 | `is_charger: true` `spawn_robot_name: patrolBot1` `spawn_robot_type: TinyRobot` |
| patrol_charger_2 | `is_charger: true` `spawn_robot_name: patrolBot2` `spawn_robot_type: TinyRobot` |
| patrol_charger_3 | `is_charger: true` `spawn_robot_name: patrolBot3` `spawn_robot_type: TinyRobot` |
| patrol_charger_4 | `is_charger: true` `spawn_robot_name: patrolBot4` `spawn_robot_type: TinyRobot` |
| Puntos de patrulla | `name: <nombre_zona>` (ej: `hall_entrada`, `sala_profesores`) |

**Fleet Cleaner — Graph 3 (CleanerBotA, 2 robots)**

| Waypoint | Propiedades |
|----------|-------------|
| cleaner_charger_1 | `is_charger: true` `spawn_robot_name: cleanerBot1` `spawn_robot_type: CleanerBotA` |
| cleaner_charger_2 | `is_charger: true` `spawn_robot_name: cleanerBot2` `spawn_robot_type: CleanerBotA` |
| Zonas de limpieza | `name: <zona>` `is_cleaning_zone: true` (ej: `clean_hall`) |

**Fleet Delivery — Graph 1 (DeliveryRobot, 2 robots)**

| Waypoint | Propiedades |
|----------|-------------|
| delivery_charger_1 | `is_charger: true` `spawn_robot_name: deliveryBot1` `spawn_robot_type: DeliveryRobot` |
| delivery_charger_2 | `is_charger: true` `spawn_robot_name: deliveryBot2` `spawn_robot_type: DeliveryRobot` |
| pickup_point | `pickup_dispenser: coke_dispenser` |
| dropoff_point | `dropoff_ingestor: coke_ingestor` |

> **Importante:** Los nombres de chargers estan hardcodeados en los fleet configs.
> Cada waypoint debe estar dibujado en lanes del graph correcto (0, 1 o 3).

### Anadir un robot nuevo

Para anadir un robot a una flota existente:

1. En Traffic Editor, crear un waypoint nuevo en una lane del graph correcto
2. Darle propiedades: `is_charger: true`, `spawn_robot_name: <nombre>`, `spawn_robot_type: <tipo>`
3. Anadir el robot al fleet config correspondiente (ver `src/project/project_config/config/estce/`)
4. Regenerar world y nav graphs (seccion 2)
5. Recompilar (seccion 3)

---

## 2. Regenerar el mundo y nav graphs

> Hacer esto **cada vez** que se guarde un cambio en el Traffic Editor.
> Se necesitan DOS comandos: uno para el world SDF y otro para los nav graphs.
> Se ejecuta **dentro del contenedor**.

Si ya hay un contenedor corriendo (ej: `rmf_demos`):

```bash
docker exec -it rmf_demos bash
```

Si no, crear uno:

```bash
rocker --nvidia=gpus --x11 --user --home \
  -- ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest bash
```

Dentro del contenedor:

```bash
source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash
cd ~/rmf-ws/src/project/project_maps/maps/estce

# 1. Regenerar el SDF world (tarda ~10 seg, genera estce.world + models/)
ros2 run rmf_building_map_tools building_map_generator gazebo \
  estce.building.yaml \
  estce.world \
  ./models

# 2. Regenerar los nav graphs (IMPRESCINDIBLE — sin esto los chargers no se encuentran)
ros2 run rmf_building_map_tools building_map_generator nav \
  estce.building.yaml \
  nav_graphs/
```

> Si se quiere empezar limpio, borrar los ficheros generados antes de regenerar:
> ```bash
> rm -f estce.world
> rm -f nav_graphs/0.yaml nav_graphs/1.yaml nav_graphs/3.yaml
> rm -rf models/estce_L1 models/estce_L2
> ```

---

## 3. Recompilar el workspace

> Hacer esto tras regenerar el mundo o modificar cualquier fichero de codigo/config.
> Se ejecuta **dentro del contenedor**.
> IMPORTANTE: si conda esta activo, hacer `conda deactivate` antes.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rmf-ws
colcon build --symlink-install \
  --packages-select project_assets project_config project_fleet_adapter \
  project_maps project_simulation \
  --paths src/project/*
```

> El flag `--paths src/project/*` es necesario para evitar el error de paquetes
> duplicados si hay copias del proyecto en otros directorios del home.

---

## 4. Lanzar la simulacion completa

Abrir **tres terminales** en paralelo. Respetar el orden.

### 4.1 Terminal 1 — API Server (arrancar PRIMERO)

> Se ejecuta en el **host**. El API server es el puente entre ROS 2 y el dashboard web.

```bash
docker run --network host --rm -it \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
```

### 4.2 Terminal 2 — Simulacion (Gazebo + RViz + Fleet adapters)

> Se ejecuta **dentro del contenedor** rocker.

Crear el contenedor:

```bash
rocker --nvidia=gpus --x11 --user --home \
  --name rmf_demos \
  -- ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest bash
```

Dentro del contenedor:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source ~/rmf-ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/rmf-ws/fuel_models

ros2 launch project_simulation estce.launch.xml \
  sim_update_rate:=50 \
  server_uri:=ws://localhost:8000/_internal
```

> **NVIDIA rendering:** Las variables `__NV_PRIME_RENDER_OFFLOAD` y
> `__GLX_VENDOR_LIBRARY_NAME` fuerzan el uso de la GPU NVIDIA en vez del
> driver Mesa (iris), que falla dentro del contenedor por falta de acceso
> a `/dev/dri`. Sin estas variables, RViz2 no abre ventana.

> **Sobre `server_uri`:** Conecta los nodos RMF con el API server para que el
> dashboard reciba datos. Sin este parametro, Gazebo funciona pero el dashboard
> no mostrara tareas ni trayectorias.

> **Modo headless** (sin GUI de Gazebo, mas ligero):
> Anadir `headless:=true` al final del comando de launch.

### 4.3 Terminal 3 — Dashboard web

> Se ejecuta en el **host**. El dashboard es una app React que muestra el mapa,
> robots, puertas, ascensores y permite despachar tareas.

```bash
docker run --network host --rm -it \
  -e RMF_SERVER_URL=http://localhost:8000 \
  -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
  ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

Abrir en el navegador: **http://localhost:3000**

---

## 5. Despachar tareas

> Se ejecuta **dentro del contenedor** de simulacion (Terminal 2).
> Abrir una nueva terminal y entrar al contenedor:

```bash
docker exec -it rmf_demos bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source ~/rmf-ws/install/setup.bash
```

### Patrol task (ronda entre waypoints)

```bash
ros2 run rmf_demos_tasks dispatch_patrol \
  -p patrol_charger_1 patrol_charger_2 \
  -n 3 --use_sim_time
```

### Clean task (limpiar una zona)

```bash
ros2 run rmf_demos_tasks dispatch_clean \
  -cs clean_hall --use_sim_time
```

### Delivery task (recoger y entregar)

```bash
ros2 run rmf_demos_tasks dispatch_delivery \
  -p pickup_point -ph coke_dispenser \
  -d dropoff_point -dh coke_ingestor \
  --use_sim_time
```

> Sustituir los nombres de waypoints por los definidos en el Traffic Editor.
> Tambien se pueden despachar tareas desde el dashboard (http://localhost:3000).

---

## 6. Flotas y puertos

| Flota | Tipo de robot | Robots | Nav Graph | Puerto fleet manager |
|-------|---------------|--------|-----------|----------------------|
| patrol_fleet | TinyRobot | 4 | 0 | 22012 |
| cleaner_fleet | CleanerBotA | 2 | 3 | 22013 |
| delivery_fleet | DeliveryRobot | 2 | 1 | 22014 |

API de cada fleet manager: `http://localhost:<puerto>/docs`

---

## 7. Problemas resueltos

### 7.1 Crash del ordenador al lanzar la simulacion
- **Causa:** 1580 modelos + 0 MB swap = OOM killer.
- **Solucion:** Crear 16 GB de swap (ver Requisitos previos).

### 7.2 Dashboard no muestra el mapa ni robots
- **Causa:** El API server (contenedor separado) y la simulacion usaban distintos
  middlewares DDS. Por defecto, el contenedor rmf_demos usa `rmw_fastrtps_cpp` y
  el API server puede usar otro. Ademas, FastRTPS usa shared memory, que no funciona
  entre contenedores con IPC namespaces distintos.
- **Solucion:** Usar **CycloneDDS** en ambos contenedores y limitar el discovery
  a localhost:
  ```
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ```
  Estas variables deben estar definidas **antes** de lanzar cualquier nodo ROS 2
  tanto en el contenedor de simulacion como en el del API server.

### 7.3 `package 'rmf_visualization_schedule' not found`
- **Causa:** Lanzar la simulacion fuera del contenedor Docker.
- **Solucion:** Todo se ejecuta dentro de rocker con `rmf_demos:jazzy-rmf-latest`.

### 7.4 `Cannot find waypoint named [X] in the navigation graph`
- **Causa:** No se regeneraron los nav graphs tras editar waypoints en Traffic Editor.
- **Solucion:** Ejecutar TAMBIEN `building_map_generator nav`.

### 7.5 `Connection to ws://localhost:8000/_internal failed`
- **Causa:** API server no arrancado o arrancado despues de la simulacion.
- **Solucion:** Arrancar el API server ANTES de la simulacion.

### 7.6 `No module named 'catkin_pkg'` al compilar
- **Causa:** Conda activo interfiere con Python del sistema.
- **Solucion:** `conda deactivate` antes de `colcon build`.

### 7.7 Paquetes duplicados al compilar con colcon
- **Causa:** colcon escanea todo el home y encuentra copias del proyecto en otros directorios.
- **Solucion:** Usar `--paths src/project/*` en el comando de colcon build.

### 7.8 RViz2 no abre ventana (MESA error: Failed to query drm device)
- **Causa:** Dentro del contenedor, Qt/RViz2 intenta usar el driver Mesa (iris/Intel)
  en vez de NVIDIA. Mesa falla porque `/dev/dri/card*` tiene permisos de root.
- **Solucion:** Forzar rendering NVIDIA con estas variables de entorno
  (ya anadidas al `.bashrc` del contenedor):
  ```
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  ```

### 7.9 `not found: "/opt/ros/humble/local_setup.bash"` al hacer source
- **Causa:** `install/setup.bash` se genero fuera del contenedor (con Humble) y
  tiene hardcodeadas las rutas del host. Dentro del contenedor (Jazzy) esas rutas
  no existen.
- **Solucion:** Recompilar dentro del contenedor con `source ~/rmf_ws/install/setup.bash`
  para que se regenere la cadena correcta (Jazzy → rmf_demos_ws → rmf_ws → rmf-ws).

### 7.10 Warnings inofensivos al arrancar

| Mensaje | Causa | Accion |
|---------|-------|--------|
| `unable to open image: .../Nivel 0 - TD_n0-1.png` | PNGs del plano 2D no copiados | Ignorar, la sim funciona |
| `/dev/dri/renderD129: Permission denied` | Nodo DRI secundario sin acceso | Ignorar |
| `set_retreat_to_charger_interval: no longer used` | Parametro obsoleto en fleet configs | Ignorar |
| `JointFeatures: Velocity control / effort limit` | Robots sin limite de esfuerzo | Ignorar |
| `zink: PERF WARNING! > 100 copy boxes` | Rendering software de muchos objetos | Ignorar |

---

## 8. Estructura del proyecto

```
~/rmf-ws/
├── src/project/
│   ├── project_assets/             # Modelos de robots (TinyRobot, CleanerBotA, DeliveryRobot)
│   ├── project_config/
│   │   ├── launch/
│   │   │   ├── common.launch.xml   # Nodos RMF comunes (schedule, map server, visualizer...)
│   │   │   └── estce.launch.xml    # Configuracion de las 3 flotas de ESTCE
│   │   └── config/estce/
│   │       ├── patrol_fleet_config.yaml    # 4 TinyRobots, graph 0, puerto 22012
│   │       ├── cleaner_fleet_config.yaml   # 2 CleanerBotA, graph 3, puerto 22013
│   │       └── delivery_fleet_config.yaml  # 2 DeliveryRobots, graph 1, puerto 22014
│   ├── project_fleet_adapter/      # Adaptador de flota Python (FastAPI + ROS 2)
│   │   ├── fleet_adapter.py        # Nodo ROS 2 que conecta RMF con los robots
│   │   ├── fleet_manager.py        # Servidor FastAPI (endpoints REST)
│   │   └── RobotClientAPI.py       # Gestion de estado de cada robot
│   ├── project_maps/
│   │   └── maps/estce/
│   │       ├── estce.building.yaml # FUENTE DE VERDAD (editar con Traffic Editor)
│   │       ├── estce.world         # SDF generado (~930 KB, ~1580 modelos) — NO editar
│   │       ├── estce_L1.png        # Plano planta 1
│   │       ├── estce_L2.png        # Plano planta 2
│   │       ├── models/             # Meshes de suelo/paredes generados — NO editar
│   │       └── nav_graphs/
│   │           ├── 0.yaml          # Patrol fleet (graph 0)
│   │           ├── 1.yaml          # Delivery fleet (graph 1)
│   │           └── 3.yaml          # Cleaner fleet (graph 3)
│   └── project_simulation/
│       └── launch/
│           ├── estce.launch.xml    # Entry point (incluye config + gazebo)
│           └── simulation.launch.xml
├── fuel_models/                    # Symlinks CamelCase -> cache Fuel (~/.gz/fuel/)
├── download_fuel_models.sh         # Descarga los ~40 modelos de Gazebo Fuel al cache
├── setup_fuel_symlinks.sh          # Crea symlinks CamelCase en fuel_models/
└── fix_lift.py                     # Patch para bug de segfault en plugin de lifts
```

### Ficheros clave

| Fichero | Para que sirve | Cuando editarlo |
|---------|----------------|-----------------|
| `estce.building.yaml` | Define el mapa, waypoints, puertas, lifts, robots | Cuando se modifique el edificio |
| `patrol_fleet_config.yaml` | Config de la flota patrol (robots, chargers, capacidades) | Cuando se anadan/quiten patrol robots |
| `cleaner_fleet_config.yaml` | Config de la flota cleaner | Cuando se anadan/quiten cleaner robots |
| `delivery_fleet_config.yaml` | Config de la flota delivery | Cuando se anadan/quiten delivery robots |
| `common.launch.xml` | Nodos RMF comunes a cualquier entorno | Raramente |
| `estce.launch.xml` (config) | Define que flotas se lanzan y con que graphs | Cuando se anada/quite una flota |

---

## 9. Referencia rapida: flujo de trabajo completo

```
1. Editar mapa (Traffic Editor)           ← Solo si hay cambios en el edificio
         |
         v
2. Regenerar world + nav graphs           ← Dentro del contenedor
         |
         v
3. Recompilar (colcon build)              ← Dentro del contenedor
         |
         v
4. Lanzar API Server (Terminal 1)         ← Host (docker run)
         |
         v
5. Lanzar Simulacion (Terminal 2)         ← Dentro del contenedor (con CycloneDDS)
         |
         v
6. Lanzar Dashboard (Terminal 3)          ← Host (docker run)
         |
         v
7. Abrir http://localhost:3000            ← Navegador
         |
         v
8. Despachar tareas (Terminal 4)          ← Dentro del contenedor
```

---

## 10. Hardware de referencia

| Componente | Especificacion |
|------------|----------------|
| CPU | i7-10870H (16 hilos) |
| RAM | 15 GB + 16 GB swap |
| GPU | RTX 3060 Laptop (6 GB VRAM) |
| Disco | NVMe 432 GB |
| OS host | Ubuntu + ROS 2 Humble |
| Contenedor | ROS 2 Jazzy (rmf_demos) |
| Gazebo | Harmonic (gz-sim 8) |
| DDS | CycloneDDS (rmw_cyclonedds_cpp) |
