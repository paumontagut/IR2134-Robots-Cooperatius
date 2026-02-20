# IR2134 — Cooperative Robots (2025-2026)

Repositorio de practicas y proyectos de **IR2134 - Robots Cooperatius** (curso 2025-2026).

## Requisitos

Descargar las imagenes Docker antes de empezar con cualquier practica:

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest
docker pull ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
docker pull ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

Tambien necesitas [rocker](https://github.com/osrf/rocker) y [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

```bash
pip install rocker
```

## Contenido

| Carpeta | Descripcion |
|---------|-------------|
| [Traffic Editor](Traffic%20Editor/) | Mapas de edificios y simulacion con Traffic Editor + Gazebo Sim |
| [First Project](First%20Project/) | Proyecto Open-RMF completo con fleet adapter, API server y dashboard web (test1 + ICC Kyoto) |
| [Large Project](Large%20Project/) | **Proyecto final** — Simulacion del edificio ESTCE con 3 flotas (8 robots), 2 plantas, puertas, ascensores y dashboard web |
