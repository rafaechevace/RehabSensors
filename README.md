# RehabSensors

Código fuente de la aplicación de realidad mixta desarrollada como parte del Trabajo Fin de Grado *"Sensorización de objetos físicos en entornos inmersivos"*.

El sistema, implementado en Unity para Meta Quest 3, integra el seguimiento de un cubo físico instrumentado (IMU + FSR + BLE) mediante fusión sensorial con visión por computador (YOLOv8-Pose), junto con un juego serio orientado a la rehabilitación del miembro superior.

## Contexto del proyecto

Este trabajo se enmarca en la línea de investigación del grupo **AIR (Artificial Intelligence and Representation)** de la Escuela Superior de Informática (ESI), Universidad de Castilla-La Mancha (UCLM), en colaboración clínica con el **Hospital Nacional de Parapléjicos de Toledo (HNPT)**.

- **Grupo de investigación:** AIR — [air.esi.uclm.es](https://air.esi.uclm.es/air/)
- **Proyecto asociado:** [REHAB-IMMERSIVE](https://air.esi.uclm.es/rehab/) — Plataforma de Realidad Virtual inmersiva para la rehabilitación de miembros superiores
- **Financiación:** Ministerio de Ciencia e Innovación (MCIN/AEI)
- **Código de referencia:** PID2020-117361RB-C21
- **Investigadores principales:** Carlos González Morcillo y [Javier A. Albusac Jiménez](https://www.esi.uclm.es/www/jalbusac/)
- **Colaborador clínico:** Hospital Nacional de Parapléjicos de Toledo (HNPT)

## Contenido del repositorio

Scripts de Unity (C#) que componen la aplicación de realidad mixta:

| Script | Descripción |
|---|---|
| `BLEManager.cs` | Gestiona la conexión BLE con el cubo instrumentado y expone sus datos (batería, FSR, orientación IMU, movimiento). |
| `YoloPoseAgent.cs` | Agente de inferencia YOLOv8-Pose: detecta los cubos en el feed de la cámara passthrough y extrae bounding boxes y keypoints. |
| `ObjectDetectionVisualizerV2.cs` | Procesa y visualiza las detecciones (clase, color) sobre los objetos reconocidos. |
| `FusionTracker.cs` | Tracker multimodal por objeto: combina datos BLE (IMU/FSR), posición visual y reglas de forma para calcular la pose final. |
| `FusionSystemManager.cs` | Orquestador del sistema: asocia detecciones visuales con los trackers físicos correspondientes. |
| `CubeContactAssesor.cs` | Evalúa el nivel de contacto/agarre del cubo (reposo, cercanía, contacto, agarre suave/firme). |
| `ShapePolicy.cs` | Clase base abstracta que define el contrato de geometría (keypoints, calibración) para cada tipo de objeto. |
| `CubeShapePolicy.cs` | Implementación de `ShapePolicy` específica para el cubo instrumentado. |
| `MemoryGame.cs` | Juego serio de rehabilitación: calibración, área de juego y tarea de colocación de cubos por color. |
| `BackgroundDataLogger.cs` | Registro de datos de sesión en CSV (sensores, visión, fusión, juego y cinemática) con fines de evaluación clínica. |
| `TutorialMedioManager.cs` | Gestiona los recursos audiovisuales de tutorial dentro de la aplicación. |

## Tecnologías

- Unity (Meta Quest 3, Meta XR SDK / MRUtilityKit)
- C#
- BLE (Bluetooth Low Energy)
- YOLOv8-Pose (detección y estimación de pose)

## Recursos relacionados

- **Datos experimentales e informes derivados:** [RehabSensorsData](https://github.com/rafaechevace/RehabSensorsData)

## Licencia

Este proyecto se distribuye bajo la licencia [Creative Commons Atribución-NoComercial-CompartirIgual 4.0 Internacional (CC BY-NC-SA 4.0)](https://creativecommons.org/licenses/by-nc-sa/4.0/deed.es).

Esto permite el uso, copia y modificación del código citando la autoría original, siempre con fines no comerciales, y exige que cualquier trabajo derivado se distribuya bajo la misma licencia.

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

## Contacto

Para dudas sobre el proyecto, contactar con el autor o con el grupo de investigación AIR (ESI-UCLM).
