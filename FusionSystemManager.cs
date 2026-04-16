// FusionSystemManager.cs
// Sensor-Fusion Tracking System — AIR Group, ESI-UCLM
//
// Orquestador central del sistema de fusión sensorial.
// Conecta tres subsistemas: visión (YOLO), BLE (IMU + FSR) y fusión (FusionTracker por cubo).
//
// Responsabilidades:
//   - Spawnea un FusionTracker por cubo cuando se conecta su periférico BLE.
//   - Enruta las detecciones visuales al tracker correcto según color.
//   - Controla el inicio/parada de la sesión (manual o automático).
//   - Modo experimentación para ablación y comparación con otros sistemas.
//
// Historial relevante:
//   - La calibración se relajó de 4 keypoints obligatorios a 3+.
//     El primer modelo YOLO no daba keypoints fiables salvo en poses
//     concretas; el modelo actual es más estable y nos lo permite.
//   - IsGeometricValid pasó de ser un gate duro a señal de calidad
//     por la misma razón: rechazaba demasiadas detecciones válidas.
//   - El filtro de falsos positivos de mano se añadió después porque
//     con ciertas iluminaciones YOLO confundía el puño cerrado/semicerrado
//     con un cubo, causando intermitencias de posición.

using UnityEngine;
using System.Collections.Generic;

public class FusionSystemManager : MonoBehaviour
{
    // ── Binding color ↔ dispositivo BLE ↔ prefab ─────────────────────
    // Cada cubo físico tiene un color, se anuncia por BLE con un nombre
    // que contiene ese color, y tiene un prefab asociado para su gemelo digital.

    [System.Serializable]
    public class ColorBinding
    {
        // Substring para matchear contra el nombre BLE (case-insensitive). Ej: "Red" matchea "RehabRed"
        public string nameFilter;
        public DetectedColor colorId;
        public GameObject prefab;
    }

    // ── Configuración en Inspector ───────────────────────────────────

    [Tooltip("Si true, arranca la inferencia al cargar la escena.")]
    public bool iniciarAutomaticamente = false;

    [Tooltip("Pipeline de visión que emite VisualDetection.")]
    public ObjectDetectionVisualizerV2 visionSource;

    [Tooltip("Agente YOLO. Se deshabilita hasta que arranque el juego.")]
    public Behaviour inferenceAgent;

    [Tooltip("Mano derecha OVR, se pasa a cada FusionTracker para manipulación por grip.")]
    public OVRHand sceneRightHand;

    [Tooltip("Mano izquierda OVR, alternativa para pacientes que usen la mano izquierda.")]
    public OVRHand sceneLeftHand;

    // Mano activa para tracking. Se elige desde MemoryGame según el toggle
    // del test de preparación del entorno. Por defecto, mano derecha.
    private OVRHand _activeHand;

    [Tooltip("Audio que suena al iniciar juego.")]
    public AudioSource successAudioSource;

    [Tooltip("Mappings color → nombre BLE → prefab.")]
    public List<ColorBinding> deviceMappings;

    public bool showDebugLogs = true;

    // ── Filtro de falsos positivos de mano ───────────────────────────
    // Añadido porque YOLO confundía puño cerrado/semicerrado con cubo
    // en ciertas condiciones de luz, generando saltos de posición.
    [Header("Hand False-Positive Filter")]
    [Tooltip("Rechaza detecciones cerca de la mano si el FSR confirma que no hay cubo.")]
    public bool enableHandFPFilter = true;

    [Tooltip("Distancia máxima (m) para considerar que la detección está 'en la mano'. 0.01 = 1cm.")]
    public float handFPMaxDistance = 0.01f;

    // ── Experimentación ──────────────────────────────────────────────
    // Para medir y comparar el rendimiento del sistema con otros setups.
    [Header("Experimentation")]
    public bool experimentationEnabled = false;

    [Tooltip("Si true, descarta detecciones estocásticamente.")]
    public bool experimentalGateDetections = false;

    [Range(0f, 1f)]
    [Tooltip("Probabilidad de que una detección pase el gate.")]
    public float detectionKeepProbability = 1f;

    [Tooltip("Seed para reproducibilidad.")]
    public int experimentalSeed = 12345;

    // ── Estado interno ───────────────────────────────────────────────

    // Flag de si la sesión está activa. Mientras sea false no se procesan detecciones ni BLE.
    private bool _juegoIniciado = false;

    // Diccionario que mapea cada color a su FusionTracker. Un tracker por cubo.
    private readonly Dictionary<DetectedColor, FusionTracker> _activeTrackers = new();

    // RNG con seed fija para que los experimentos de ablación sean reproducibles.
    private System.Random _rng;

    // ── API pública ──────────────────────────────────────────────────

    // Devuelve copia de la lista de trackers (copia para que nadie modifique el diccionario desde fuera)
    public List<FusionTracker> GetAllTrackers() => new List<FusionTracker>(_activeTrackers.Values);

    // Versión sin copia para iterar sin allocar memoria (útil en Update loops)
    public IEnumerable<FusionTracker> ActiveTrackers => _activeTrackers.Values;

    // Configura experimentación en runtime y propaga a todos los trackers activos.
    public void SetExperimentation(bool enabled, bool gateDetections = false,
                                   float keepProbability = 1f, int? seed = null)
    {
        experimentationEnabled = enabled;
        experimentalGateDetections = gateDetections;

        // Clamp para asegurar que la probabilidad queda entre 0 y 1
        detectionKeepProbability = Mathf.Clamp01(keepProbability);

        // Si nos pasan un seed nuevo, recreamos el RNG para que los resultados
        // sean reproducibles desde ese punto
        if (seed.HasValue)
        {
            experimentalSeed = seed.Value;
            _rng = new System.Random(experimentalSeed);
        }

        // Propagar el flag a cada tracker para que ajusten su comportamiento interno
        foreach (var tracker in _activeTrackers.Values)
        {
            if (tracker != null) tracker.SetExperimentation(enabled);
        }
    }

    /// <summary>
    /// Establece qué mano se usa para el tracking de grip.
    /// Llamado desde MemoryGame según la elección del paciente en la calibración.
    /// Propaga la mano activa a todos los trackers ya instanciados.
    /// </summary>
    public void SetActiveHand(bool useRightHand)
    {
        OVRHand chosen = useRightHand ? sceneRightHand : sceneLeftHand;
        string label = useRightHand ? "DERECHA" : "IZQUIERDA";

        if (chosen == null)
        {
            Debug.LogError($"[FSM] ¡¡ {(useRightHand ? "sceneRightHand" : "sceneLeftHand")} " +
                           $"NO está asignado en el Inspector !!");
            chosen = useRightHand ? sceneLeftHand : sceneRightHand;
        }

        _activeHand = chosen;

        // Log diagnóstico: nombre del GameObject + InstanceID para verificar
        // que no son el mismo objeto en ambos slots
        if (_activeHand != null)
            Debug.Log($"[FSM] ★ Mano activa = {label} → " +
                      $"GameObject: \"{_activeHand.gameObject.name}\" " +
                      $"(InstanceID={_activeHand.GetInstanceID()})");
        else
            Debug.LogError("[FSM] ¡¡ _activeHand es NULL — el grip NO funcionará !!");

        // Propagar a todos los trackers ya existentes
        foreach (var tracker in _activeTrackers.Values)
        {
            if (tracker != null) tracker.trackingHand = _activeHand;
        }
    }

    // ── Lifecycle ────────────────────────────────────────────────────

    private void Start()
    {
        // ── Validación de manos ──────────────────────────────────────
        if (sceneRightHand == null)
            Debug.LogError("[FSM] sceneRightHand NO asignado en Inspector.");
        if (sceneLeftHand == null)
            Debug.LogError("[FSM] sceneLeftHand NO asignado en Inspector.");
        if (sceneRightHand != null && sceneLeftHand != null &&
            sceneRightHand.GetInstanceID() == sceneLeftHand.GetInstanceID())
            Debug.LogError("[FSM] ¡¡ sceneRightHand y sceneLeftHand apuntan al MISMO objeto !! " +
                           $"Ambos son \"{sceneRightHand.gameObject.name}\". " +
                           "Asigna un OVRHand diferente a cada slot.");
        else if (sceneRightHand != null && sceneLeftHand != null)
            Debug.Log($"[FSM] Manos OK: R=\"{sceneRightHand.gameObject.name}\" " +
                      $"L=\"{sceneLeftHand.gameObject.name}\"");

        // Solo asignar la mano por defecto si SetActiveHand no se llamó antes.
        // SetActiveHand se llama desde MemoryGame ANTES de activar detectionRoot,
        // por lo que _activeHand puede ya estar configurada a la mano izquierda.
        if (_activeHand == null)
            _activeHand = sceneRightHand;

        // Suscribirnos a los eventos que nos interesan:
        // - Cuando la cámara detecta cubos (visión)
        // - Cuando un dispositivo BLE se conecta
        if (visionSource != null) visionSource.OnVisualDetections += HandleVisualDetections;
        BLEManager.OnConnected += HandleBleConnected;

        // Puede que haya dispositivos BLE que se conectaron antes de que este script
        // arrancase (ej: viniendo de otra escena). Los registramos ahora.
        RegisterAlreadyConnectedDevices();

        // Inicializar el RNG con la seed configurada en Inspector
        _rng = new System.Random(experimentalSeed);

        if (!iniciarAutomaticamente)
        {
            // Si no arrancamos automáticamente, deshabilitamos YOLO y visión.
            // Se activarán cuando alguien llame a IniciarJuego() (ej: desde un botón UI).
            if (inferenceAgent != null) inferenceAgent.enabled = false;
            if (visionSource != null) visionSource.enabled = false;
        }
        else
        {
            // Modo automático: marcamos como iniciado directamente
            _juegoIniciado = true;
        }
    }

    private void OnDestroy()
    {
        // Desuscribirnos para no dejar callbacks huérfanos al destruir este objeto.
        // Si no hacemos esto, Unity puede intentar llamar a funciones de un objeto destruido.
        if (visionSource != null) visionSource.OnVisualDetections -= HandleVisualDetections;
        BLEManager.OnConnected -= HandleBleConnected;
    }

    // ── Control de sesión ────────────────────────────────────────────

    // Arranca la sesión de tracking. Llamado desde UI o desde otro script.
    public void IniciarJuego()
    {
        // Evitar doble inicio
        if (_juegoIniciado) return;
        _juegoIniciado = true;

        // Activar la inferencia YOLO y el pipeline de visión
        if (inferenceAgent != null) inferenceAgent.enabled = true;
        if (visionSource != null) visionSource.enabled = true;

        // Registrar BLEs que se hayan conectado mientras el juego estaba parado.
        // Esto pasa si el usuario conecta los cubos antes de darle a "Iniciar".
        RegisterAlreadyConnectedDevices();

        // Feedback sonoro para el usuario
        if (successAudioSource != null) successAudioSource.Play();
    }

    // Para la sesión. Los trackers siguen en memoria pero dejan de recibir datos.
    public void DetenerJuego()
    {
        _juegoIniciado = false;

        // Desactivar inferencia y visión para no gastar recursos innecesariamente
        if (inferenceAgent != null) inferenceAgent.enabled = false;
        if (visionSource != null) visionSource.enabled = false;
    }

    // ── Gestión de dispositivos BLE ──────────────────────────────────

    // Recorre los dispositivos BLE ya conectados y crea trackers para ellos.
    // Necesario porque el BLE puede conectarse antes de que este script exista
    // (ej: persistido entre escenas) o antes de que el juego arranque.
    private void RegisterAlreadyConnectedDevices()
    {
        if (BLEManager.Instance == null) return;

        // Para cada MAC ya conectada, simulamos el evento de conexión
        foreach (string mac in BLEManager.Instance.connectedDevices.Keys)
            HandleBleConnected(mac);
    }

    // Callback que se ejecuta cuando un periférico BLE se conecta.
    // Busca en deviceMappings a qué color corresponde y spawnea su tracker.
    private void HandleBleConnected(string macAddress)
    {
        // Si el juego no ha arrancado, ignoramos conexiones BLE.
        // Ya las pillaremos en RegisterAlreadyConnectedDevices() cuando arranque.
        if (!_juegoIniciado) return;

        // Obtener los datos del dispositivo por su MAC
        DeviceData device = BLEManager.Instance.GetDeviceData(macAddress);
        if (device == null) return;

        // Buscar qué mapping le corresponde comparando el nombre del dispositivo
        // con cada nameFilter. Ej: si el BLE se llama "RehabRed" y hay un mapping
        // con nameFilter="Red", matchea y se le asigna el color/prefab de ese mapping.
        foreach (ColorBinding mapping in deviceMappings)
        {
            if (device.name.IndexOf(mapping.nameFilter, System.StringComparison.OrdinalIgnoreCase) >= 0)
            {
                SpawnOrUpdateTracker(mapping, macAddress);
                return; // Solo un mapping por dispositivo
            }
        }
    }

    // Busca un tracker existente para ese color, o instancia el prefab si no hay ninguno.
    // Después lo configura con la mano, el color, el nombre BLE y lo registra.
    private void SpawnOrUpdateTracker(ColorBinding mapping, string mac)
    {
        // Primero buscar en la escena por si ya hay un tracker de ese color
        // (puede pasar al volver a una escena o al reconectar un BLE)
        FusionTracker tracker = FindExistingTracker(mapping.colorId);

        // Si no existe y tenemos prefab, instanciamos uno nuevo
        if (tracker == null && mapping.prefab != null)
        {
            GameObject instance = Instantiate(mapping.prefab);
            instance.name = $"Tracker_{mapping.colorId}";

            // Intentar obtener el FusionTracker del prefab. Si el prefab no lo tiene,
            // se lo añadimos nosotros (por si se nos olvidó ponerlo en el prefab).
            tracker = instance.GetComponent<FusionTracker>() ?? instance.AddComponent<FusionTracker>();
        }

        if (tracker == null) return;

        // Configurar el tracker con todo lo que necesita
        if (_activeHand != null)
        {
            tracker.trackingHand = _activeHand;
            Debug.Log($"[FSM] Tracker {mapping.colorId}: trackingHand → " +
                      $"\"{_activeHand.gameObject.name}\" (ID={_activeHand.GetInstanceID()})");
        }
        else
        {
            Debug.LogError($"[FSM] Tracker {mapping.colorId}: _activeHand es NULL, " +
                           $"¡trackingHand no se ha asignado!");
        }

        tracker.myColorIdentity = mapping.colorId;   // qué color de cubo representa
        tracker.assignedName = mapping.nameFilter;     // nombre para buscar su BLE en BLEManager
        tracker.ResetState();                          // limpiar estado previo
        tracker.SetExperimentation(experimentationEnabled);

        // Registrarlo en nuestro diccionario para poder enrutarle detecciones luego
        _activeTrackers[mapping.colorId] = tracker;
    }

    // Busca en toda la escena un FusionTracker que ya tenga asignado este color.
    // FindObjectsByType es la alternativa moderna a FindObjectsOfType (sin deprecation warning).
    private static FusionTracker FindExistingTracker(DetectedColor color)
    {
        foreach (FusionTracker t in FindObjectsByType<FusionTracker>(FindObjectsSortMode.None))
        {
            if (t.myColorIdentity == color) return t;
        }
        return null;
    }

    // ── Procesado de detecciones visuales ─────────────────────────────
    // Este callback se ejecuta cada vez que la cámara+YOLO emite un batch de detecciones.
    // Cada detección lleva: posición 3D, color detectado, clase (CuboSensor/CuboNormal),
    // keypoints y si pasa la validación geométrica.

    private void HandleVisualDetections(List<VisualDetection> detections)
    {
        if (!_juegoIniciado) return;

        foreach (VisualDetection det in detections)
        {
            // Buscar el tracker que corresponde al color de esta detección.
            // Si no hay tracker para ese color (ej: cubo no conectado por BLE), lo ignoramos.
            if (!_activeTrackers.TryGetValue(det.ColorCategory, out FusionTracker tracker))
                continue;

            // Filtro de falsos positivos de mano:
            // Si la detección está pegada a la mano pero el FSR dice que no hay cubo,
            // YOLO está viendo el puño, no un cubo real. Saltamos toda la detección
            // (tanto posición como calibración) para evitar intermitencias.
            if (enableHandFPFilter && IsHandFalsePositive(tracker, det.Position))
            {
                if (showDebugLogs)
                    Debug.Log($"[FSM:{det.ColorCategory}] Hand FP filter: skipped");
                continue;
            }

            // CuboSensor = YOLO detectó la cara del cubo que tiene la pegatina/sticker.
            // Solo en esta cara tenemos keypoints para calibrar la orientación.
            if (det.Class == CubeClass.CuboSensor)
            {
                // Leer la rotación actual del IMU (sensor BLE dentro del cubo)
                Quaternion sensorRot = GetSensorRotation(tracker);

                // Actualizar la visualización de debug (esferitas en los keypoints
                // y flechas de dirección). Esto siempre se hace, independientemente
                // de si vamos a calibrar o no, para poder depurar visualmente.
                tracker.UpdateKeypointDebug(
                    det.KeypointsWorld,
                    det.KeypointWorldValid,
                    sensorRot);

                // Contar cuántos keypoints son válidos en este frame
                int validCount = CountValidKeypoints(det.KeypointWorldValid);

                // Calibración de yaw (orientación horizontal):
                // Necesitamos al menos 3 keypoints válidos Y que el IMU esté enviando datos
                // (sensorRot != identity significa que hay datos reales del IMU).
                //
                // Antes exigíamos 4/4 + validación geométrica, pero el primer modelo YOLO
                // no daba keypoints fiables. Con el modelo actual funciona bien con 3+.
                if (validCount >= 3 && sensorRot != Quaternion.identity)
                {
                    tracker.ApplyKeypointCalibration(
                        det.KeypointsWorld,
                        det.KeypointWorldValid,
                        sensorRot,
                        det.IsGeometricValid,   // señal de calidad, ya no bloquea la calibración
                        validCount);
                }
            }

            // ── Logging: datos de visión ──
            if (experimentationEnabled)
            {
                float[] vis = new float[4];
                for (int i = 0; i < 4; i++)
                    vis[i] = (det.KeypointWorldValid != null &&
                              i < det.KeypointWorldValid.Length &&
                              det.KeypointWorldValid[i]) ? 1f : 0f;

                string calibStatus = tracker.HasInitialCalibration ? "ongoing" : "initial";
                BackgroundDataLogger.Instance?.LogVisionData(
                    det.ColorCategory.ToString(), det.BBox, det.Score,
                    det.KeypointsPixel, vis, calibStatus);
            }

            // Siempre actualizar la posición visual del tracker, sea CuboSensor o CuboNormal.
            // La posición viene de la bounding box de YOLO proyectada a 3D.
            tracker.UpdateVisualPosition(det.Position);
        }
    }

    // Lee la rotación IMU del cubo vinculado a este tracker.
    // mountingRotation compensa cómo está montado físicamente el sensor dentro del cubo
    // (no siempre está alineado con las caras del cubo).
    private static Quaternion GetSensorRotation(FusionTracker tracker)
    {
        if (BLEManager.Instance == null) return Quaternion.identity;

        // Buscar el dispositivo BLE por nombre (ej: "Red" busca el dispositivo cuyo nombre contiene "Red")
        DeviceData data = BLEManager.Instance.GetDeviceByName(tracker.assignedName);

        // Si no hay datos o el quaternion tiene w=0, el IMU aún no ha enviado nada válido
        if (data == null || data.orientation.w == 0f) return Quaternion.identity;

        // Aplicar la rotación de montaje y normalizar para evitar drift numérico
        return Quaternion.Euler(tracker.mountingRotation) * data.orientation.normalized;
    }

    // ── Filtro de falsos positivos de mano ───────────────────────────
    // Problema original: con ciertas luces, YOLO detectaba un cubo donde solo
    // había un puño cerrado o semicerrado. Esto hacía que la posición del cubo
    // digital saltase entre su posición real y la mano.
    //
    // Solución: cruzar la posición de la detección con el FSR (sensor de presión).
    // Si la detección está cerca de la mano Y el FSR dice que no hay presión,
    // es un falso positivo y lo descartamos.

    private bool IsHandFalsePositive(FusionTracker tracker, Vector3 detectedPos)
    {
        // Si no tenemos referencia de la mano o no está siendo trackeada,
        // no podemos filtrar nada. Al arrancar, OVRHand puede estar en (0,0,0)
        // y eso causaría rechazos falsos.
        if (_activeHand == null || !_activeHand.IsTracked) return false;

        // ¿Está la detección cerca de la mano?
        float dist = Vector3.Distance(detectedPos, _activeHand.transform.position);
        if (dist > handFPMaxDistance) return false;

        // Sí está cerca. Ahora preguntamos: ¿el usuario realmente tiene un cubo en la mano?
        // Si el tracker dice que está en grip → la detección es legítima, es el cubo real.
        if (tracker.IsGripping) return false;

        // Doble check leyendo el valor raw del FSR directamente.
        // Esto cubre el caso edge donde el usuario acaba de agarrar el cubo
        // pero la máquina de estados de grip aún no ha transicionado.
        if (BLEManager.Instance != null)
        {
            DeviceData data = BLEManager.Instance.GetDeviceByName(tracker.assignedName);
            if (data != null && data.grip >= tracker.gripThresholdEnter)
                return false;   // Hay presión en el FSR → cubo real en mano
        }

        // Cerca de la mano + sin presión FSR → casi seguro que YOLO ve el puño
        return true;
    }

    // ── Utilidades ───────────────────────────────────────────────────

    // Cuenta cuántos keypoints son válidos en este frame.
    // Un keypoint es inválido cuando YOLO no lo detectó o su confianza es muy baja.
    private static int CountValidKeypoints(bool[] kptsValid)
    {
        if (kptsValid == null) return 0;

        int count = 0;
        for (int i = 0; i < kptsValid.Length; i++)
        {
            if (kptsValid[i]) count++;
        }
        return count;
    }
}