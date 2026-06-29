using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Meta.XR.MRUtilityKit;
using UnityEngine.Serialization;

public class FusionSystemManager : MonoBehaviour
{
    // Orquestador de la arquitectura: conecta BLE, vision, mano activa y trackers fisicos.
    // Aqui se asocian detecciones visuales con objetos inteligentes y se propagan ajustes comunes.
    // Para entender el flujo: BLE crea/actualiza trackers; vision envia lotes; este manager
    // decide que deteccion pertenece a que tracker antes de delegar posicion/calibracion.
    //
    // Idea clave: este script no calcula la pose final del objeto. Solo responde a eventos
    // y reparte informacion al FusionTracker correcto.
    [System.Serializable]
    public class DeviceBinding
    {
        // Relaciona identidad BLE, color esperado y prefab fisico instanciado en la escena.
        // nameFilter se compara contra DeviceData.name; colorId ayuda a emparejar detecciones visuales.
        public string nameFilter;
        public DetectedColor colorId;
        public GameObject prefab;
    }

    // --- Referencias y configuracion global de la escena ---
    // autoStart permite probar la fusion sin pasar por el flujo completo de MemoryGame.
    [FormerlySerializedAs("iniciarAutomaticamente")]
    public bool autoStart = false;

    // Fuente de vision 3D y agente de inferencia que se encienden solo cuando empieza la partida.
    public ObjectDetectionVisualizerV2 visionSource;
    public Behaviour inferenceAgent;

    // Manos de escena: MemoryGame decide cual es activa y este manager la copia a todos los trackers.
    public OVRHand sceneRightHand;
    public OVRHand sceneLeftHand;
    private OVRHand _activeHand;

    // Feedback sonoro de inicio y tabla de correspondencia dispositivo BLE -> prefab/color.
    public AudioSource successAudioSource;
    public List<DeviceBinding> deviceMappings;
    public bool showDebugLogs = true;

    // Filtro anti falsos positivos: evita asignar detecciones pegadas a la mano cuando no corresponden.
    [Header("Hand False-Positive Filter")]
    public bool enableHandFPFilter = true;
    public float handFPMaxDistance = 0.30f;

    // Altura de mesa detectada por MRUK. Se propaga a los trackers para bloquear Y de forma coherente.
    [Header("MRUK Calibration")]
    public bool useMRUKTableHeight = true;
    private float _calibratedTableHeight = 0f;
    private bool _hasTableHeight = false;

    // Correccion fina opcional usando vision estable al inicio: compensa pequenos errores de profundidad/mesa.
    [Header("Auto Table Height Correction")]
    public bool autoCorrectTableHeightFromVision = true;
    public int autoTableCorrectionSampleCount = 24;
    public float maxAutoTableCorrection = 0.06f;

    // --- Estado interno de sesion ---
    // Lista activa y correccion vertical comun. Estos campos se reinician al iniciar/detener partida.
    private bool _gameStarted = false;
    private readonly List<FusionTracker> _activeTrackers = new();
    private readonly List<float> _autoTableCorrectionSamples = new();
    private bool _autoTableCorrectionApplied = false;
    private float _autoTableCorrectionValue = 0f;

    // Devuelve una copia para que otros sistemas no modifiquen la lista interna por accidente.
    public List<FusionTracker> GetAllTrackers() => new List<FusionTracker>(_activeTrackers);

    public IEnumerable<FusionTracker> ActiveTrackers => _activeTrackers;

    public void SetActiveHand(bool useRightHand)
    {
        // Todos los objetos fusionados siguen la mano elegida durante la calibracion.
        _activeHand = useRightHand ? sceneRightHand : sceneLeftHand;
        if (_activeHand == null)
        {
            Debug.LogError($"[FSM] {(useRightHand ? "sceneRightHand" : "sceneLeftHand")} is not assigned.");
            _activeHand = useRightHand ? sceneLeftHand : sceneRightHand;
        }

        foreach (var tracker in _activeTrackers)
            if (tracker != null) tracker.trackingHand = _activeHand;
    }

    private void Start()
    {
        // Start lo llama Unity antes del primer Update.
        // Vision y BLE se conectan por eventos para reaccionar aunque lleguen en distinto orden.
        // "Evento" significa: otro script avisa cuando hay datos nuevos y este metodo responde.
        if (_activeHand == null) _activeHand = sceneRightHand;

        if (visionSource != null) visionSource.OnVisualDetections += HandleVisualDetections;
        BLEManager.OnConnected += HandleBleConnected;

        if (inferenceAgent != null) inferenceAgent.enabled = false;
        if (visionSource != null) visionSource.enabled = false;

        if (autoStart)
        {
            StartGame();
        }
    }

    private void OnDestroy()
    {
        // Libera eventos para que no lleguen detecciones a un gestor destruido.
        if (visionSource != null) visionSource.OnVisualDetections -= HandleVisualDetections;
        BLEManager.OnConnected -= HandleBleConnected;
    }

    public void StartGame()
    {
        // Arranca la fusion de partida una sola vez y prepara la correccion de mesa.
        if (_gameStarted) return;
        _gameStarted = true;
        ResetAutoTableCorrection();
        StartCoroutine(SecuenciaInicioYOLO());
    }

    private IEnumerator SecuenciaInicioYOLO()
    {
        // Antes de activar vision, intenta fijar altura de mesa para que todos partan de la misma referencia.
        float timeout = 1.5f;
        while (!_hasTableHeight && timeout > 0)
        {
            CalibrateMRUKTableHeight();
            if (_hasTableHeight) break;
            timeout -= Time.deltaTime;
            yield return null;
        }

        if (inferenceAgent != null) inferenceAgent.enabled = true;
        if (visionSource != null) visionSource.enabled = true;
        RegisterAlreadyConnectedDevices();
        if (successAudioSource != null) successAudioSource.Play();
    }

    public void StopGame()
    {
        // Parar inferencia ahorra bateria y evita detecciones cuando no hay partida activa.
        _gameStarted = false;
        if (inferenceAgent != null) inferenceAgent.enabled = false;
        if (visionSource != null) visionSource.enabled = false;
    }

    public void CalibrateMRUKTableHeight()
    {
        if (!useMRUKTableHeight || MRUK.Instance == null) return;

        if (MRUKTableHeightUtility.TryGetHighestTableTop(
                out float tableTopY,
                out bool roomAvailable,
                showDebugLogs,
                "FusionSystemManager/MRUK"))
        {
            _calibratedTableHeight = tableTopY;
            _hasTableHeight = true;
            if (showDebugLogs) Debug.Log($"[FusionSystemManager/MRUK] Table calibrated: y={_calibratedTableHeight:F3}");
            PropagateTableHeight();
            return;
        }

        if (!roomAvailable)
        {
            if (showDebugLogs) Debug.LogWarning("[FusionSystemManager/MRUK] No room loaded yet.");
        }
        else
        {
            _hasTableHeight = false;
            if (showDebugLogs) Debug.LogWarning("[FusionSystemManager/MRUK] No TABLE/DESK anchor found. Height will not be locked by MRUK.");
        }
    }

    private void PropagateTableHeight()
    {
        // Copia la altura calibrada a todos los trackers ya activos.
        foreach (var tracker in _activeTrackers)
        {
            if (tracker != null)
            {
                tracker.tableWorldHeightY = _calibratedTableHeight;
                tracker.lockToTable = _hasTableHeight;
            }
        }
    }

    private void RegisterAlreadyConnectedDevices()
    {
        // Si BLE ya tenia dispositivos conectados, crea sus trackers al iniciar el juego.
        if (BLEManager.Instance == null) return;
        foreach (string mac in BLEManager.Instance.connectedDevices.Keys)
            HandleBleConnected(mac);
    }

    private void HandleBleConnected(string macAddress)
    {
        // Convierte una conexion BLE en tracker fisico cuando el juego esta corriendo.
        if (!_gameStarted) return;
        DeviceData device = BLEManager.Instance.GetDeviceData(macAddress);
        if (device == null) return;

        foreach (DeviceBinding mapping in deviceMappings)
        {
            if (device.name.IndexOf(mapping.nameFilter, System.StringComparison.OrdinalIgnoreCase) >= 0)
            {
                SpawnOrUpdateTracker(mapping, macAddress);
                return;
            }
        }
    }

    private void SpawnOrUpdateTracker(DeviceBinding mapping, string mac)
    {
        // Reutiliza trackers para evitar duplicados tras reconexion BLE o prefabs ya presentes.
        FusionTracker tracker = FindExistingTracker(mapping.nameFilter, mapping.colorId);

        if (tracker == null && mapping.prefab != null)
        {
            GameObject instance = Instantiate(mapping.prefab);
            instance.name = $"Tracker_{mapping.nameFilter}";
            if (!instance.TryGetComponent(out tracker))
                tracker = instance.AddComponent<FusionTracker>();
        }
        if (tracker == null) return;

        if (_activeHand != null) tracker.trackingHand = _activeHand;
        tracker.myColorIdentity = mapping.colorId;
        tracker.assignedName = mapping.nameFilter;
        tracker.ResetState();
        tracker.autoTableHeightCorrection = _autoTableCorrectionValue;

        if (_hasTableHeight && useMRUKTableHeight)
        {
            // La altura compartida mantiene coherentes vision, juego y objetos fusionados.
            tracker.tableWorldHeightY = _calibratedTableHeight;
            tracker.lockToTable = true;
        }
        else if (useMRUKTableHeight)
        {
            CalibrateMRUKTableHeight();
            if (_hasTableHeight)
            {
                tracker.tableWorldHeightY = _calibratedTableHeight;
                tracker.lockToTable = true;
            }
        }

        if (!_activeTrackers.Contains(tracker))
            _activeTrackers.Add(tracker);
    }

    private static FusionTracker FindExistingTracker(string assignedName, DetectedColor color)
    {
        // Busca un tracker ya instanciado con la misma identidad logica.
        foreach (FusionTracker tracker in FindObjectsByType<FusionTracker>(FindObjectsSortMode.None))
            if (tracker.assignedName == assignedName && tracker.myColorIdentity == color) return tracker;
        return null;
    }

    private void HandleVisualDetections(List<VisualDetection> detections)
    {
        // Primero reserva colores claros para resolver detecciones ambiguas dentro del mismo lote.
        // La asociacion se hace en capas: color/clase, movimiento IMU cerca de la mano y ultimo recurso espacial.
        // detections es la lista de objetos que YOLO vio en este frame; _activeTrackers son objetos fisicos reales.
        if (!_gameStarted) return;

        HashSet<FusionTracker> colorClaimedThisBatch = ClaimColorMatchedTrackers(detections);

        foreach (VisualDetection det in detections)
        {
            FusionTracker targetTracker = FindColorMatchedTracker(det);
            bool usedColorFallback = false;

            if (targetTracker == null)
                targetTracker = TryFindKinematicFallback(det, colorClaimedThisBatch, out usedColorFallback);

            if (targetTracker == null)
            {
                targetTracker = TryFindSpatialFallback(det, colorClaimedThisBatch, out bool usedSpatialFallback);
                usedColorFallback = usedSpatialFallback;
            }

            if (targetTracker == null) continue;

            ApplyDetectionToTracker(det, targetTracker, usedColorFallback);
        }
    }

    private HashSet<FusionTracker> ClaimColorMatchedTrackers(List<VisualDetection> detections)
    {
        // Esta primera pasada no mueve nada; solo reserva trackers con clase/color claro.
        // Sirve para que un fallback no "robe" un tracker que ya tuvo una deteccion fiable en el mismo lote.
        HashSet<FusionTracker> colorClaimedThisBatch = new();

        foreach (VisualDetection det in detections)
        {
            FusionTracker tracker = FindColorMatchedTracker(det);
            if (tracker != null)
            {
                colorClaimedThisBatch.Add(tracker);
            }
        }

        return colorClaimedThisBatch;
    }

    private FusionTracker FindColorMatchedTracker(VisualDetection det)
    {
        // Camino principal: asociacion por clase visual y color estimado.
        // Es el caso ideal: YOLO ve una clase compatible y el color coincide con un tracker.
        foreach (FusionTracker tracker in _activeTrackers)
        {
            if (!IsClassMatch(det, tracker)) continue;

            bool singleClass = tracker.shape.VisibleClass == tracker.shape.OccludedClass;
            bool colorMatch = singleClass || tracker.myColorIdentity == det.ColorCategory;

            if (colorMatch)
            {
                return tracker;
            }
        }

        return null;
    }

    private FusionTracker TryFindKinematicFallback(
        VisualDetection det, HashSet<FusionTracker> colorClaimedThisBatch, out bool usedColorFallback)
    {
        // Respaldo cinematico: si la mano oculta el color, el IMU puede revelar que objeto se mueve.
        // Se usa solo cerca de la mano, porque lejos de la mano la deteccion visual suele ser mas fiable.
        usedColorFallback = false;
        bool handIsTracked = _activeHand != null && _activeHand.IsTracked;
        float distToHand = handIsTracked ? Vector3.Distance(det.Position, _activeHand.transform.position) : 999f;
        bool handIsNearDet = handIsTracked && distToHand < handFPMaxDistance;

        if (handIsNearDet)
        {
            // Se cuentan candidatos en movimiento. Solo se acepta si hay exactamente uno.
            FusionTracker theOnlyMovingTracker = null;
            int movingCount = 0;
            string movingNames = "";

            foreach (FusionTracker tracker in _activeTrackers)
            {
                if (!IsClassMatch(det, tracker)) continue;
                if (colorClaimedThisBatch.Contains(tracker)) continue;

                if (tracker.contactAssessor.IsIMUMoving)
                {
                    movingCount++;
                    theOnlyMovingTracker = tracker;
                    movingNames += tracker.assignedName + " ";
                }
            }

            if (movingCount == 1)
            {
                usedColorFallback = true;
                Debug.Log($"[FSM-Kinematic] SUCCESS: YOLO did not see the color, but IMU motion revealed: {theOnlyMovingTracker.assignedName}");
                return theOnlyMovingTracker;
            }

            if (movingCount == 0)
            {
                Debug.LogWarning(
                    $"[FSM-Kinematic] FAILURE: Hand near cube (Color:{det.ColorCategory}), " +
                    "but no tracker reports motion (IsIMUMoving is false). Lower imuMotionThresholdDeg.");
            }
            else
            {
                Debug.LogWarning($"[FSM-Kinematic] FAILURE: Ambiguous motion. {movingCount} cubes are moving at once: {movingNames}. The table may have been bumped.");
            }

            return null;
        }

        if (!handIsTracked)
            Debug.LogWarning("[FSM-Kinematic] FAILURE: Meta hand tracking was lost on this frame.");
        else
            Debug.LogWarning($"[FSM-Kinematic] FAILURE: Detection is {distToHand:F2}m from the wrist. Limit is {handFPMaxDistance:F2}m. Increase handFPMaxDistance.");

        return null;
    }

    private FusionTracker TryFindSpatialFallback(
        VisualDetection det, HashSet<FusionTracker> colorClaimedThisBatch, out bool usedColorFallback)
    {
        // Ultimo recurso: si queda un solo candidato razonable sin reclamar, se usa.
        // Mantiene continuidad cuando color/clase fallan, pero evita elegir si hay demasiada ambiguedad.
        usedColorFallback = false;
        FusionTracker bestElim = null;
        int unclaimedCount = 0;
        float bestElimDist = float.MaxValue;

        foreach (FusionTracker tracker in _activeTrackers)
        {
            if (!IsClassMatch(det, tracker)) continue;
            if (colorClaimedThisBatch.Contains(tracker)) continue;

            // De los candidatos no reclamados, se guarda el mas cercano a la deteccion visual.
            unclaimedCount++;
            float dist = Vector3.Distance(det.Position, tracker.transform.position);
            if (dist < bestElimDist)
            {
                bestElimDist = dist;
                bestElim = tracker;
            }
        }

        if (unclaimedCount == 1 && bestElim != null)
        {
            usedColorFallback = true;
            return bestElim;
        }

        if (unclaimedCount > 1 && bestElim != null && bestElimDist < handFPMaxDistance * 2f)
        {
            usedColorFallback = true;
            return bestElim;
        }

        return null;
    }

    private void ApplyDetectionToTracker(VisualDetection det, FusionTracker targetTracker, bool usedColorFallback)
    {
        // Punto de entrega entre vision y tracker:
        // - FSM resuelve identidad y filtrado por mano.
        // - FusionTracker conserva memoria temporal y decide movimiento final en LateUpdate.
        TryCollectAutoTableCorrection(det, targetTracker);
        Vector3 detectionPosition = ResolveDetectionPositionForTracker(det, targetTracker);

        targetTracker.NotifyYoloDetection(detectionPosition);

        // Si contacto indica que es mano cercana y no objeto, ignora posicion para no arrastrar el cubo.
        if (enableHandFPFilter && !usedColorFallback && IsHandFalsePositive(targetTracker, detectionPosition))
            return;

        if (det.Class == targetTracker.shape.VisibleClass)
        {
            // Solo la clase visible aporta keypoints para yaw; la oculta solo actualiza posicion.
            Quaternion sensorRot = GetSensorRotation(targetTracker);

            // El visualizador debug vive fuera del tracker, pero usa la misma forma y objetivo visual
            // para que las flechas expliquen exactamente la calibracion que se esta intentando.
            targetTracker.KeypointDebug.Update(
                det.KeypointsWorld, det.KeypointWorldValid, sensorRot,
                targetTracker.shape, targetTracker.ShowKeypointDebug, targetTracker.VisualTargetPosition);

            if (sensorRot != Quaternion.identity)
            {
                targetTracker.ApplyKeypointCalibration(
                    det.KeypointsWorld, det.KeypointWorldValid,
                    sensorRot, det.IsGeometricValid);
            }
        }

        int kpLen = det.KeypointWorldValid != null ? det.KeypointWorldValid.Length : 0;
        float[] vis = new float[kpLen];
        for (int i = 0; i < kpLen; i++) vis[i] = det.KeypointWorldValid[i] ? 1f : 0f;

        // El logger guarda estado de calibracion para separar arranque, correccion continua y analisis bruto.
        // Aunque no haya keypoints utiles, se registra la caja para poder analizar fallos de vision despues.
        string calibStatus = targetTracker.HasInitialCalibration ? "ongoing" : "initial";

        BackgroundDataLogger logger = BackgroundDataLogger.Instance;
        if (logger != null)
        {
            logger.LogVisionData(det.ColorCategory.ToString(), det.BBox, det.Score, det.KeypointsPixel, vis, calibStatus);
            logger.LogVisionDataAll(det.ColorCategory.ToString(), det.BBox, det.Score, det.KeypointsPixel, vis, calibStatus);
        }

        targetTracker.UpdateVisualPosition(detectionPosition);
    }

    private static bool IsClassMatch(VisualDetection det, FusionTracker tracker)
    {
        // Centraliza la comprobacion repetida sin cambiar la regla de asociacion.
        return tracker != null &&
               tracker.shape != null &&
               (det.Class == tracker.shape.VisibleClass || det.Class == tracker.shape.OccludedClass);
    }

    private Vector3 ResolveDetectionPositionForTracker(VisualDetection det, FusionTracker tracker)
    {
        // Si el objeto esta bloqueado a mesa, proyecta la caja al plano calibrado en lugar de usar profundidad.
        if (tracker == null || !tracker.lockToTable || visionSource == null || tracker.IsGripping)
            return det.Position;

        float tablePlaneY = tracker.tableWorldHeightY + tracker.tableHeightCorrection + tracker.autoTableHeightCorrection;
        return visionSource.TryProjectToTablePlane(
            det.BBox.xMin, det.BBox.yMin, det.BBox.xMax, det.BBox.yMax,
            tablePlaneY, out Vector3 tableProjectedPosition)
            ? tableProjectedPosition
            : det.Position;
    }

    private void TryCollectAutoTableCorrection(VisualDetection det, FusionTracker tracker)
    {
        // Acumula muestras estables de vision para corregir pequenas diferencias entre mesa real y visual.
        // No corrige mientras hay contacto o movimiento: solo toma muestras cuando el objeto esta quieto.
        if (!autoCorrectTableHeightFromVision || _autoTableCorrectionApplied) return;
        if (tracker == null || !tracker.lockToTable || tracker.IsGripping) return;
        if (!tracker.IsStableForInitial || tracker.CurrentContactLevel >= ContactLevel.Touching) return;

        float expectedCenterY = tracker.GetTableLockedCenterYWithoutAutoCorrection();
        float sample = det.Position.y - expectedCenterY;
        if (Mathf.Abs(sample) > maxAutoTableCorrection) return;

        // Cada muestra dice cuanto difiere la vision de la altura de mesa esperada.
        _autoTableCorrectionSamples.Add(sample);
        int samplesNeeded = Mathf.Max(5, autoTableCorrectionSampleCount);
        if (_autoTableCorrectionSamples.Count < samplesNeeded) return;

        // Se aplica una sola vez por sesion para no perseguir ruido durante el juego.
        float correction = Median(_autoTableCorrectionSamples);
        ApplyAutoTableCorrection(correction);
        _autoTableCorrectionApplied = true;
        _autoTableCorrectionValue = correction;

        if (showDebugLogs)
            Debug.Log($"[FSM] Auto table height correction applied: {correction:F3}m from {_autoTableCorrectionSamples.Count} stable samples.");
    }

    private void ApplyAutoTableCorrection(float correction)
    {
        // Aplica la correccion al juego si existe; si no, directamente a los trackers activos.
        if (MemoryGame.Instance != null)
        {
            MemoryGame.Instance.SetAutoTableHeightCorrection(correction);
            return;
        }

        foreach (var tracker in _activeTrackers)
            if (tracker != null) tracker.autoTableHeightCorrection = correction;
    }

    private void ResetAutoTableCorrection()
    {
        // Reinicia muestras y elimina correcciones anteriores al empezar una partida nueva.
        _autoTableCorrectionSamples.Clear();
        _autoTableCorrectionApplied = false;
        _autoTableCorrectionValue = 0f;
        foreach (var tracker in _activeTrackers)
            if (tracker != null) tracker.autoTableHeightCorrection = 0f;
        if (MemoryGame.Instance != null) MemoryGame.Instance.SetAutoTableHeightCorrection(0f);
    }

    private static float Median(List<float> values)
    {
        // Usa mediana para que una muestra mala no domine la correccion de mesa.
        if (values == null || values.Count == 0) return 0f;
        float[] sorted = values.ToArray();
        System.Array.Sort(sorted);
        int mid = sorted.Length / 2;
        return sorted.Length % 2 == 0
            ? (sorted[mid - 1] + sorted[mid]) * 0.5f
            : sorted[mid];
    }

    private static Quaternion GetSensorRotation(FusionTracker tracker)
    {
        // Combina orientacion BLE con la rotacion de montaje configurada en el tracker.
        if (BLEManager.Instance == null) return Quaternion.identity;
        DeviceData deviceData = BLEManager.Instance.GetDeviceByName(tracker.assignedName);
        if (deviceData == null || deviceData.orientation.w == 0f) return Quaternion.identity;
        return Quaternion.Euler(tracker.mountingRotation) * deviceData.orientation.normalized;
    }

    private bool IsHandFalsePositive(FusionTracker tracker, Vector3 detectedPos)
    {
        // Durante agarre real no se filtra; fuera de agarre se respeta la evaluacion de contacto.
        if (tracker.IsGripping) return false;
        return tracker.contactAssessor.ShouldFilterAsHandFP;
    }

}
