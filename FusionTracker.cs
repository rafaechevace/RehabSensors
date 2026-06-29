using UnityEngine;
using System;

public class FusionTracker : MonoBehaviour
{
    // Tracker multimodal de un objeto fisico. Combina IMU/FSR por BLE,
    // posicion visual, mano activa y reglas de forma.
    // Entrada principal: BLEManager aporta rotacion/FSR, ObjectDetectionVisualizerV2 aporta posiciones,
    // y LateUpdate decide la pose final del objeto visible.
    //
    // Relacion con otros scripts:
    // - FusionSystemManager asocia detecciones YOLO a este tracker y llama a NotifyYoloDetection,
    //   ApplyKeypointCalibration y UpdateVisualPosition.
    // - CubeContactAssessor resume FSR/IMU/mano en niveles de contacto.
    // - ShapePolicy define geometria: keypoints, apoyo sobre mesa y snapping de rotacion.
    // - MemoryGame puede fijar tableWorldHeightY/lockToTable para alinear juego y fusion.
    //
    // Nota para quien empieza con Unity/C#:
    // - Los campos public o [SerializeField] aparecen en el Inspector y se configuran en prefabs/escenas.
    // - Los campos private con guion bajo son memoria interna de esta instancia durante la partida.
    // - Vector3 representa posicion/direccion 3D; Quaternion representa rotacion.
    // --- Configuracion visible en Inspector ---
    // Identidad logica del objeto: el manager usa estos datos para asociar BLE, color visual y prefab.
    [Header("Identity")]
    public string assignedName;
    public DetectedColor myColorIdentity;

    // Politica de forma: encapsula lo que cambia entre cubo, taza u otro objeto fisico.
    // Asi FusionTracker no necesita conocer detalles geometricos especificos de cada pieza.
    [Header("Shape")]
    public ShapePolicy shape;

    // Referencias de escena que el tracker necesita para mover el objeto visible y leer mano/cabeza.
    [Header("References")]
    public Transform visualObject;
    public OVRHand trackingHand;
    public Transform playerHead;

    // Bloqueo a mesa: fuerza la altura Y cuando el objeto esta apoyado para evitar temblores de profundidad.
    // La altura puede venir de MRUK, MemoryGame o una correccion manual/automatica.
    [Header("Table Lock")]
    public bool lockToTable = false;
    public float tableWorldHeightY = 0.0f;
    [Tooltip("Correccion manual de errores de mesa en metros. Positivo sube el objeto, negativo lo baja.")]
    public float tableHeightCorrection = 0.0f;
    [Tooltip("Correccion automatica estimada desde muestras visuales estables al inicio de la sesion.")]
    public float autoTableHeightCorrection = 0.0f;
    [Tooltip("Desviacion visual maxima en Y que aun se trata como contacto con mesa.")]
    public float snapThreshold = 0.06f;

    // Suavizado de posicion y pequenos umbrales para ignorar ruido visual o rotacional.
    [Header("Movement Control")]
    public float visualCorrectionSpeed = 12.0f;
    public float angularDeadzoneDeg = 1.5f;
    public float positionDeadzoneMeters = 0.005f;

    // Parametros de rotacion/calibracion: deciden cuando confiar en keypoints y como mezclar sensor + vision.
    [Header("Rotation & Stability")]
    public float rotationSmoothSpeed = 30.0f;
    public float calibrationLerpSpeed = 8.0f;
    public float stabilityAngleThreshold = 3.0f;
    public float keypointCalibrationDeadzoneDeg = 0f;
    public float instantCorrectionThresholdDeg = 15.0f;
    public float recalibCooldownSeconds = 2.0f;
    public bool requireHighConfidenceForRecalib = true;

    // Contadores de estabilidad: cuantos frames seguidos debe repetirse una condicion antes de aceptarla.
    [Header("Stability")]
    public int stabilityRequiredFrames = 8;
    public int initialStabilityRequiredFrames = 3;

    // Estabilidad de cabeza: si el usuario mueve mucho la cabeza, la proyeccion visual puede ser menos fiable.
    [Header("Head Stability")]
    public int headStableRequiredFrames = 3;
    public float headStableThresholdDeg = 1.5f;

    // Filtro temporal de offsets: evita recalibrar con una sola deteccion aislada o erronea.
    [Header("Temporal Filter")]
    public int medianFilterSize = 3;
    public float maxBufferSpreadDeg = 20.0f;

    // Limite amplio para la primera alineacion sensor-vision.
    [Header("Initial Calibration")]
    public float initialCalibrationMaxOffset = 180f;

    // Orientacion fisica del sensor respecto al objeto. Compensa como esta montada la electronica.
    [Header("Physical Mounting")]
    public Vector3 mountingRotation = Vector3.zero;

    // Visuales auxiliares de keypoints. Solo ayudan a depurar calibracion; no afectan al gameplay.
    [Header("Debug Keypoints")]
    [SerializeField] private bool showKeypointDebug = true;
    [SerializeField] private float keypointSphereRadius = 0.004f;

    // Evento usado por el logger ligero de calibracion.
    public event Action<float, double> OnCalibrationErrorMeasured;

    // Umbrales FSR: entrada y salida separados para que el agarre no parpadee cerca del limite.
    [Header("Grip Detection")]
    public int gripThresholdEnter = 20;
    public int gripThresholdExit = 10;
    public float grabCooldown = 0.5f;

    // Evaluador de contacto: resume senales de FSR, IMU y vision en una lectura simple de apoyo/manipulacion.
    [Header("Contact Assessment")]
    public CubeContactAssessor contactAssessor = new CubeContactAssessor();

    // Magnitud minima de movimiento lineal IMU para considerar que el objeto se esta desplazando.
    [Header("IMU Linear Motion")]
    public float imuLinearMotionThreshold = 0.18f;

    // Puente IMU-mano durante oclusion: mantiene el objeto pegado a la mano si la vision se pierde,
    // pero solo cuando IMU, proximidad y contacto hacen probable que el paciente lo este manipulando.
    [Header("IMU Occlusion Hand Bridge")]
    public bool enableImuOcclusionHandBridge = true;
    public float occlusionBridgeEntryRadius = 0.045f;
    public float occlusionBridgeContactSlack = 0.02f;
    public float occlusionBridgeMaxSeconds = 3.0f;
    public float occlusionBridgeMaxActiveSeconds = 6.0f;
    public float occlusionBridgeTableCoverRadius = 0.14f;
    public float occlusionBridgeCoverMemorySeconds = 0.45f;
    public float occlusionBridgeEntryLinearMotionThreshold = 0.6f;
    public float occlusionBridgeReleaseLinearMotionThreshold = 0.1f;
    public int occlusionBridgeReleaseRestFrames = 2;
    public float occlusionBridgeReentryCooldownSeconds = 0.35f;
    public float occlusionBridgeCandidateSwitchMargin = 0.03f;
    public int occlusionBridgeEntryFrames = 2;
    public int occlusionBridgeExitFrames = 4;

    // Logger diagnostico separado para inspeccionar por que el puente IMU-mano entra o se bloquea.
    [Header("Debug IMU Motion")]
    [SerializeField] private FusionImuMotionDebugLogger imuMotionDebug = new FusionImuMotionDebugLogger();

    // --- Estado interno: agarre, oclusion y evidencia BLE reciente ---
    // No se configura en Inspector: Unity lo recalcula durante la partida.
    // FSR representa agarre confirmado; el puente IMU cubre oclusiones cuando hay movimiento
    // de sensor pero la vision deja de ver bien el objeto.
    private bool _isGripping;
    private bool _isOcclusionBridgeActive;
    private int _occlusionBridgeEntryCount;
    private int _occlusionBridgeExitCount;
    private int _occlusionBridgeRestExitCount;
    private Vector3 _occlusionBridgeLocalPosition;
    private float _occlusionBridgeStartTime;
    private float _lastOcclusionBridgeCoverTime = -10f;
    private float _lastOcclusionBridgeStopTime = -10f;
    private bool _hasAcceptedVisualPosition;
    private int _lastImuPayloadLength;
    private bool _lastImuHasLinearMotion;
    private float _lastReadImuLinearMotion;

    // Lectura publica corta para otros scripts: ultimo movimiento lineal decodificado desde IMU.
    public float LastLinearMotion => _lastReadImuLinearMotion;

    // --- Estado interno: ultimo frame visual y relacion con fisicas/mano ---
    // Guarda la ultima posicion YOLO, si hubo deteccion este frame y datos necesarios para seguir la mano.
    private Vector3 _lastYoloPosition;
    private bool _hasYoloThisFrame;
    private float _lastReleaseTime;
    private Rigidbody _rb;
    private Vector3 _gripLocalPosition;

    // --- Estado interno: calibracion sensor-vision ---
    // La calibracion no cambia el dato BLE bruto; guarda un offset de yaw que alinea sensor y keypoints.
    private Quaternion _calibrationOffset = Quaternion.identity;
    private Quaternion _targetCalibrationOffset = Quaternion.identity;

    // --- Estado interno: pose objetivo y estabilidad ---
    // _visualTargetPosition es el ancla de posicion que llega de YOLO/profundidad y se suaviza al aplicar.
    // _smoothedRotation es la rotacion final filtrada que vera el usuario en escena.
    private Vector3 _visualTargetPosition;
    private Quaternion _lastValidSensorRotation;
    private Quaternion _smoothedRotation;
    private int _stableFramesCount;
    private bool _forceNextVisualUpdate = false;
    private bool _hasInitialCalibration = false;

    // Buffer circular pequeno para aceptar recalibraciones solo si varias mediciones visuales coinciden.
    // Si los offsets se dispersan demasiado, se descarta para evitar saltos de orientacion.
    private float[] _offsetBuffer;
    private int _offsetWriteIdx;
    private int _offsetCount;
    private bool _wasStable;
    private float _lastRecalibTime;
    private bool _pendingPostReleaseCalib;

    // Estabilidad de cabeza calculada frame a frame. Protege calibraciones mientras cambia la vista del usuario.
    private Quaternion _lastHeadRotation;
    private int _headStableFrames;

    // Cache de orientacion y visuales auxiliares para no recrear objetos ni recalcular ejes innecesariamente.
    private Vector3 _cachedDownAxis = Vector3.down;
    private FusionKeypointDebugVisualizer _keypointDebug;

    // Ultima pose enviada a logs de experimento. Permite derivar velocidades y evitar filas ambiguas.
    private Vector3 _lastLoggedPosition;
    private Quaternion _lastLoggedRotation = Quaternion.identity;
    private bool _hasLastLoggedPose;

    // Lista estatica = compartida por todos los FusionTracker activos.
    // Se usa solo para decidir que objeto tiene permiso de seguir la mano en una oclusion.
    private static readonly System.Collections.Generic.List<FusionTracker> ActiveTrackers = new();
    private static FusionTracker _occlusionBridgeOwner;

    // --- Estado publico de solo lectura ---
    // Propiedades con "=>" son accesos cortos: calculan o devuelven un valor sin permitir modificarlo desde fuera.
    public bool IsStable => _stableFramesCount >= stabilityRequiredFrames;
    public bool IsStableForInitial => _stableFramesCount >= initialStabilityRequiredFrames;
    public bool IsHeadStable => _headStableFrames >= headStableRequiredFrames;
    public bool HasInitialCalibration => _hasInitialCalibration;
    public float LastVisualUpdateTime { get; private set; }
    public bool IsGripping => _isGripping;
    public ContactLevel CurrentContactLevel => contactAssessor.Level;

    // Datos internos expuestos solo a helpers de depuracion. La logica principal no debe depender de ellos.
    internal bool HasAcceptedVisualPositionForDebug => _hasAcceptedVisualPosition;
    internal bool IsOcclusionBridgeActiveForDebug => _isOcclusionBridgeActive;
    internal int LastImuPayloadLengthForDebug => _lastImuPayloadLength;
    internal bool LastImuHasLinearMotionForDebug => _lastImuHasLinearMotion;
    internal float LastOcclusionBridgeStopTimeForDebug => _lastOcclusionBridgeStopTime;
    internal int OcclusionBridgeEntryCountForDebug => _occlusionBridgeEntryCount;
    internal static FusionTracker OcclusionBridgeOwnerForDebug => _occlusionBridgeOwner;
    internal bool ShowKeypointDebug => showKeypointDebug;
    internal Vector3 VisualTargetPosition => _visualTargetPosition;
    internal FusionKeypointDebugVisualizer KeypointDebug
    {
        get
        {
            if (_keypointDebug == null) InitDebugVisuals();
            return _keypointDebug;
        }
    }

    public void NotifyYoloDetection(Vector3 yoloWorldPos)
    {
        // Guarda la ultima posicion YOLO para clasificar contacto y oclusion en LateUpdate.
        // No mueve el objeto: solo actualiza evidencia visual para el evaluador de contacto.
        _lastYoloPosition = yoloWorldPos;
        _hasYoloThisFrame = true;
    }

    private void Awake()
    {
        // Awake lo llama Unity una vez al crear el componente, antes del primer frame.
        // Aqui se preparan referencias, escala real y buffers antes de recibir datos BLE o vision.
        if (visualObject == null) visualObject = transform;
        _rb = GetComponent<Rigidbody>();
        _visualTargetPosition = transform.position;
        _lastValidSensorRotation = _smoothedRotation = transform.rotation;

        if (playerHead == null && Camera.main != null) playerHead = Camera.main.transform;
        _lastHeadRotation = playerHead != null ? playerHead.rotation : Quaternion.identity;

        if (shape != null) shape.ApplyRealDimensionScaling(visualObject);

        _forceNextVisualUpdate = true;
        _offsetBuffer = new float[Mathf.Max(3, medianFilterSize)];
        InitDebugVisuals();
    }

    private void OnEnable()
    {
        // Registro global usado para repartir propiedad del puente de oclusion entre trackers.
        if (!ActiveTrackers.Contains(this))
            ActiveTrackers.Add(this);
    }

    private void OnDisable()
    {
        // Libera registro y propiedad si este tracker se desactiva.
        ActiveTrackers.Remove(this);
        if (_occlusionBridgeOwner == this)
            _occlusionBridgeOwner = null;
    }

    // Destruye visuales de depuracion creados en runtime.
    private void OnDestroy() { DestroyDebugVisuals(); }

    private void InitDebugVisuals()
    {
        // Crea esferas de keypoints y flechas de direccion para inspeccionar calibracion.
        if (_keypointDebug == null) _keypointDebug = new FusionKeypointDebugVisualizer();
        _keypointDebug.Initialize(shape, myColorIdentity, keypointSphereRadius);
    }

    private void DestroyDebugVisuals()
    {
        // Limpia objetos auxiliares para no dejarlos vivos al cambiar de escena.
        _keypointDebug?.DestroyVisuals();
        _keypointDebug = null;
    }

    public void ApplyKeypointCalibration(Vector3[] kptsWorld, bool[] kptsValid, Quaternion sensorRotation, bool isGeometricValid)
    {
        // Corrige yaw del sensor cuando la vision ofrece keypoints estables y geometricamente fiables.
        // Orden de rechazo intencional: primero evita calibrar durante agarre, luego exige forma,
        // estabilidad del objeto y estabilidad de cabeza antes de tocar offsets.
        // Este metodo solo modifica offsets de rotacion; la aplicacion real ocurre despues en LateUpdate.
        // Estas primeras salidas son filtros de seguridad: si una condicion no se cumple, no se calibra.
        if (_isGripping) return;
        if (shape == null || !shape.CanCalibrateYaw(kptsValid)) return;
        if (!_hasInitialCalibration) { if (!IsStableForInitial) return; } else { if (!IsStable) return; }
        if (!IsHeadStable) return;

        // Compara la direccion frontal visual con la direccion frontal del sensor.
        // Si ambas no apuntan igual, la diferencia es el "offset" que corrige el yaw del IMU.
        Vector3 faceDir = shape.ComputeForwardXZ(kptsWorld, kptsValid, _visualTargetPosition);
        if (faceDir == Vector3.zero) return;

        Vector3 sensorFwd = sensorRotation * Vector3.forward; sensorFwd.y = 0;
        if (sensorFwd.sqrMagnitude < 0.0001f) return;
        sensorFwd.Normalize();

        // Atan2 convierte una direccion XZ en angulo horizontal. DeltaAngle evita saltos 359 -> 0 grados.
        float visualYaw = Mathf.Atan2(faceDir.x, faceDir.z) * Mathf.Rad2Deg;
        float sensorYaw = Mathf.Atan2(sensorFwd.x, sensorFwd.z) * Mathf.Rad2Deg;
        float rawOffset = Mathf.DeltaAngle(sensorYaw, visualYaw);

        if (!_hasInitialCalibration)
        {
            // La primera calibracion acepta un offset amplio, pero exige estabilidad.
            if (Mathf.Abs(rawOffset) > initialCalibrationMaxOffset) return;
            _targetCalibrationOffset = Quaternion.Euler(0, rawOffset, 0);
            _calibrationOffset = _targetCalibrationOffset;
            _hasInitialCalibration = true; _pendingPostReleaseCalib = false;
            LogCalibrationEvent("initial_keypoint_yaw", rawOffset);
            ClearOffsetBuffer();
            return;
        }

        if (_pendingPostReleaseCalib)
        {
            // Tras soltar el objeto, permite una correccion fuerte si la deteccion es de alta confianza.
            // Esto recupera el alineamiento despues de manipular el objeto con la mano.
            if (shape.IsHighConfidence(kptsValid, isGeometricValid) && Mathf.Abs(rawOffset) < 90f)
            {
                _targetCalibrationOffset = Quaternion.Euler(0, rawOffset, 0);
                _calibrationOffset = _targetCalibrationOffset;
                _pendingPostReleaseCalib = false; _lastRecalibTime = Time.time;
                LogCalibrationEvent("post_release_keypoint_yaw", rawOffset);
                ClearOffsetBuffer();
                return;
            }
            if (Time.time - _lastReleaseTime > 5.0f) _pendingPostReleaseCalib = false;
            return;
        }

        if (Time.time - _lastRecalibTime < recalibCooldownSeconds) return;
        if (requireHighConfidenceForRecalib && !shape.IsHighConfidence(kptsValid, isGeometricValid)) return;

        // En recalibracion continua se filtra por mediana para evitar saltos por keypoints ruidosos.
        _offsetBuffer[_offsetWriteIdx] = rawOffset;
        _offsetWriteIdx = (_offsetWriteIdx + 1) % _offsetBuffer.Length;
        _offsetCount = Mathf.Min(_offsetCount + 1, _offsetBuffer.Length);

        // No se usa una sola medicion: se espera a tener varias para comparar consistencia.
        int needed = Mathf.Max(3, medianFilterSize);
        if (_offsetCount < needed) return;

        // Si las mediciones del buffer estan muy dispersas, probablemente hay ruido visual.
        float minV = float.MaxValue, maxV = float.MinValue;
        for (int i = 0; i < _offsetCount; i++)
        {
            if (_offsetBuffer[i] < minV) minV = _offsetBuffer[i];
            if (_offsetBuffer[i] > maxV) maxV = _offsetBuffer[i];
        }
        if (maxV - minV > maxBufferSpreadDeg) { ClearOffsetBuffer(); return; }

        // La mediana representa el offset "central" y evita que un valor raro domine la correccion.
        float median = ComputeMedian(); float absM = Mathf.Abs(median);
        if (absM < keypointCalibrationDeadzoneDeg) { ClearOffsetBuffer(); return; }

        _targetCalibrationOffset = Quaternion.Euler(0, median, 0);
        _lastRecalibTime = Time.time;
        if (absM >= instantCorrectionThresholdDeg) _calibrationOffset = _targetCalibrationOffset;
        LogCalibrationEvent("continuous_keypoint_yaw", median);
        ClearOffsetBuffer();
    }

    private void LogCalibrationEvent(string calibrationType, float rawOffset)
    {
        BackgroundDataLogger logger = BackgroundDataLogger.Instance;
        if (logger != null) logger.LogCalibrationEvent(assignedName, calibrationType, rawOffset);
    }

    // Reinicia el filtro temporal de offsets de yaw.
    private void ClearOffsetBuffer() { _offsetCount = 0; _offsetWriteIdx = 0; }

    private float ComputeMedian()
    {
        // Mediana simple sobre el buffer actual para rechazar outliers visuales.
        float[] sortedOffsets = new float[_offsetCount];
        for (int i = 0; i < _offsetCount; i++) sortedOffsets[i] = _offsetBuffer[i];
        Array.Sort(sortedOffsets); int middle = _offsetCount / 2;
        return (_offsetCount % 2 == 0) ? (sortedOffsets[middle - 1] + sortedOffsets[middle]) * 0.5f : sortedOffsets[middle];
    }

    public void UpdateVisualPosition(Vector3 targetPos)
    {
        // Acepta una nueva posicion visual y decide si debe mover el objetivo interno.
        // Importante: aqui no se escribe transform.position. Solo se actualiza _visualTargetPosition,
        // que ApplyTransform consumira al final del frame.
        float timeSinceLastUpdate = Time.time - LastVisualUpdateTime;
        if (timeSinceLastUpdate > 0.5f) _forceNextVisualUpdate = true;

        // LastVisualUpdateTime mide frescura de vision; _hasAcceptedVisualPosition habilita puentes/diagnostico.
        LastVisualUpdateTime = Time.time;
        _hasAcceptedVisualPosition = true;
        if (_isGripping) return;

        // Si la cabeza se mueve y el objeto esta estable, se ignora microvision para no perseguir ruido.
        bool contactBypass = contactAssessor.Level >= ContactLevel.Touching;
        if (!_forceNextVisualUpdate && !contactBypass && IsStable && !IsHeadStable) return;

        Vector3 finalPos = targetPos;

        if (lockToTable)
        {
            // Mientras no haya levantamiento confirmado, fuerza el centro a la altura de mesa.
            float tableLockedY = GetTableLockedCenterY();
            bool confirmedLift = contactAssessor.Level >= ContactLevel.HeldSoft && (targetPos.y > tableLockedY + snapThreshold);
            if (!confirmedLift)
            {
                finalPos.y = tableLockedY;
            }
        }

        bool handNear = trackingHand != null && Vector3.Distance(trackingHand.transform.position, transform.position) < 0.12f;
        bool bypassDeadzone = _forceNextVisualUpdate || handNear;

        if (!bypassDeadzone)
        {
            // Deadzones evitan microcorrecciones por ruido cuando el objeto y la cabeza estan estables.
            // Hay una deadzone en metros y otra angular vista desde la cabeza del usuario.
            if (Vector3.Distance(finalPos, _visualTargetPosition) < positionDeadzoneMeters) return;
            if (playerHead != null && Vector3.Angle(finalPos - playerHead.position, _visualTargetPosition - playerHead.position) < angularDeadzoneDeg) return;
        }

        _visualTargetPosition = finalPos;
        _forceNextVisualUpdate = false;
    }

    private float GetTableLockedCenterY()
    {
        // Altura final de centro incluyendo correccion automatica.
        return GetTableLockedCenterYWithoutAutoCorrection() + autoTableHeightCorrection;
    }

    public float GetTableLockedCenterYWithoutAutoCorrection()
    {
        // Altura de centro basada en mesa, offset manual y geometria de la forma.
        float centerYOffset = shape != null ? shape.GetTableCenterYOffset(visualObject, _cachedDownAxis) : 0f;
        return tableWorldHeightY + centerYOffset + tableHeightCorrection;
    }

    private void LateUpdate()
    {
        // Bucle principal de fusion, ejecutado tras Update para recibir ya las detecciones del frame.
        // Flujo:
        // 1) Suaviza offset de calibracion.
        // 2) Lee BLE y calcula estabilidad del objeto/cabeza.
        // 3) Actualiza contacto y puente de oclusion.
        // 4) Resuelve agarre y aplica transform final.
        // 5) Registra CSV.
        if (Quaternion.Angle(_calibrationOffset, _targetCalibrationOffset) > 0.1f)
            _calibrationOffset = Quaternion.Slerp(_calibrationOffset, _targetCalibrationOffset, Time.deltaTime * calibrationLerpSpeed);
        else _calibrationOffset = _targetCalibrationOffset;

        // Primero se lee BLE y se convierte en rotacion ya compensada.
        ReadSensorData(out Quaternion rawRot, out bool grip, out bool release, out int rawFsrValue, out float imuLinearMotion);

        // La estabilidad combina movimiento angular y estado de la cabeza para aceptar calibraciones.
        float imuDeltaDeg = Quaternion.Angle(rawRot, _lastValidSensorRotation);
        float contactMotionDeltaDeg = GetContactMotionDeltaDeg(imuDeltaDeg, imuLinearMotion);
        if (imuDeltaDeg < stabilityAngleThreshold) _stableFramesCount++; else _stableFramesCount = 0;

        // Si el objeto deja de estar estable, se descartan offsets visuales acumulados previamente.
        bool nowStable = _hasInitialCalibration ? IsStable : IsStableForInitial;
        if (!nowStable && _wasStable) ClearOffsetBuffer();
        _wasStable = nowStable;
        _lastValidSensorRotation = rawRot;

        // La cabeza estable importa porque la proyeccion visual depende de la camara del visor.
        if (playerHead != null)
        {
            if (Quaternion.Angle(playerHead.rotation, _lastHeadRotation) < headStableThresholdDeg) _headStableFrames++;
            else _headStableFrames = 0;
            _lastHeadRotation = playerHead.rotation;
        }
        else _headStableFrames = headStableRequiredFrames;

        contactAssessor.fsrHardThreshold = gripThresholdEnter;
        contactAssessor.fsrReleaseThreshold = gripThresholdExit;

        // Se prepara la posicion de mano aunque no haya tracking; el evaluador recibe handTracked=false.
        Vector3 handPos = trackingHand != null ? trackingHand.transform.position : Vector3.zero;
        bool handTracked = trackingHand != null && trackingHand.IsTracked;

        // El evaluador de contacto decide si la mano esta cerca, tocando o agarrando el objeto.
        contactAssessor.UpdateSignals(
            fsrValue: rawFsrValue, imuAngularDeltaDeg: contactMotionDeltaDeg, handPosition: handPos,
            handTracked: handTracked, cubeTrackedPosition: transform.position,
            yoloDetectionPosition: _lastYoloPosition, hasYoloThisFrame: _hasYoloThisFrame, currentTime: Time.time);

        // Con contacto actualizado, se resuelve puente de oclusion, agarre y transform final.
        UpdateOcclusionBridge(grip, imuLinearMotion);
        imuMotionDebug?.Log(this, imuLinearMotion, contactMotionDeltaDeg, rawFsrValue, grip);

        // La marca de YOLO dura solo un frame: si no llega nueva deteccion, el siguiente LateUpdate lo sabra.
        _hasYoloThisFrame = false;

        UpdateGripState(grip, release, rawRot);
        ApplyTransform(_calibrationOffset * rawRot);
        LogSessionCsvData(rawRot, rawFsrValue, grip, imuDeltaDeg);
    }

    private void LogSessionCsvData(Quaternion rawRot, int rawFsrValue, bool fsrGrip, float imuDeltaDeg)
    {
        // Vuelca sensores, pose fusionada y estado de cubo en los CSV de fondo.
        BackgroundDataLogger logger = BackgroundDataLogger.Instance;
        if (logger == null) return;

        BLEManager ble = BLEManager.Instance;
        DeviceData deviceData = ble != null ? ble.GetDeviceByName(assignedName) : null;
        if (deviceData != null && (deviceData.HasImuData || deviceData.HasGripData))
        {
            logger.LogSensorData(assignedName, rawRot, rawFsrValue);
        }

        Quaternion currentRot = visualObject != null ? visualObject.rotation : transform.rotation;
        Vector3 currentPos = transform.position;
        Vector3 velocity = Vector3.zero;
        float speedMps = 0f;
        float angularVelocityDegS = 0f;

        // La primera muestra no tiene "anterior"; desde la segunda se calculan velocidades por diferencia.
        if (_hasLastLoggedPose && Time.deltaTime > 0f)
        {
            velocity = (currentPos - _lastLoggedPosition) / Time.deltaTime;
            speedMps = velocity.magnitude;
            angularVelocityDegS = Quaternion.Angle(_lastLoggedRotation, currentRot) / Time.deltaTime;
        }

        _lastLoggedPosition = currentPos;
        _lastLoggedRotation = currentRot;
        _hasLastLoggedPose = true;

        logger.LogFusionData(assignedName, currentPos, currentRot, _isGripping, _cachedDownAxis);

        // gripSource explica de donde viene el agarre registrado: FSR real, puente IMU o nada.
        string gripSource = _isGripping ? "fsr" : (_isOcclusionBridgeActive ? "imu_bridge" : "none");
        logger.LogCubeStateData(
            assignedName, currentPos, currentRot, velocity, speedMps, angularVelocityDegS, _isGripping,
            gripSource, contactAssessor.Level.ToString(), rawFsrValue, imuDeltaDeg, _cachedDownAxis);
    }

    private float GetContactMotionDeltaDeg(float angularDeltaDeg, float imuLinearMotion)
    {
        // Convierte aceleracion lineal IMU en evidencia angular equivalente para el evaluador de contacto.
        if (imuLinearMotionThreshold <= 0f || imuLinearMotion < imuLinearMotionThreshold)
            return angularDeltaDeg;

        float assessorThreshold = Mathf.Max(0.0001f, contactAssessor.imuMotionThresholdDeg);
        float linearMotionAsDelta = assessorThreshold * (imuLinearMotion / imuLinearMotionThreshold);
        return Mathf.Max(angularDeltaDeg, linearMotionAsDelta);
    }

    private void ReadSensorData(out Quaternion rawRot, out bool grip, out bool release, out int rawFsrValue, out float imuLinearMotion)
    {
        // Lee el paquete BLE asociado al tracker y deja valores seguros si el dispositivo falta.
        // out significa que el metodo devuelve varios valores a la vez: rotacion, agarre, liberacion, etc.
        rawRot = _lastValidSensorRotation; grip = release = false; rawFsrValue = 0; imuLinearMotion = 0f;
        BLEManager ble = BLEManager.Instance;
        DeviceData deviceData = ble != null ? ble.GetDeviceByName(assignedName) : null;
        if (deviceData == null) return;
        rawFsrValue = deviceData.grip;
        imuLinearMotion = deviceData.imuLinearMotion;
        _lastReadImuLinearMotion = imuLinearMotion;
        _lastImuPayloadLength = deviceData.imuPayloadLength;
        _lastImuHasLinearMotion = deviceData.imuPayloadLength >= 20;
        grip = deviceData.grip > gripThresholdEnter; release = deviceData.grip < gripThresholdExit;
        if (deviceData.orientation.w != 0f) rawRot = Quaternion.Euler(mountingRotation) * deviceData.orientation.normalized;
    }

    private void UpdateGripState(bool fsrGrip, bool fsrRelease, Quaternion currentRaw)
    {
        // Cambia entre agarre y liberacion usando umbrales FSR con cooldown.
        // Cuando entra en agarre, la posicion pasa a estar anclada a la mano hasta que el FSR baja.
        if (!_isGripping)
        {
            if (fsrGrip && Time.time > _lastReleaseTime + grabCooldown && trackingHand != null)
            {
                StopOcclusionBridge();
                EnterGrip(currentRaw);
            }
        }
        else
        {
            if (fsrRelease) ExitGrip(currentRaw);
        }
    }

    private void UpdateOcclusionBridge(bool fsrGrip, float imuLinearMotion)
    {
        // Mantiene una posicion guiada por la mano cuando la vision se pierde por oclusion.
        // Es deliberadamente conservador: solo entra con mano cercana, movimiento IMU y sin otro tracker mejor.
        // Estado inactivo: acumula evidencia de entrada. Estado activo: sigue un punto local en la mano
        // hasta que hay reposo, la mano se aleja o se agota el tiempo maximo.
        if (!enableImuOcclusionHandBridge || _isGripping || fsrGrip || trackingHand == null || !trackingHand.IsTracked)
        {
            StopOcclusionBridge();
            return;
        }

        float visionAge = Time.time - LastVisualUpdateTime;
        bool bridgeEntryMotion = HasBridgeEntryMotion(imuLinearMotion);
        bool bridgeReleaseRest = IsBridgeReleaseRest(imuLinearMotion);
        Vector3 handPos = trackingHand.transform.position;
        float handToObject = Vector3.Distance(handPos, transform.position);
        float handToVisualTarget = Vector3.Distance(handPos, _visualTargetPosition);

        // Se guarda memoria de que la mano cubrio la zona para tolerar perdidas visuales breves.
        UpdateOcclusionBridgeCoverMemory(handPos, handToObject, handToVisualTarget);
        bool coverRecent = HasRecentOcclusionBridgeCover();
        bool visionTooOld = !_hasAcceptedVisualPosition || (visionAge > occlusionBridgeMaxSeconds && !coverRecent);

        if (_isOcclusionBridgeActive)
        {
            // Si el puente esta activo, se conserva mientras la mano siga vinculada y no expire.
            // Esta rama solo decide si seguir, mezclar con vision recuperada o salir del puente.
            float maxActiveSeconds = Mathf.Max(occlusionBridgeMaxSeconds, occlusionBridgeMaxActiveSeconds);
            bool bridgeExpired = Time.time - _occlusionBridgeStartTime > maxActiveSeconds;

            // Para cerrar por reposo se exigen varios frames, evitando parpadeos por ruido de la IMU.
            bool noCurrentHandLinkAtRest = !IsHandCurrentlyLinkedForOcclusionBridge(handPos, handToObject, handToVisualTarget) && bridgeReleaseRest;

            if (bridgeReleaseRest || noCurrentHandLinkAtRest)
                _occlusionBridgeRestExitCount++;
            else
                _occlusionBridgeRestExitCount = 0;

            if (_occlusionBridgeRestExitCount >= occlusionBridgeReleaseRestFrames)
            {
                StopOcclusionBridge();
                return;
            }

            bool handLeft = !IsHandCurrentlyLinkedForOcclusionBridge(handPos, handToObject, handToVisualTarget);
            bool shouldExit = handLeft || bridgeExpired;
            
            if (shouldExit)
            {
                _occlusionBridgeExitCount++;
                if (_occlusionBridgeExitCount >= occlusionBridgeExitFrames)
                    StopOcclusionBridge();
            }
            else
            {
                _occlusionBridgeExitCount = 0;
                if (_hasYoloThisFrame)
                {
                    // Cuando vuelve vision, mezcla suavemente el ancla de mano hacia el objetivo visual.
                    Vector3 currentTarget = trackingHand.transform.TransformPoint(_occlusionBridgeLocalPosition);
                    Vector3 blendedTarget = Vector3.Lerp(currentTarget, _visualTargetPosition, 0.35f);
                    _occlusionBridgeLocalPosition = trackingHand.transform.InverseTransformPoint(blendedTarget);
                }
            }
            return;
        }

        // A partir de aqui el puente esta apagado. Estas condiciones bloquean la entrada.
        if (visionTooOld || !bridgeEntryMotion ||
            Time.time - _lastOcclusionBridgeStopTime < occlusionBridgeReentryCooldownSeconds ||
            IsOcclusionBridgeOwnedByAnotherTracker())
        {
            _occlusionBridgeEntryCount = 0;
            return;
        }

        bool isCloseEnough = IsHandCloseEnoughForOcclusionBridge(handPos, handToObject, handToVisualTarget);
        if (!isCloseEnough && !coverRecent)
        {
            _occlusionBridgeEntryCount = 0;
            return;
        }

        if (!IsBestOcclusionBridgeCandidate(handPos))
        {
            _occlusionBridgeEntryCount = 0;
            return;
        }

        // Entrada con histéresis: no basta un frame bueno, hacen falta occlusionBridgeEntryFrames.
        _occlusionBridgeEntryCount++;
        if (_occlusionBridgeEntryCount >= occlusionBridgeEntryFrames)
        {
            // Activa el puente y guarda la posicion local respecto a la mano para seguirla.
            _isOcclusionBridgeActive = true;
            _occlusionBridgeOwner = this;
            _occlusionBridgeExitCount = 0;
            _occlusionBridgeRestExitCount = 0;
            
            Vector3 bridgeObjectStart;
            if (!isCloseEnough && coverRecent)
            {
                // Si la mano cubre la mesa pero no esta exactamente encima, se empieza entre vision y mano.
                bridgeObjectStart = Vector3.Lerp(_visualTargetPosition, handPos, 0.7f);
            }
            else
            {
                bridgeObjectStart = _visualTargetPosition;
            }

            _occlusionBridgeLocalPosition = trackingHand.transform.InverseTransformPoint(bridgeObjectStart);
            _occlusionBridgeStartTime = Time.time;
        }
    }

    private void UpdateOcclusionBridgeCoverMemory(Vector3 handPos, float handToObject, float handToVisualTarget)
    {
        // Recuerda durante unos instantes que la mano cubrio la zona, aunque luego se separe un poco.
        if (!_hasAcceptedVisualPosition) return;
        if (IsHandDirectlyCloseForOcclusionBridge(handToObject, handToVisualTarget) || IsHandCoveringTableArea(handPos))
        {
            _lastOcclusionBridgeCoverTime = Time.time;
        }
    }

    internal bool IsHandCloseEnoughForOcclusionBridge(Vector3 handPos, float handToObject, float handToVisualTarget)
    {
        // Combina cercania directa con cobertura de la zona de mesa.
        return IsHandDirectlyCloseForOcclusionBridge(handToObject, handToVisualTarget) || IsHandCoveringTableArea(handPos);
    }

    private bool IsHandCurrentlyLinkedForOcclusionBridge(Vector3 handPos, float handToObject, float handToVisualTarget)
    {
        // Durante el puente activo se usa un radio mas laxo para evitar cortes por ruido de mano.
        if (_isOcclusionBridgeActive)
        {
            float looseRadius = Mathf.Max(occlusionBridgeEntryRadius * 3.5f, 0.16f);
            float closestHandDistance = Mathf.Min(handToObject, handToVisualTarget);
            if (closestHandDistance <= looseRadius) return true;

            return IsHandCoveringTableArea(handPos);
        }

        return IsHandDirectlyCloseForOcclusionBridge(handToObject, handToVisualTarget) || IsHandCoveringTableArea(handPos);
    }

    private bool IsHandDirectlyCloseForOcclusionBridge(float handToObject, float handToVisualTarget)
    {
        // Evalua cercania de mano al objeto real o al objetivo visual, ampliada si hay contacto reciente.
        float closestHandDistance = Mathf.Min(handToObject, handToVisualTarget);
        if (closestHandDistance <= occlusionBridgeEntryRadius) return true;

        if (contactAssessor.Level < ContactLevel.Nearby) return false;

        float contactRadius = Mathf.Max(occlusionBridgeEntryRadius, contactAssessor.nearbyRadius + occlusionBridgeContactSlack);
        return closestHandDistance <= contactRadius;
    }

    internal bool HasRecentOcclusionBridgeCover()
    {
        // Indica si la cobertura de mano sigue dentro de la ventana de memoria.
        return occlusionBridgeCoverMemorySeconds > 0f && Time.time - _lastOcclusionBridgeCoverTime <= occlusionBridgeCoverMemorySeconds;
    }

    private bool IsHandCoveringTableArea(Vector3 handPos)
    {
        // Determina si la mano cubre la zona donde se espera el objeto sobre la mesa.
        float tableCoverDistance = GetTableCoverDistance(handPos);
        return tableCoverDistance >= 0f && tableCoverDistance <= occlusionBridgeTableCoverRadius;
    }

    private float GetTableCoverDistance(Vector3 handPos)
    {
        // Mide distancia XZ de la mano al objeto y al objetivo visual cuando hay bloqueo a mesa.
        if (!lockToTable || occlusionBridgeTableCoverRadius <= 0f) return -1f;

        Vector2 handXZ = new Vector2(handPos.x, handPos.z);
        float handToObjectXZ = Vector2.Distance(handXZ, new Vector2(transform.position.x, transform.position.z));
        float handToVisualXZ = Vector2.Distance(handXZ, new Vector2(_visualTargetPosition.x, _visualTargetPosition.z));
        return Mathf.Min(handToObjectXZ, handToVisualXZ);
    }

    internal bool HasBridgeEntryMotion(float imuLinearMotion)
    {
        // Requiere movimiento lineal suficiente para entrar al puente de oclusion.
        float threshold = Mathf.Max(imuLinearMotionThreshold, occlusionBridgeEntryLinearMotionThreshold);
        return threshold > 0f && imuLinearMotion >= threshold;
    }

    internal bool IsBridgeReleaseRest(float imuLinearMotion)
    {
        // Detecta reposo suficiente para cerrar el puente sin esperar a vision.
        float threshold = Mathf.Max(0.001f, occlusionBridgeReleaseLinearMotionThreshold);
        return imuLinearMotion <= threshold;
    }

    internal bool IsOcclusionBridgeOwnedByAnotherTracker()
    {
        // Solo un tracker puede poseer el puente de oclusion en un momento dado.
        return _occlusionBridgeOwner != null && _occlusionBridgeOwner != this;
    }

    private float GetOcclusionBridgeCandidateDistance(Vector3 handPos)
    {
        // Calcula la distancia competitiva de este objeto para decidir si merece el puente.
        // Devuelve -1 cuando este tracker no es candidato razonable.
        float bestDistance = float.MaxValue;
        float tableDistance = GetTableCoverDistance(handPos);
        if (tableDistance >= 0f && tableDistance <= occlusionBridgeTableCoverRadius)
            bestDistance = tableDistance;

        float directDistance = Mathf.Min(
            Vector3.Distance(handPos, transform.position),
            Vector3.Distance(handPos, _visualTargetPosition));
        float directRadius = Mathf.Max(occlusionBridgeEntryRadius, contactAssessor.nearbyRadius + occlusionBridgeContactSlack);
        if (directDistance <= directRadius)
            bestDistance = Mathf.Min(bestDistance, directDistance);

        return bestDistance == float.MaxValue ? -1f : bestDistance;
    }

    internal bool IsBestOcclusionBridgeCandidate(Vector3 handPos)
    {
        // Compara este tracker contra los demas para evitar que dos objetos sigan la misma mano.
        // Si otro tracker esta mas cerca o se mueve mucho mas, este se retira.
        float myDistance = GetOcclusionBridgeCandidateDistance(handPos);
        if (myDistance < 0f) return false;

        for (int i = ActiveTrackers.Count - 1; i >= 0; i--)
        {
            FusionTracker tracker = ActiveTrackers[i];
            if (tracker == null) { ActiveTrackers.RemoveAt(i); continue; }
            if (tracker == this || !tracker.isActiveAndEnabled || !tracker.enableImuOcclusionHandBridge) continue;
            if (tracker.trackingHand != trackingHand && tracker.trackingHand != null && trackingHand != null) continue;

            float otherDistance = tracker.GetOcclusionBridgeCandidateDistance(handPos);
            if (otherDistance >= 0f && otherDistance + occlusionBridgeCandidateSwitchMargin < myDistance) return false;
            bool otherMovingMuchMore = tracker.LastLinearMotion > _lastReadImuLinearMotion * 2f;
            bool similarDistance = Mathf.Abs(otherDistance - myDistance) < occlusionBridgeCandidateSwitchMargin;
            if (otherDistance >= 0f && similarDistance && otherMovingMuchMore) return false;
        }
        return true;
    }

    private void StopOcclusionBridge()
    {
        // Apaga el puente de oclusion y libera la propiedad global si era de este tracker.
        bool wasActive = _isOcclusionBridgeActive;
        _isOcclusionBridgeActive = false;
        _occlusionBridgeEntryCount = 0;
        _occlusionBridgeExitCount = 0;
        _occlusionBridgeRestExitCount = 0;
        if (_occlusionBridgeOwner == this) _occlusionBridgeOwner = null;
        if (wasActive) _lastOcclusionBridgeStopTime = Time.time;
    }

    private void EnterGrip(Quaternion currentRaw)
    {
        // En agarre FSR, el objeto pasa a seguir la mano y deja de depender de vision.
        // Se guarda la posicion local respecto a la mano para conservar la distancia/orientacion del agarre.
        _isGripping = true;
        if (_rb != null) _rb.isKinematic = true;
        if (trackingHand != null) _gripLocalPosition = trackingHand.transform.InverseTransformPoint(transform.position);
    }

    private void ExitGrip(Quaternion currentRaw)
    {
        // Al soltar, coloca el objeto donde estaba la mano y pide una recalibracion visual posterior.
        // Despues de soltar puede haber un salto entre IMU y vision; por eso se marca _pendingPostReleaseCalib.
        _isGripping = false; _lastReleaseTime = Time.time;
        if (_rb != null) _rb.isKinematic = false;

        if (trackingHand != null)
        {
            Vector3 releasePos = trackingHand.transform.TransformPoint(_gripLocalPosition);
            _visualTargetPosition = releasePos; transform.position = releasePos;
        }

        _calibrationOffset = _targetCalibrationOffset = visualObject.rotation * Quaternion.Inverse(currentRaw);
        _pendingPostReleaseCalib = true;
        StopOcclusionBridge();
        _forceNextVisualUpdate = true;
    }

    private void ApplyTransform(Quaternion idealRot)
    {
        // Unico punto que escribe la pose final del GameObject.
        // Prioridad: agarre FSR > puente de oclusion mano/IMU > vision suavizada + rotacion BLE.
        if (_isGripping)
        {
            // Agarre confirmado por FSR: la mano manda posicion y el IMU manda rotacion.
            if (trackingHand != null && trackingHand.IsTracked)
            {
                transform.SetPositionAndRotation(
                    trackingHand.transform.TransformPoint(_gripLocalPosition),
                    idealRot);
            }
            else
            {
                transform.rotation = idealRot;
            }

            _smoothedRotation = idealRot;
        }
        else if (_isOcclusionBridgeActive && trackingHand != null && trackingHand.IsTracked)
        {
            // Puente activo: no hay FSR, pero hay evidencia IMU/mano suficiente para seguir la mano.
            transform.SetPositionAndRotation(
                trackingHand.transform.TransformPoint(_occlusionBridgeLocalPosition),
                idealRot);
            _smoothedRotation = idealRot;
        }
        else
        {
            // Fuera de agarre, mezcla posicion visual y rotacion fisica segun nivel de manipulacion.
            // Si no se manipula, ShapePolicy puede encajar el objeto en una pose estable sobre la mesa.
            bool isManipulated = contactAssessor.Level >= ContactLevel.Touching || contactAssessor.IsIMUMoving;
            Quaternion stableRot = (shape != null && !isManipulated) ? shape.SnapRestingPose(idealRot, ref _cachedDownAxis) : idealRot;

            // La velocidad de seguimiento cambia segun confianza: mas rapida durante contacto/movimiento.
            float speed = visualCorrectionSpeed;
            if (contactAssessor.Level >= ContactLevel.Touching) speed *= 4f;
            else if (contactAssessor.IsIMUMoving) speed *= 2.5f;
            else speed *= 0.5f;

            bool movingWithoutFsr = !_isGripping && isManipulated && (Time.time - LastVisualUpdateTime) < 0.15f;

            if (movingWithoutFsr)
            {
                // Si hay movimiento sin FSR pero vision fresca, evita retraso en la posicion.
                transform.position = _visualTargetPosition;
            }
            else
            {
                transform.position = Vector3.Lerp(transform.position, _visualTargetPosition, Time.deltaTime * speed);
            }

            if (isManipulated)
            {
                // Durante manipulacion real no se suaviza la rotacion para no retrasar la respuesta.
                _smoothedRotation = stableRot;
            }
            else
            {
                if (Quaternion.Angle(_smoothedRotation, stableRot) > 0.05f)
                    _smoothedRotation = Quaternion.Slerp(_smoothedRotation, stableRot, Time.deltaTime * rotationSmoothSpeed);
                else _smoothedRotation = stableRot;
            }

            transform.rotation = _smoothedRotation;
        }
    }

    public void ResetState()
    {
        // Reinicia calibracion, contacto, puente, logs y visuales auxiliares para un nuevo ensayo.
        // No destruye el tracker ni cambia su identidad: BLE/FSM pueden reutilizarlo tras reconexion.
        _targetCalibrationOffset = _calibrationOffset = Quaternion.identity;
        _isGripping = false;
        StopOcclusionBridge();
        _lastOcclusionBridgeCoverTime = -10f;
        _hasAcceptedVisualPosition = false;
        _forceNextVisualUpdate = true;
        _hasInitialCalibration = false; _stableFramesCount = 0;
        _pendingPostReleaseCalib = false; _cachedDownAxis = Vector3.down;
        ClearOffsetBuffer(); _wasStable = false;
        _headStableFrames = 0;
        _lastHeadRotation = playerHead != null ? playerHead.rotation : Quaternion.identity;
        _hasYoloThisFrame = false;
        _hasLastLoggedPose = false;
        _lastLoggedPosition = transform.position;
        _lastLoggedRotation = visualObject != null ? visualObject.rotation : transform.rotation;
        contactAssessor.Reset();

        _keypointDebug?.Hide();
    }
}
