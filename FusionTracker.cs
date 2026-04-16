// FusionTracker.cs
// Sensor-Fusion Tracking System — AIR Group, ESI-UCLM
//
// Tracker individual por cubo. Fusiona la orientación del IMU (BLE) con la
// posición + yaw de los keypoints (YOLO + depth map) para generar un pose
// 6-DoF estable y con baja latencia para el gemelo digital del cubo.
//
// Arquitectura de fusión:
//   BLE (IMU+FSR, 15-30 Hz) + Visión (YOLO+Depth, 1-5 Hz) → FusionTracker
//     → Calibración yaw, Lerp de posición, Detección de grip, Bloqueo de cara
//     → Gemelo digital (visualObject) con pose 6-DoF estable
//
// Calibración de yaw (3 caminos, añadidos incrementalmente según necesidades):
//   1. INITIAL — Primera calibración: se aplica inmediata, 3 frames de estabilidad.
//   2. POST-RELEASE — Tras soltar el cubo: una corrección inmediata si la detección
//      es buena (4/4 kpts + geo válido), si no llega en 5s se desiste.
//   3. ONGOING — Acumula muestras en un buffer, calcula la mediana, y si son
//      consistentes (diferencia < 20°) aplica la corrección. Espera 2s entre
//      correcciones. Si el error es grande (>15°) corrige de golpe, si no suaviza.
//
// Grip — El FSR (sensor de presión) usa umbrales distintos para entrar y salir
// del grip (más alto para entrar, más bajo para salir) para evitar rebotes.
// Durante grip: rotación 3D completa del IMU + posición de la mano.
// Al soltar: recaptura el offset completo preservando qué cara está abajo.
//
// Bloqueo de cara — Cuando el cubo está en la mesa, detecta cuál de las 6 caras
// está abajo y la bloquea contra el suelo. Solo deja rotar en yaw (giro horizontal).
// Usa un margen para no cambiar de cara por ruido del sensor.
//
// Gyro bias — Estimación opcional del drift del giroscopio durante reposo.
// Desactivado por defecto. Los IMUs 6-ejes (sin magnetómetro) derivan en yaw
// con el tiempo; esto lo mide cuando el cubo está quieto y lo compensa.
//
// Historial relevante:
//   - Los tres paths de calibración se fueron añadiendo según surgían problemas:
//     primero el initial, luego el ongoing, luego el post-release.
//   - El symmetry snap de 90° venía del branch de QR codes y se desactivó porque
//     bloqueaba las correcciones y rompía el cambio de ejes por cara.
//   - El auto-scale de realCubeDimensions se añadió porque el cubo real no es
//     perfectamente cúbico y la rotación del gemelo digital no coincidía.
//   - El sticker face-check se desactivó (comentado) porque funciona bien sin él.

using UnityEngine;
using System;

public class FusionTracker : MonoBehaviour
{
    [Header("Identity")]
    public bool visionOnlyMode = false;
    public string assignedName;
    public DetectedColor myColorIdentity;

    [Header("References")]
    public Transform visualObject;
    // Mano activa para tracking de grip. Se asigna desde FusionSystemManager
    // según la elección del paciente en la calibración (puede ser derecha o izquierda).
    public OVRHand trackingHand;
    public Transform playerHead;
    public SurfaceSnapper surfaceSnapper;

    [Header("Real Cube Dimensions (metres)")]
    [Tooltip("Exact dimensions of the physical cube in metres.\n" +
             "Measure with a ruler/caliper. X=width, Y=height, Z=depth.\n" +
             "Example: 3.0cm × 3.0cm × 3.5cm → (0.030, 0.030, 0.035)\n" +
             "Set to (0,0,0) to skip auto-scaling.")]
    public Vector3 realCubeDimensions = Vector3.zero;

    [Header("Movement Control")]
    public float visualCorrectionSpeed = 12.0f;
    public Vector3 inferenceViewOffset = new Vector3(-0.032f, 0f, 0f);
    public float angularDeadzoneDeg = 1.5f;

    [Tooltip("Vertical offset applied to the depth-map position (metres).\n" +
             "Depth map hits the top face; pivot is at the base → set to -halfHeight.")]
    public float depthYOffset = -0.015f;

    [Tooltip("Minimum distance (metres) the new detection must differ from\n" +
             "the current target to accept the update. Filters depth-map jitter.\n" +
             "0.005 = 5mm. Increase if the cube still jitters.")]
    public float positionDeadzoneMeters = 0.005f;

    [Header("Rotation & Stability")]
    public float rotationSmoothSpeed = 30.0f;
    public float calibrationLerpSpeed = 8.0f;
    public float stabilityAngleThreshold = 3.0f;
    public float keypointCalibrationDeadzoneDeg = 0f;
    public float instantCorrectionThresholdDeg = 15.0f;
    public float rotationDeadzone = 0.1f;

    [Tooltip("Minimum seconds between ongoing recalibrations.")]
    public float recalibCooldownSeconds = 2.0f;

    [Tooltip("When ON: ongoing recalibration requires 4 keypoints + geometric check.")]
    public bool requireHighConfidenceForRecalib = true;

    // ── Umbrales de estabilidad ──────────────────────────────────────
    // Antes: stabilityRequiredFrames = 40 → a ~1-2 Hz de detección = 20-40s de espera.
    // Ahora: relajado a 8 frames, mucho más alcanzable en Quest.
    [Header("Stability (relaxed for Quest detection rates)")]
    public int stabilityRequiredFrames = 8;
    public int initialStabilityRequiredFrames = 3;

    // ── Filtro temporal ──────────────────────────────────────────────
    // Antes: medianFilterSize=5, maxBufferSpreadDeg=12.
    // Ahora: buffer más pequeño, tolerancia de spread más amplia.
    [Header("Temporal Filter (relaxed)")]
    public int medianFilterSize = 3;
    public float maxBufferSpreadDeg = 20.0f;

    [Header("Initial Calibration")]
    public float initialCalibrationMaxOffset = 180f;

    [Header("Physical Mounting")]
    public Vector3 mountingRotation = Vector3.zero;

    // ── Calibración con simetría ─────────────────────────────────────
    // Portado del branch de QR. El snap de 90° está desactivado para keypoints
    // porque bloqueaba las correcciones (ver nota en ApplyKeypointCalibration).
    [Header("Symmetry-Aware Calibration")]
    [Tooltip("Enable 90° snap for cube symmetry, like the QR branch does.")]
    public bool useSymmetrySnap = true;

    // ── Estimación de bias del giroscopio ────────────────────────────
    // Los IMUs 6-ejes (sin magnetómetro) derivan en yaw con el tiempo.
    // Esto mide cuánto drift hay cuando el cubo está quieto y lo compensa.
    // Desactivado por defecto, activar si se nota drift acumulado en reposo.
    [Header("Gyro Bias Estimation")]
    [Tooltip("Enable automatic yaw bias estimation during rest periods.")]
    public bool enableBiasEstimation = false;
    [Tooltip("Seconds of stability required before bias estimation starts.")]
    public float biasEstimationDelay = 1.5f;
    [Tooltip("How fast the bias estimate converges (lower = slower, more stable).")]
    public float biasLearningRate = 0.02f;

    [Header("Debug Keypoints")]
    [Tooltip("Show keypoint spheres and direction arrows on Quest.")]
    public bool showKeypointDebug = true;
    public float keypointSphereRadius = 0.004f;
    public bool verboseCalibrationLog = true;

    [Header("Experimentation")]
    public Transform groundTruthTransform;
    public event Action<Vector3, Vector3, double> OnVisualDetection;
    public event Action<double> OnGripStarted;
    public event Action<double> OnGripEnded;
    public event Action<float, double> OnCalibrationErrorMeasured;
    private bool _expEnabled = false;
    public void SetExperimentation(bool en) => _expEnabled = en;

    [Header("Grip Detection")]
    public int gripThresholdEnter = 20;
    public int gripThresholdExit = 10;
    public float grabCooldown = 0.5f;

    // ── Estado privado ───────────────────────────────────────────────

    private bool _isGripping;
    private float _lastReleaseTime;
    private Rigidbody _rb;

    // Posición del cubo en espacio local de la mano cuando se agarra.
    // Se usa para mover el cubo con la mano durante el grip.
    private Vector3 _gripLocalPosition;

    // Offset de calibración: la rotación que hay que aplicar al IMU raw para
    // que coincida con lo que ve la cámara. Se calcula a partir de los keypoints.
    private Quaternion _calibrationOffset = Quaternion.identity;

    // Target hacia el que interpola _calibrationOffset (para transiciones suaves)
    private Quaternion _targetCalibrationOffset = Quaternion.identity;

    // Posición 3D objetivo del cubo, viene de YOLO + depth map
    private Vector3 _visualTargetPosition;

    // Última rotación válida leída del IMU
    private Quaternion _lastValidSensorRotation;

    // Rotación suavizada que se aplica al visualObject (evita saltos)
    private Quaternion _smoothedRotation;

    // Contador de frames consecutivos en los que el IMU no ha cambiado mucho.
    // Se usa para el gate de estabilidad antes de calibrar.
    private int _stableFramesCount;

    // Flags de control
    private bool _forceNextVisualUpdate = false;
    private bool _hasInitialCalibration = false;

    // ── Buffer del filtro temporal (mediana) ─────────────────────────
    // Se van acumulando offsets de yaw. Cuando hay suficientes muestras
    // y son consistentes (spread bajo), se aplica la mediana como corrección.
    private float[] _offsetBuffer;
    private int _offsetWriteIdx;
    private int _offsetCount;
    private bool _wasStable;
    private float _lastRecalibTime;

    // Flag que se activa al soltar el cubo: esperamos una detección de alta
    // confianza para recalibrar. Si no llega en 5s, timeout.
    private bool _pendingPostReleaseCalib;

    // ── Estado del bias del giroscopio ────────────────────────────────
    private float _yawBiasEstimate = 0f;       // bias acumulado en deg/s
    private float _stableRestTime = 0f;         // segundos estable y sin grip
    private Quaternion _biasReferenceRot;       // rotación al empezar el reposo
    private bool _biasReferenceSet = false;

    // ── Visualización de debug (generado con IA generativa) ────────
    // 4 esferitas (una por keypoint) + 2 flechas (visión vs IMU)
    private GameObject[] _kpSpheres;
    private LineRenderer _arrowVisual;   // verde: dirección según keypoints
    private LineRenderer _arrowSensor;   // cyan: dirección según IMU
    private static readonly Color[] KP_COLORS = { Color.red, Color.green, Color.blue, Color.yellow };

    // ── Propiedades públicas ─────────────────────────────────────────

    public bool IsStable => _stableFramesCount >= stabilityRequiredFrames;
    public bool IsStableForInitial => _stableFramesCount >= initialStabilityRequiredFrames;
    public bool HasInitialCalibration => _hasInitialCalibration;
    public float LastVisualUpdateTime { get; private set; }
    public double LastDetectionTime { get; private set; }
    public Quaternion HybridRotation => visualObject != null ? visualObject.rotation : transform.rotation;
    public Quaternion WristRotation => trackingHand != null ? trackingHand.transform.rotation : Quaternion.identity;
    public bool IsGripping => _isGripping;
    public float YawBiasEstimate => _yawBiasEstimate;

    // ══════════════════════════════════════════════════════════════════
    //  Lifecycle
    // ══════════════════════════════════════════════════════════════════

    private void Awake()
    {
        // Si no se asignó visualObject en Inspector, usamos nuestro propio transform
        if (visualObject == null) visualObject = transform;
        _rb = GetComponent<Rigidbody>();
        _visualTargetPosition = transform.position;
        _lastValidSensorRotation = _smoothedRotation = transform.rotation;

        // Buscar la cámara principal como referencia de la cabeza del jugador
        if (playerHead == null && Camera.main != null) playerHead = Camera.main.transform;

        // Intentar encontrar SurfaceSnapper en nosotros mismos o en hijos
        if (surfaceSnapper == null) surfaceSnapper = GetComponent<SurfaceSnapper>();
        if (surfaceSnapper == null) surfaceSnapper = GetComponentInChildren<SurfaceSnapper>();

        // ── Auto-escalar el visual para que coincida con el cubo real ──
        // Añadido porque el cubo real no es perfectamente cúbico (ej: 3x3x3.5cm)
        // y al rotar se notaba que el gemelo digital no coincidía.
        if (realCubeDimensions.x > 0f && realCubeDimensions.y > 0f && realCubeDimensions.z > 0f)
        {
            // Asumimos que la mesh base es un cubo 1×1×1.
            Renderer rend = visualObject.GetComponentInChildren<Renderer>();
            if (rend != null)
            {
                // Tamaño nativo de la mesh sin escalar
                Vector3 meshSize = rend.localBounds.size;
                if (meshSize.x > 0f && meshSize.y > 0f && meshSize.z > 0f)
                {
                    Vector3 currentScale = visualObject.localScale;

                    // Tamaño real de la mesh con la escala actual
                    Vector3 actualMeshSize = Vector3.Scale(meshSize, currentScale);

                    // Calcular la escala necesaria para que la mesh mida exactamente
                    // lo mismo que el cubo físico real
                    visualObject.localScale = new Vector3(
                        currentScale.x * (realCubeDimensions.x / actualMeshSize.x),
                        currentScale.y * (realCubeDimensions.y / actualMeshSize.y),
                        currentScale.z * (realCubeDimensions.z / actualMeshSize.z));

                    Debug.Log($"[FT:{myColorIdentity}] Scaled visual to match real cube: " +
                              $"{realCubeDimensions.x * 100:F1}×{realCubeDimensions.y * 100:F1}×{realCubeDimensions.z * 100:F1}cm " +
                              $"(scale={visualObject.localScale})");
                }
            }

        }

        _forceNextVisualUpdate = true;

        // Inicializar el buffer del filtro de mediana con el tamaño configurado (mínimo 3)
        _offsetBuffer = new float[Mathf.Max(3, medianFilterSize)];
        InitDebugVisuals();
    }

    private void OnDestroy() { DestroyDebugVisuals(); }

    // ══════════════════════════════════════════════════════════════════
    //  Visualización de debug (generado con IA generativa)
    // ══════════════════════════════════════════════════════════════════

    // Crea 4 esferitas de colores (una por keypoint) y 2 flechas (visión e IMU)
    // para poder depurar visualmente en Quest.
    private void InitDebugVisuals()
    {
        _kpSpheres = new GameObject[4];
        for (int i = 0; i < 4; i++)
        {
            _kpSpheres[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _kpSpheres[i].name = $"KP_{myColorIdentity}_{i}";
            _kpSpheres[i].transform.localScale = Vector3.one * keypointSphereRadius * 2f;

            // Quitar el collider para que no interfiera con la física
            var col = _kpSpheres[i].GetComponent<Collider>();
            if (col) Destroy(col);

            // Material unlit para que se vea sin importar la iluminación
            var r = _kpSpheres[i].GetComponent<Renderer>();
            if (r) { r.material = new Material(Shader.Find("Unlit/Color")); r.material.color = KP_COLORS[i]; }
            _kpSpheres[i].SetActive(false);
        }

        // Flecha verde = dirección según keypoints (visión)
        // Flecha cyan = dirección según IMU (sensor)
        _arrowVisual = MakeArrow($"Arrow_Visual_{myColorIdentity}", Color.green);
        _arrowSensor = MakeArrow($"Arrow_Sensor_{myColorIdentity}", Color.cyan);
    }

    // Crea un LineRenderer con forma de flecha (ancho decreciente)
    private LineRenderer MakeArrow(string name, Color c)
    {
        var go = new GameObject(name);
        var lr = go.AddComponent<LineRenderer>();
        lr.positionCount = 2;
        lr.startWidth = 0.003f;
        lr.endWidth = 0.001f;
        lr.material = new Material(Shader.Find("Unlit/Color"));
        lr.material.color = c;
        lr.useWorldSpace = true;
        go.SetActive(false);
        return lr;
    }

    private void DestroyDebugVisuals()
    {
        if (_kpSpheres != null)
            foreach (var s in _kpSpheres) if (s) Destroy(s);
        if (_arrowVisual) Destroy(_arrowVisual.gameObject);
        if (_arrowSensor) Destroy(_arrowSensor.gameObject);
    }

    // Actualiza las esferitas de debug en la posición de los keypoints
    // y las flechas de dirección (verde=visión, cyan=IMU).
    // Llamado desde FusionSystemManager en cada detección de CuboSensor.
    public void UpdateKeypointDebug(Vector3[] kptsWorld, bool[] kptsValid,
                                     Quaternion sensorRotation)
    {
        // Contar keypoints válidos para decidir si mostramos flechas
        int validCount = 0;
        if (kptsValid != null)
            for (int i = 0; i < kptsValid.Length; i++)
                if (kptsValid[i]) validCount++;

        // Mostrar/ocultar cada esferita según si su keypoint es válido
        for (int i = 0; i < 4; i++)
        {
            bool vis = showKeypointDebug && kptsValid != null
                     && i < kptsValid.Length && kptsValid[i];
            _kpSpheres[i].SetActive(vis);
            if (vis) _kpSpheres[i].transform.position = kptsWorld[i];
        }

        // Con menos de 3 keypoints no podemos calcular dirección fiable
        if (!showKeypointDebug || validCount < 3)
        {
            if (_arrowVisual) _arrowVisual.gameObject.SetActive(false);
            if (_arrowSensor) _arrowSensor.gameObject.SetActive(false);
            return;
        }

        // Calcular la dirección frontal de la cara del sticker (solo XZ, sin Y)
        Vector3 faceDir = ComputeFaceDirectionXZ(kptsWorld, kptsValid);
        Vector3 center = ComputeValidCenter(kptsWorld, kptsValid);

        // Dirección frontal del IMU proyectada al plano horizontal
        Vector3 sensorFwd = sensorRotation * Vector3.forward;
        sensorFwd.y = 0;

        float arrowLen = 0.07f;

        // Flecha verde: hacia donde apuntan los keypoints
        if (_arrowVisual)
        {
            bool valid = faceDir.sqrMagnitude > 0.001f;
            _arrowVisual.gameObject.SetActive(valid);
            if (valid)
            {
                _arrowVisual.SetPosition(0, center);
                _arrowVisual.SetPosition(1, center + faceDir.normalized * arrowLen);
            }
        }

        // Flecha cyan: hacia donde apunta el IMU (ligeramente elevada para que no se solape)
        if (_arrowSensor)
        {
            bool valid = sensorFwd.sqrMagnitude > 0.001f;
            _arrowSensor.gameObject.SetActive(valid);
            if (valid)
            {
                Vector3 off = Vector3.up * 0.005f;
                _arrowSensor.SetPosition(0, center + off);
                _arrowSensor.SetPosition(1, center + off + sensorFwd.normalized * arrowLen);
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Dirección frontal desde keypoints
    // ══════════════════════════════════════════════════════════════════

    // Calcula la dirección "forward" de la cara del sticker en el plano XZ (solo yaw).
    //
    // Layout de los keypoints en la cara del sticker:
    //   KP0: Arriba-Derecha    KP1: Arriba-Izquierda
    //   KP3: Abajo-Derecha     KP2: Abajo-Izquierda
    //
    // Método: promedio del vector izquierdo (KP2→KP1) y derecho (KP3→KP0).
    // Todos aplanados a Y=0 porque solo nos interesa el yaw.
    // Devuelve zero si hay menos de 3 keypoints válidos.
    private Vector3 ComputeFaceDirectionXZ(Vector3[] kpts, bool[] valid)
    {
        int validCount = 0;
        for (int i = 0; i < 4; i++)
            if (valid != null && i < valid.Length && valid[i]) validCount++;

        if (validCount < 3) return Vector3.zero;

        // Aplanar todos los puntos a Y=0 (solo queremos yaw)
        Vector3[] flat = new Vector3[4];
        for (int i = 0; i < 4; i++)
            flat[i] = new Vector3(kpts[i].x, 0f, kpts[i].z);

        Vector3 forwardDir = Vector3.zero;

        // Lado izquierdo: de abajo-izquierda a arriba-izquierda
        if (valid[2] && valid[1])
            forwardDir += (flat[1] - flat[2]);

        // Lado derecho: de abajo-derecha a arriba-derecha
        if (valid[3] && valid[0])
            forwardDir += (flat[0] - flat[3]);

        // Promediando ambos lados nos da la dirección frontal.
        // Nota: se eliminó el chequeo con Vector3.Dot y toCamera que
        // volteaba el cubo en algunos casos.

        return forwardDir.sqrMagnitude > 0.0001f ? forwardDir.normalized : Vector3.zero;
    }

    // Overload de compatibilidad: asume los 4 keypoints válidos
    private Vector3 ComputeFaceDirectionXZ(Vector3[] kpts)
    {
        bool[] allValid = { true, true, true, true };
        return ComputeFaceDirectionXZ(kpts, allValid);
    }

    // Centroide de solo los keypoints válidos
    private static Vector3 ComputeValidCenter(Vector3[] kpts, bool[] valid)
    {
        Vector3 sum = Vector3.zero;
        int count = 0;
        for (int i = 0; i < 4; i++)
        {
            if (valid != null && i < valid.Length && valid[i])
            {
                sum += kpts[i];
                count++;
            }
        }
        return count > 0 ? sum / count : Vector3.zero;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Calibración por keypoints (3 caminos: initial, post-release, ongoing)
    // ══════════════════════════════════════════════════════════════════

    // Punto de entrada principal para calibrar el yaw.
    // Requiere 3+ keypoints válidos y datos activos del IMU.
    //
    // Los tres caminos se fueron añadiendo según las necesidades:
    //   PATH 1 (initial): el primero que se implementó, calibración inmediata.
    //   PATH 3 (ongoing): se añadió después para corregir drift acumulado.
    //   PATH 2 (post-release): se añadió al final porque al soltar el cubo
    //     tras girarlo, el offset cambiaba y necesitaba recalibrarse rápido.
    public void ApplyKeypointCalibration(Vector3[] kptsWorld, bool[] kptsValid,
                                          Quaternion sensorRotation,
                                          bool isGeometricValid, int validKeypointCount)
    {
        // No calibrar mientras el usuario tiene el cubo en la mano
        if (_isGripping) return;

        // Gate mínimo: necesitamos al menos 3 keypoints
        if (validKeypointCount < 3)
        {
            if (verboseCalibrationLog && Time.frameCount % 60 == 0)
                Debug.Log($"[FT:{myColorIdentity}] GATE: validKPs={validKeypointCount} < 3");
            return;
        }

        // ── Sticker face-check (desactivado) ─────────────────────────
        // Comprobaba si el sticker estaba de lado mirando la distancia de la
        // fila superior vs inferior al jugador. Funciona bien sin él ahora,
        // así que se dejó comentado.
        if (playerHead != null)
        {
            float topDist = 0f;   // KP0 + KP1 (fila superior)
            int topCount = 0;
            float botDist = 0f;   // KP2 + KP3 (fila inferior)
            int botCount = 0;

            for (int i = 0; i < 4; i++)
            {
                if (!kptsValid[i]) continue;
                float d = Vector3.Distance(playerHead.position, kptsWorld[i]);
                if (i <= 1) { topDist += d; topCount++; }
                else { botDist += d; botCount++; }
            }

            if (topCount > 0 && botCount > 0)
            {
                float avgTop = topDist / topCount;
                float avgBot = botDist / botCount;
                // Si la fila inferior está más lejos, el sticker está de lado.
                // Desactivado porque ahora no hace falta.
                /*
                if (avgBot > avgTop + 0.015f) // 1.5cm tolerancia
                {
                    if (verboseCalibrationLog)
                        Debug.Log($"[FT:{myColorIdentity}] GATE: sticker sideways " +
                                  $"(top={avgTop:F3} bot={avgBot:F3})");
                    return;
                }
                */
            }
        }

        // ── Gate de estabilidad ──────────────────────────────────────
        // No calibrar si el IMU se está moviendo mucho (el cubo no está quieto).
        // Para la calibración inicial pedimos menos frames de estabilidad
        // porque queremos que el cubo "arranque" rápido.
        if (!_hasInitialCalibration)
        {
            if (!IsStableForInitial)
            {
                if (verboseCalibrationLog && Time.frameCount % 60 == 0)
                    Debug.Log($"[FT:{myColorIdentity}] GATE: waiting initial stability " +
                              $"{_stableFramesCount}/{initialStabilityRequiredFrames}");
                return;
            }
        }
        else
        {
            if (!IsStable)
            {
                if (verboseCalibrationLog && Time.frameCount % 60 == 0)
                    Debug.Log($"[FT:{myColorIdentity}] GATE: waiting stability " +
                              $"{_stableFramesCount}/{stabilityRequiredFrames}");
                return;
            }
        }

        // ── Calcular el offset de yaw ────────────────────────────────
        // Comparamos hacia dónde apuntan los keypoints (visión) vs hacia dónde
        // apunta el IMU (sensor). La diferencia es el error de yaw que hay que corregir.
        Vector3 faceDir = ComputeFaceDirectionXZ(kptsWorld, kptsValid);
        if (faceDir == Vector3.zero)
        {
            if (verboseCalibrationLog)
                Debug.Log($"[FT:{myColorIdentity}] GATE: faceDir is zero");
            return;
        }

        // Dirección forward del IMU proyectada al plano horizontal
        Vector3 sensorFwd = sensorRotation * Vector3.forward;
        sensorFwd.y = 0;
        if (sensorFwd.sqrMagnitude < 0.0001f)
        {
            if (verboseCalibrationLog)
                Debug.Log($"[FT:{myColorIdentity}] GATE: sensorFwd is zero");
            return;
        }
        sensorFwd.Normalize();

        // Convertir ambas direcciones a ángulos de yaw y calcular la diferencia
        float visualYaw = Mathf.Atan2(faceDir.x, faceDir.z) * Mathf.Rad2Deg;
        float sensorYaw = Mathf.Atan2(sensorFwd.x, sensorFwd.z) * Mathf.Rad2Deg;
        float rawOffset = Mathf.DeltaAngle(sensorYaw, visualYaw);

        // Nota: el symmetry snap de 90° (del branch de QR) está DESACTIVADO.
        // Con QR tenías 4 caras posibles → ambigüedad de 90°. Con keypoints
        // solo hay una cara (la del sticker), así que rawOffset ES el error real.
        // El snap redondeaba errores intermedios (ej: 45°) a 0° y bloqueaba
        // todas las correcciones.

        Debug.Log($"[FT:{myColorIdentity}] PASSED all gates: vYaw={visualYaw:F1} sYaw={sensorYaw:F1} " +
                  $"raw={rawOffset:F1} " +
                  $"geo={isGeometricValid} kps={validKeypointCount} " +
                  $"init={_hasInitialCalibration} stable={_stableFramesCount}");

        // ── PATH 1: CALIBRACIÓN INICIAL ──────────────────────────────
        // Primera vez que calibramos. Se aplica inmediatamente sin suavizado
        // porque no hay referencia previa con la que hacer lerp.
        if (!_hasInitialCalibration)
        {
            if (Mathf.Abs(rawOffset) > initialCalibrationMaxOffset) return;

            // Aplicar directamente: tanto el target como el offset actual
            _targetCalibrationOffset = Quaternion.Euler(0, rawOffset, 0);
            _calibrationOffset = _targetCalibrationOffset;
            _hasInitialCalibration = true;
            _pendingPostReleaseCalib = false;
            ClearOffsetBuffer();
            if (_expEnabled) OnCalibrationErrorMeasured?.Invoke(Mathf.Abs(rawOffset), ExperimentClock.Now);
            Debug.Log($"[FT:{myColorIdentity}] ★ INITIAL calib: {rawOffset:F1}° (raw, no snap)");
            return;
        }

        // ── PATH 2: CALIBRACIÓN POST-RELEASE ────────────────────────
        // Justo después de soltar el cubo, necesitamos recalibrar rápido porque
        // el usuario puede haberlo girado. Esperamos UNA detección de alta
        // confianza (4/4 keypoints + validación geométrica) y la aplicamos
        // inmediatamente. Si no llega en 5s, desistimos.
        if (_pendingPostReleaseCalib)
        {
            bool highConfidence = validKeypointCount == 4 && isGeometricValid;

            if (highConfidence && Mathf.Abs(rawOffset) < 90f)
            {
                // Aplicar directamente, sin mediana ni suavizado
                _targetCalibrationOffset = Quaternion.Euler(0, rawOffset, 0);
                _calibrationOffset = _targetCalibrationOffset;
                _pendingPostReleaseCalib = false;
                _lastRecalibTime = Time.time;
                ClearOffsetBuffer();
                Debug.Log($"[FT:{myColorIdentity}] ★ POST-RELEASE calib: {rawOffset:F1}° " +
                          $"(4kp, geo=true, high confidence)");
                return;
            }

            // Timeout: si en 5s no llega una buena detección, dejamos de esperar
            if (Time.time - _lastReleaseTime > 5.0f)
            {
                _pendingPostReleaseCalib = false;
                if (verboseCalibrationLog)
                    Debug.Log($"[FT:{myColorIdentity}] Post-release calib timeout (no good keypoints in 5s)");
            }

            // Mientras esperamos post-release, no caemos al ongoing
            return;
        }

        // ── PATH 3: ONGOING (buffer de mediana + cooldown) ───────────
        // Corrección continua del drift. Se acumulan muestras de offset en un
        // buffer circular, se calcula la mediana, y si es consistente se aplica.
        // Cooldown entre recalibraciones para no estar corrigiendo cada frame.

        // Respetar el cooldown entre recalibraciones
        if (Time.time - _lastRecalibTime < recalibCooldownSeconds)
            return;

        // Solo muestras de alta confianza entran al buffer de recalibración
        if (requireHighConfidenceForRecalib && (validKeypointCount < 4 || !isGeometricValid))
        {
            if (verboseCalibrationLog && Time.frameCount % 60 == 0)
                Debug.Log($"[FT:{myColorIdentity}] ONGOING GATE: kps={validKeypointCount} geo={isGeometricValid} — skipped");
            return;
        }

        // Escribir la muestra en el buffer circular
        _offsetBuffer[_offsetWriteIdx] = rawOffset;
        _offsetWriteIdx = (_offsetWriteIdx + 1) % _offsetBuffer.Length;
        _offsetCount = Mathf.Min(_offsetCount + 1, _offsetBuffer.Length);

        // Esperar a tener suficientes muestras para calcular mediana
        int needed = Mathf.Max(3, medianFilterSize);
        if (_offsetCount < needed)
        {
            if (verboseCalibrationLog)
                Debug.Log($"[FT:{myColorIdentity}] BUFFER: {_offsetCount}/{needed} samples (raw={rawOffset:F1}°)");
            return;
        }

        // Comprobar que las muestras son consistentes (spread bajo).
        // Si hay mucho spread, los datos son ruidosos y limpiamos el buffer.
        float minV = float.MaxValue, maxV = float.MinValue;
        for (int i = 0; i < _offsetCount; i++)
        {
            if (_offsetBuffer[i] < minV) minV = _offsetBuffer[i];
            if (_offsetBuffer[i] > maxV) maxV = _offsetBuffer[i];
        }
        if (maxV - minV > maxBufferSpreadDeg)
        {
            Debug.Log($"[FT:{myColorIdentity}] BUFFER: spread {maxV - minV:F1}° > {maxBufferSpreadDeg}° — cleared");
            ClearOffsetBuffer();
            return;
        }

        // Calcular la mediana del buffer
        float median = ComputeMedian();
        float absM = Mathf.Abs(median);

        // Si la corrección es menor que la deadzone, no merece la pena aplicarla
        if (absM < keypointCalibrationDeadzoneDeg)
        {
            if (verboseCalibrationLog)
                Debug.Log($"[FT:{myColorIdentity}] DEADZONE: median={median:F1}° < {keypointCalibrationDeadzoneDeg}°");
            ClearOffsetBuffer();
            return;
        }

        // Aplicar la corrección. Si el error es grande (>15°), instantáneo.
        // Si es pequeño, se interpola suavemente con lerp en LateUpdate.
        _targetCalibrationOffset = Quaternion.Euler(0, median, 0);
        _lastRecalibTime = Time.time;
        if (absM >= instantCorrectionThresholdDeg)
        {
            _calibrationOffset = _targetCalibrationOffset;
            Debug.Log($"[FT:{myColorIdentity}] ★ INSTANT recalib: {median:F1}°");
        }
        else
        {
            Debug.Log($"[FT:{myColorIdentity}] ★ Smooth recalib: {median:F1}°");
        }
        if (_expEnabled) OnCalibrationErrorMeasured?.Invoke(absM, ExperimentClock.Now);
        ClearOffsetBuffer();
    }

    // Overload de compatibilidad con la firma antigua de 3 parámetros
    public void ApplyKeypointCalibration(Vector3[] kptsWorld, Quaternion sensorRotation,
                                          bool isGeometricValid)
    {
        bool[] allValid = { true, true, true, true };
        ApplyKeypointCalibration(kptsWorld, allValid, sensorRotation, isGeometricValid, 4);
    }

    // Extrae solo el yaw de un quaternion, eliminando pitch y roll
    private static Quaternion ForceYawOnly(Quaternion q)
    {
        return Quaternion.Euler(0f, q.eulerAngles.y, 0f);
    }

    private void ClearOffsetBuffer() { _offsetCount = 0; _offsetWriteIdx = 0; }

    // Calcula la mediana del buffer de offsets. Se usa en vez de la media
    // porque la mediana es más robusta frente a outliers.
    private float ComputeMedian()
    {
        float[] s = new float[_offsetCount];
        for (int i = 0; i < _offsetCount; i++) s[i] = _offsetBuffer[i];
        Array.Sort(s);
        int m = _offsetCount / 2;
        return (_offsetCount % 2 == 0) ? (s[m - 1] + s[m]) * 0.5f : s[m];
    }

    // ══════════════════════════════════════════════════════════════════
    //  Estimación de bias del giroscopio
    // ══════════════════════════════════════════════════════════════════

    // Los IMUs 6-ejes (sin magnetómetro) derivan en yaw con el tiempo.
    // Cuando el cubo está quieto en la mesa, medimos cuánto drift hay
    // y acumulamos una estimación con media móvil exponencial (EMA).
    // Esa estimación se resta cada frame en CompensateBias().
    //
    // Desactivado por defecto (enableBiasEstimation = false).
    private void UpdateBiasEstimation(Quaternion currentRaw)
    {
        if (!enableBiasEstimation) return;

        // Solo estimamos bias cuando el cubo está quieto y no se está agarrando
        bool atRest = !_isGripping && IsStable;

        if (!atRest)
        {
            // Se movió o se agarró: resetear el contador de reposo
            _stableRestTime = 0f;
            _biasReferenceSet = false;
            return;
        }

        _stableRestTime += Time.deltaTime;

        // Guardar la rotación de referencia al empezar el reposo
        if (!_biasReferenceSet)
        {
            _biasReferenceRot = currentRaw;
            _biasReferenceSet = true;
            return;
        }

        // Esperar un poco antes de empezar a estimar (dejar que se estabilice)
        if (_stableRestTime < biasEstimationDelay) return;

        // Medir cuánto ha derivado el yaw desde la referencia
        float refYaw = _biasReferenceRot.eulerAngles.y;
        float curYaw = currentRaw.eulerAngles.y;
        float yawDrift = Mathf.DeltaAngle(refYaw, curYaw);

        // Calcular tasa de drift (deg/s) en el periodo de reposo
        float elapsed = _stableRestTime;
        if (elapsed > 0.1f)
        {
            float driftRate = yawDrift / elapsed;

            // EMA: converge lentamente hacia el drift rate real
            _yawBiasEstimate = Mathf.Lerp(_yawBiasEstimate, driftRate, biasLearningRate);

            if (verboseCalibrationLog && Time.frameCount % 120 == 0)
                Debug.Log($"[FT:{myColorIdentity}] Bias: drift={yawDrift:F2}° " +
                          $"rate={driftRate:F3}°/s est={_yawBiasEstimate:F3}°/s");
        }

        // Renovar la referencia cada 5s para evitar problemas de precisión float
        if (_stableRestTime > 5f)
        {
            _biasReferenceRot = currentRaw;
            _stableRestTime = biasEstimationDelay; // seguir estimando
        }
    }

    // Aplica la compensación de bias a la rotación raw del sensor.
    // Solo compensa yaw porque es donde derivan los IMUs 6-ejes.
    private Quaternion CompensateBias(Quaternion rawRot)
    {
        if (!enableBiasEstimation || Mathf.Abs(_yawBiasEstimate) < 0.001f)
            return rawRot;

        // Restar el bias estimado por frame
        float correction = -_yawBiasEstimate * Time.deltaTime;
        return Quaternion.Euler(0, correction, 0) * rawRot;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Actualización de posición visual
    // ══════════════════════════════════════════════════════════════════

    // Llamado desde FusionSystemManager con la posición 3D proyectada desde depth map.
    // Aplica depthYOffset y deadzones de distancia + angular.
    // Las deadzones se desactivan cuando la mano está cerca (<12cm) o tras soltar el cubo.
    public void UpdateVisualPosition(Vector3 targetPos)
    {
        LastVisualUpdateTime = Time.time;
        LastDetectionTime = ExperimentClock.Now;
        if (_expEnabled) OnVisualDetection?.Invoke(targetPos, transform.localScale, LastDetectionTime);

        // No actualizar posición durante grip, la mano manda
        if (_isGripping) return;

        // El depth map detecta la cara superior del cubo, pero el pivot del objeto
        // está en la base. Compensamos con depthYOffset (típicamente -halfHeight).
        targetPos.y += depthYOffset;

        // ── Smart deadzone: se desactiva cuando la mano está cerca ──
        // Si la mano está acercándose al cubo, queremos que la posición se
        // actualice rápido para que el agarre sea fluido.
        bool handNear = false;
        if (trackingHand != null)
        {
            float handDist = Vector3.Distance(
                trackingHand.transform.position, transform.position);
            handNear = handDist < 0.12f;
        }

        // También se bypasea la deadzone justo después de soltar el cubo
        // (forceNextVisualUpdate) para que se reposicione inmediatamente.
        bool bypassDeadzone = _forceNextVisualUpdate || handNear;

        if (!bypassDeadzone)
        {
            // Deadzone de distancia: si la nueva posición está muy cerca de la actual,
            // es ruido del depth map, no movimiento real.
            if (Vector3.Distance(targetPos, _visualTargetPosition) < positionDeadzoneMeters)
                return;

            // Deadzone angular: si desde la perspectiva del jugador la posición nueva
            // está en casi la misma dirección, es jitter lateral del pixel.
            if (playerHead != null &&
                Vector3.Angle(targetPos - playerHead.position,
                              _visualTargetPosition - playerHead.position) < angularDeadzoneDeg)
                return;
        }

        _visualTargetPosition = targetPos;
        _forceNextVisualUpdate = false;
    }

    // ══════════════════════════════════════════════════════════════════
    //  LateUpdate — Bucle principal de fusión
    // ══════════════════════════════════════════════════════════════════

    // Se ejecuta cada frame después de Update.
    // Aquí es donde se juntan IMU + calibración + grip + posición visual.
    private void LateUpdate()
    {
        // Interpolar el offset de calibración hacia el target.
        // Esto hace que las correcciones pequeñas sean suaves en vez de saltar.
        if (Quaternion.Angle(_calibrationOffset, _targetCalibrationOffset) > 0.1f)
        {
            _calibrationOffset = Quaternion.Slerp(
                _calibrationOffset, _targetCalibrationOffset,
                Time.deltaTime * calibrationLerpSpeed);
        }
        else
        {
            _calibrationOffset = _targetCalibrationOffset;
        }
        // Nota: NO hacemos ForceYawOnly aquí. _calibrationOffset es ahora 3D completo
        // para preservar qué cara está abajo. SnapDownAxisToWorldDown en ApplyTransform
        // se encarga de aplanar para cuando está en la mesa.

        // Leer datos del sensor: rotación IMU, estado del FSR (grip/release)
        ReadSensorData(out Quaternion rawRot, out bool grip, out bool release);

        // ── Logging: datos crudos del sensor ──
        if (_expEnabled)
        {
            int rawGrip = 0;
            DeviceData dd = BLEManager.Instance?.GetDeviceByName(assignedName);
            if (dd != null) rawGrip = dd.grip;
            BackgroundDataLogger.Instance?.LogSensorData(myColorIdentity.ToString(), rawRot, rawGrip);
        }

        // Compensar bias del giroscopio antes de todo lo demás
        rawRot = CompensateBias(rawRot);

        // Contar frames consecutivos estables (IMU sin cambios grandes).
        // Esto alimenta el gate de estabilidad para la calibración.
        if (Quaternion.Angle(rawRot, _lastValidSensorRotation) < stabilityAngleThreshold)
            _stableFramesCount++;
        else
            _stableFramesCount = 0;

        // Usar un umbral de estabilidad más relajado si aún no tenemos calibración inicial
        bool nowStable = _hasInitialCalibration ? IsStable : IsStableForInitial;

        // Si acabamos de dejar de estar estables, limpiar el buffer de mediana
        // porque las muestras anteriores ya no son de fiar
        if (!nowStable && _wasStable) ClearOffsetBuffer();
        _wasStable = nowStable;

        _lastValidSensorRotation = rawRot;

        // Estimación de bias del giroscopio (se ejecuta cada frame, aunque esté desactivada
        // internamente hace return inmediato si enableBiasEstimation es false)
        UpdateBiasEstimation(rawRot);

        // Actualizar la máquina de estados del grip (FSR)
        UpdateGripState(grip, release, rawRot);

        // Aplicar la pose final: calibrationOffset * rawRot = rotación calibrada del cubo
        ApplyTransform(_calibrationOffset * rawRot);

        // ── Logging: resultado de fusión ──
        if (_expEnabled)
        {
            Vector3 downFace = transform.rotation * Vector3.down;
            BackgroundDataLogger.Instance?.LogFusionData(
                myColorIdentity.ToString(), transform.position, transform.rotation,
                _isGripping, downFace);
        }
    }

    // Lee la rotación del IMU y el estado del FSR del dispositivo BLE asignado.
    // Si no hay datos, mantiene la última rotación válida.
    private void ReadSensorData(out Quaternion rawRot, out bool grip, out bool release)
    {
        // Valores por defecto: última rotación conocida, sin grip ni release
        rawRot = _lastValidSensorRotation; grip = release = false;

        DeviceData data = BLEManager.Instance?.GetDeviceByName(assignedName);
        if (data == null) return;

        // FSR con histéresis: umbral de entrada más alto que el de salida
        // para evitar que oscile entre grip y no grip
        grip = data.grip > gripThresholdEnter;
        release = data.grip < gripThresholdExit;

        // Si el quaternion tiene w=0, el IMU aún no ha enviado datos válidos
        if (data.orientation.w != 0f)
            rawRot = Quaternion.Euler(mountingRotation) * data.orientation.normalized;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Máquina de estados del grip
    // ══════════════════════════════════════════════════════════════════

    // El grip funciona con histéresis del FSR (sensor de presión):
    //   - Entra en grip cuando FSR > gripThresholdEnter
    //   - Sale de grip cuando FSR < gripThresholdExit
    //   - Cooldown entre release y siguiente grip para evitar rebotes
    //
    // Al soltar: recaptura el offset de calibración completo (3D, no solo yaw)
    // para preservar qué cara está abajo después de girar el cubo.
    private void UpdateGripState(bool grip, bool release, Quaternion currentRaw)
    {
        if (!_isGripping)
        {
            // ── Intentar entrar en grip ──
            // Condiciones: FSR dice grip + ha pasado el cooldown + tenemos referencia de mano
            if (grip && Time.time > _lastReleaseTime + grabCooldown && trackingHand != null)
            {
                _isGripping = true;

                Debug.Log($"[FT:{myColorIdentity}] ★ GRIP START → trackingHand=\"{trackingHand.gameObject.name}\" " +
                          $"(ID={trackingHand.GetInstanceID()}) pos={trackingHand.transform.position}");

                // Hacer kinematic para que la física no interfiera durante el grip
                if (_rb != null) _rb.isKinematic = true;

                // Guardar la posición del cubo en espacio local de la mano.
                // Así cuando la mano se mueva, el cubo la sigue manteniendo
                // la misma posición relativa.
                _gripLocalPosition = trackingHand.transform.InverseTransformPoint(transform.position);

                if (_expEnabled) OnGripStarted?.Invoke(ExperimentClock.Now);
            }
        }
        else if (release)
        {
            // ── Salir del grip ──
            _isGripping = false; _lastReleaseTime = Time.time;

            // Restaurar física
            if (_rb != null) _rb.isKinematic = false;

            if (_expEnabled) OnGripEnded?.Invoke(ExperimentClock.Now);

            // Poner el cubo donde la mano lo soltó
            if (trackingHand != null)
            {
                Vector3 releasePos = trackingHand.transform.TransformPoint(_gripLocalPosition);
                _visualTargetPosition = releasePos;
                transform.position = releasePos;
            }

            // ── Recapturar offset de calibración COMPLETO ────────────
            // Fórmula: newOffset * currentRaw = visualObject.rotation
            //        → newOffset = visualObject.rotation * Inverse(currentRaw)
            // Esto captura TODO: qué cara está abajo, hacia dónde apunta.
            // Es crucial porque el usuario puede haber girado el cubo 
            // mientras lo tenía en la mano.
            Quaternion fullOffset = visualObject.rotation * Quaternion.Inverse(currentRaw);
            _calibrationOffset = _targetCalibrationOffset = fullOffset;

            // Forzar que la próxima detección visual actualice posición (sin deadzone)
            _forceNextVisualUpdate = true;

            // Armar la calibración post-release: esperamos una buena detección
            _pendingPostReleaseCalib = true;

            // Resetear el estimador de bias porque la situación ha cambiado
            _biasReferenceSet = false;
            _stableRestTime = 0f;
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Aplicación del transform final
    // ══════════════════════════════════════════════════════════════════

    // Dos modos según si el cubo está en la mano o en la mesa:
    //   - En mano: rotación 3D completa del IMU + posición de la mano
    //   - En mesa: bloqueo de cara (SnapDownAxisToWorldDown) + lerp de posición
    private void ApplyTransform(Quaternion idealRot)
    {
        if (_isGripping)
        {
            // En mano: seguir la posición de la mano y usar la rotación IMU tal cual
            if (trackingHand != null) transform.position = trackingHand.transform.TransformPoint(_gripLocalPosition);
            visualObject.rotation = idealRot; _smoothedRotation = idealRot;
        }
        else
        {
            // En mesa: bloquear la cara inferior contra el suelo y solo permitir yaw.
            // Esto evita que el cubo "flote" o se incline por ruido del IMU.
            Quaternion stableRot = SnapDownAxisToWorldDown(idealRot);

            // Interpolar posición suavemente hacia el target visual
            Vector3 target = _visualTargetPosition;
            transform.position = Vector3.Lerp(transform.position, target, Time.deltaTime * visualCorrectionSpeed);

            // Interpolar rotación suavemente, con un mínimo para evitar
            // micro-oscilaciones innecesarias
            if (Quaternion.Angle(_smoothedRotation, stableRot) > 0.05f)
                _smoothedRotation = Quaternion.Slerp(_smoothedRotation, stableRot, Time.deltaTime * rotationSmoothSpeed);
            else
                _smoothedRotation = stableRot;
            visualObject.rotation = _smoothedRotation;
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Bloqueo de cara inferior (face-down lock)
    // ══════════════════════════════════════════════════════════════════

    // Eje local que apunta hacia abajo actualmente. Se cachea para histéresis:
    // no cambiamos de cara a menos que otra sea SIGNIFICATIVAMENTE mejor.
    private Vector3 _cachedDownAxis = Vector3.down;

    // Cuando el cubo está en la mesa, queremos que una de sus 6 caras esté
    // perfectamente plana contra el suelo (sin inclinación). Solo dejamos
    // que rote en yaw (girar sobre sí mismo).
    //
    // Algoritmo:
    //   1. Comprobar cuál de los 6 ejes locales apunta más hacia abajo
    //   2. Histéresis de 0.15 para no cambiar de cara por ruido
    //   3. Calcular la rotación que pone ese eje exactamente hacia abajo
    //   4. Extraer solo el yaw del resto de la rotación
    //   5. Reconstruir: faceDown * yawOnly = rotación final estable
    private Quaternion SnapDownAxisToWorldDown(Quaternion rot)
    {
        // Los 6 ejes locales posibles que podrían estar apuntando hacia abajo
        Vector3[] localAxes = {
            Vector3.down, Vector3.up,
            Vector3.left, Vector3.right,
            Vector3.back, Vector3.forward
        };

        // ¿El eje cacheado sigue apuntando bien hacia abajo?
        Vector3 currentDown = rot * _cachedDownAxis;
        float currentDot = Vector3.Dot(currentDown, Vector3.down);

        // Solo cambiar a otro eje si es significativamente mejor (histéresis)
        // Esto evita que el cubo "parpadee" entre caras cuando está en un ángulo límite
        Vector3 bestAxis = _cachedDownAxis;
        float bestDot = currentDot;
        float hysteresis = 0.15f;

        foreach (Vector3 axis in localAxes)
        {
            Vector3 worldAxis = rot * axis;
            float dot = Vector3.Dot(worldAxis, Vector3.down);
            if (dot > bestDot + hysteresis)
            {
                bestDot = dot;
                bestAxis = axis;
            }
        }
        _cachedDownAxis = bestAxis;

        // Paso 1: rotación que pone el eje elegido exactamente hacia abajo
        Quaternion faceDownBase = Quaternion.FromToRotation(_cachedDownAxis, Vector3.down);

        // Paso 2: lo que sobra después de quitar faceDown = yaw + ruido de pitch/roll
        Quaternion remainder = Quaternion.Inverse(faceDownBase) * rot;

        // Paso 3: quedarnos solo con el yaw del residuo
        float yaw = remainder.eulerAngles.y;
        Quaternion yawOnly = Quaternion.Euler(0f, yaw, 0f);

        // Paso 4: reconstruir la rotación final = cara plana + solo giro horizontal
        return faceDownBase * yawOnly;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Reset de estado
    // ══════════════════════════════════════════════════════════════════

    // Resetea todo. Llamado al reconectar BLE o al reiniciar la sesión.
    public void ResetState()
    {
        _targetCalibrationOffset = _calibrationOffset = Quaternion.identity;
        _isGripping = false; _forceNextVisualUpdate = true;
        _hasInitialCalibration = false; _stableFramesCount = 0;
        _pendingPostReleaseCalib = false;
        _cachedDownAxis = Vector3.down;
        ClearOffsetBuffer(); _wasStable = false;
        _yawBiasEstimate = 0f; _stableRestTime = 0f; _biasReferenceSet = false;

        // Ocultar todos los debug visuals
        if (_kpSpheres != null) foreach (var s in _kpSpheres) if (s) s.SetActive(false);
        if (_arrowVisual) _arrowVisual.gameObject.SetActive(false);
        if (_arrowSensor) _arrowSensor.gameObject.SetActive(false);
    }
}