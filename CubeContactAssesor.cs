using UnityEngine;

public enum ContactLevel
{
    // Niveles acumulativos: cada valor indica evidencia mas fuerte de interaccion.
    Resting = 0,
    Nearby = 1,
    Touching = 2,
    HeldSoft = 3,
    HeldHard = 4
}

[System.Serializable]
public class CubeContactAssessor
{
    // Fusiona tacto, inercia y proximidad de mano para estimar contacto con el cubo.
    // Mantiene la interaccion cuando la vision se degrada por oclusion.

    // --- Umbrales configurables ---
    // Distancias mano-objeto usadas para distinguir cercania de contacto probable.
    [Header("Distance Thresholds (meters)")]
    public float nearbyRadius = 0.15f;
    public float touchingRadius = 0.08f;

    // Umbrales FSR separados por intensidad y liberacion para reducir parpadeos.
    [Header("FSR Thresholds")]
    public int fsrHardThreshold = 20;
    public int fsrSoftThreshold = 5;
    public int fsrReleaseThreshold = 10;

    // Movimiento IMU minimo sostenido para considerar que el cubo rota por manipulacion real.
    [Header("IMU Motion Detection")]
    [Tooltip("Cambio angular, en grados por frame, necesario para considerar que el cubo rota.")]
    public float imuMotionThresholdDeg = 0.5f;

    public int imuMotionRequiredFrames = 2;

    // Ventanas temporales para decidir si YOLO sigue fresco y si una salida de mesa mantiene agarre suave.
    [Header("Timing")]
    public float yoloStalenessSeconds = 1.0f;
    public float departureGraceSeconds = 0.4f;

    // --- Estado interno de lectura de senales ---
    // Guarda la ultima evidencia para calcular nivel actual y exponer diagnostico a otros sistemas.
    private int _imuMovingFrameCount;
    private float _lastYoloTime;
    private ContactLevel _currentLevel = ContactLevel.Resting;
    private bool _departureGraceActive;
    private float _departureGraceStart;
    private float _lastHandToYoloDist = float.MaxValue;
    private float _lastHandToCubeDist = float.MaxValue;
    private float _lastImuDeltaDeg;
    private int _lastFsrValue;
    private bool _lastHandTracked;

    public ContactLevel Level => _currentLevel;

    public bool IsIMUMoving => _imuMovingFrameCount >= imuMotionRequiredFrames;

    public bool ShouldFilterAsHandFP
    {
        // Nearby suele indicar que YOLO podria estar viendo piel o mano en lugar del objeto.
        get { return _currentLevel == ContactLevel.Nearby; }
    }

    public void UpdateSignals(
        int fsrValue, float imuAngularDeltaDeg, Vector3 handPosition,
        bool handTracked, Vector3 cubeTrackedPosition, Vector3 yoloDetectionPosition,
        bool hasYoloThisFrame, float currentTime)
    {
        // Combina FSR, movimiento IMU y proximidad de mano para separar contacto real
        // de falsos positivos visuales cuando la mano cubre o roza el cubo.
        _lastFsrValue = fsrValue;
        _lastImuDeltaDeg = imuAngularDeltaDeg;
        _lastHandTracked = handTracked;

        if (hasYoloThisFrame)
            _lastYoloTime = currentTime;

        if (handTracked && hasYoloThisFrame)
            _lastHandToYoloDist = Vector3.Distance(handPosition, yoloDetectionPosition);

        if (handTracked)
            _lastHandToCubeDist = Vector3.Distance(handPosition, cubeTrackedPosition);

        if (imuAngularDeltaDeg >= imuMotionThresholdDeg)
            _imuMovingFrameCount = Mathf.Min(_imuMovingFrameCount + 1, imuMotionRequiredFrames + 10);
        else
            _imuMovingFrameCount = Mathf.Max(_imuMovingFrameCount - 1, 0);

        bool imuMoving = IsIMUMoving;
        bool yoloFresh = (currentTime - _lastYoloTime) < yoloStalenessSeconds;
        // La frescura evita confiar en posiciones visuales antiguas durante agarre o salida de mesa.
        bool fsrHard = fsrValue >= fsrHardThreshold;
        bool fsrSoft = fsrValue >= fsrSoftThreshold && !fsrHard;

        bool handNearCube = handTracked && _lastHandToCubeDist < nearbyRadius;
        bool handNearYolo = handTracked && yoloFresh && _lastHandToYoloDist < nearbyRadius;
        bool handNear = handNearCube || handNearYolo;
        bool handOnYolo = handTracked && yoloFresh && _lastHandToYoloDist < touchingRadius;

        // Un FSR fuerte gana siempre porque es la evidencia mas directa de agarre real.
        if (fsrHard && handTracked)
        {
            _currentLevel = ContactLevel.HeldHard;
            return;
        }

        if (handNearCube && imuMoving && handTracked)
        {
            // Al levantar el cubo, YOLO puede perderlo unos frames; la gracia conserva el agarre suave.
            _departureGraceActive = true;
            _departureGraceStart = currentTime;
        }
        if (!imuMoving || (currentTime - _departureGraceStart) > departureGraceSeconds)
            _departureGraceActive = false;

        bool heldSoft = false;
        // HeldSoft representa agarre probable sin FSR fuerte: varias senales debiles pesan juntas.
        // Cubre casos clinicos donde el paciente sujeta sin presionar justo el sensor.
        if (handOnYolo && imuMoving) heldSoft = true;
        if (handNear && fsrSoft && imuMoving) heldSoft = true;
        if (handOnYolo && fsrSoft) heldSoft = true;
        if ((handNearCube || _departureGraceActive) && imuMoving && handTracked)
            heldSoft = true;

        if (heldSoft)
        {
            _currentLevel = ContactLevel.HeldSoft;
            return;
        }

        if (handOnYolo)
        {
            _currentLevel = ContactLevel.Touching;
            return;
        }

        if (handNear)
        {
            // Nearby no ancla el cubo, pero ayuda a filtrar detecciones de mano confundidas con objeto.
            _currentLevel = ContactLevel.Nearby;
            return;
        }

        _currentLevel = ContactLevel.Resting;
    }

    public void Reset()
    {
        // Limpia todo el historial de contacto al reiniciar tracker, ensayo o juego.
        _currentLevel = ContactLevel.Resting;
        _imuMovingFrameCount = 0;
        _lastHandToYoloDist = float.MaxValue;
        _lastHandToCubeDist = float.MaxValue;
        _lastImuDeltaDeg = 0f;
        _lastFsrValue = 0;
        _lastHandTracked = false;
        _lastYoloTime = -10f;
        _departureGraceActive = false;
        _departureGraceStart = -10f;
    }
}
