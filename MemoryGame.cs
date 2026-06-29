using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.XR.Oculus;

public class MemoryGame : MonoBehaviour
{
    // Juego serio de rehabilitacion. Convierte el sistema de tracking en una tarea repetitiva
    // con feedback inmediato y datos clinicos medibles.
    // Calibra el alcance, crea el area sobre la mesa y pide colocar cubos por color.
    // Mapa mental para leer este archivo:
    // 1) Setup/calibracion: captura tres puntos de mano y calcula mesa/area.
    // 2) Partida: activa fusion/vision, muestra plataformas y puntua colocaciones.
    // 3) Cierre: apaga deteccion, VFX, corutinas y logging de sesion.
    //
    // Unity llama automaticamente a Awake, Start y Update. Los botones del menu llaman
    // a metodos publicos como BeginCalibrationRightHand o RestartGame.
    public static MemoryGame Instance { get; private set; }

    // --- Referencias de escena configuradas en Inspector ---
    // Manos OVR que se usan para calibrar alcance y registrar movimiento del paciente.
    [Header("Hands")]
    public OVRHand rightHand;
    public OVRHand leftHand;

    // Mirada opcional. Si no hay eye tracking fiable, el logger usa la direccion de la cabeza.
    [Header("Eye Tracking")]
    public OVREyeGaze eyeGaze;

    // Parametros de captura de puntos: cuanto tiempo mantener la mano quieta y como validar separacion.
    [Header("Calibration")]
    public float holdTime = 4.25f;
    public float stillnessThreshold = 0.04f;
    public float minPointSeparation = 0.15f;

    // La calibracion por alcance adapta el espacio al paciente y a la mesa real.
    // platformPrefab es el objetivo que aparece en mesa; los materiales indican el color esperado.
    [Header("Platform")]
    public GameObject platformPrefab;

    [Tooltip("Mitad del grosor de la plataforma para que no se hunda en la mesa.")]
    public float platformHeightOffset = 0.02f;

    [Tooltip("Correccion manual de la altura de mesa. Positivo sube plataformas y cubos bloqueados.")]
    public float tableHeightCorrection = 0.0f;

    [Tooltip("Correccion automatica estimada con el cubo apoyado en la mesa.")]
    public float autoTableHeightCorrection = 0.0f;

    public Material matRed;
    public Material matGreen;
    public Material matBlue;

    // --- Reglas principales del minijuego ---
    // Controlan longitud de partida, dificultad temporal y separacion entre objetivos.
    [Header("Chase Settings")]
    public int targetPerCycle = 10;
    public int totalRounds = 5;

    [Tooltip("Segundos disponibles en la primera plataforma.")]
    public int initialCountdownSeconds = 20;

    [Tooltip("Minimo de segundos por plataforma.")]
    public int minCountdownSeconds = 2;

    [Tooltip("Segundos restados al contador despues de cada acierto.")]
    public int countdownDecrementStep = 1;

    // Reducir el tiempo convierte una tarea simple en entrenamiento progresivo.
    [Tooltip("Pausa entre un acierto y la aparicion de la siguiente plataforma, en segundos.")]
    public float pauseBetweenPlatforms = 0.6f;

    public float minPlatformSeparation = 0.15f;

    // Efecto visual opcional entre plataformas completadas para reforzar el feedback de acierto.
    [Header("Trail Orb")]
    [Tooltip("Prefab de orbe que viaja de la plataforma completada a la siguiente.")]
    public GameObject orbVFXPrefab;

    [Tooltip("Duracion del viaje del orbe, en segundos.")]
    public float orbTravelDuration = 0.75f;

    // Puntuacion basica: acierto, fallo y recompensa por encadenar aciertos.
    [Header("Scoring")]
    public int pointsPerCorrect = 100;
    public int penaltyPerIncorrect = 25;
    public int comboBonus = 50;

    // Textos de HUD que se actualizan durante la partida. Pueden quedar vacios en escenas de prueba.
    [Header("HUD")]
    public TMP_Text lblTime;
    public TMP_Text lblHits;
    public TMP_Text lblMisses;
    public TMP_Text lblCoins;

    // Tutorial y menus externos. MemoryGame los abre/cierra, pero no implementa su contenido.
    [Header("Tutorial")]
    public TutorialMediaManager tutorialMedia;

    [Header("Menus")]
    public GameObject settingsCanvas;
    public GameObject tutorialCanvas;
    [Tooltip("Toggle opcional que abre ajustes. Si se deja vacio, se busca AJUSTES/Settings en runtime.")]
    public Toggle settingsToggle;

    // Audio de feedback. Cada AudioSource puede tener clip propio; el script solo dispara el momento correcto.
    [Header("Audio")]
    public AudioSource backgroundMusic;
    public AudioSource correctSound;
    public AudioSource incorrectSound;
    public AudioSource roundCompleteSound;
    public AudioSource gameOverSound;
    public AudioSource calibrationCompleteSound;
    [Tooltip("Clip opcional al capturar un punto. Si esta vacio, usa el clip de calibrationCompleteSound.")]
    public AudioClip calibrationCompleteClip;
    [Range(0f, 1f)]
    public float calibrationCompleteVolume = 0.7f;

    // Prefabs visuales para remarcar aparicion de cubo/plataforma o fin de calibracion.
    [Header("VFX")]
    public GameObject cubeSpawnVFXPrefab;
    public GameObject calibrationCompleteVFXPrefab;

    // Raiz que contiene sistemas de deteccion/fusion. Se activa al jugar y se apaga en menus.
    [Header("Detection")]
    public GameObject detectionRoot;

    // --- Estado publico de partida ---
    // Son propiedades de solo lectura: otros scripts pueden consultar puntuacion/fase, pero no sobrescribirlas.
    public bool IsGameActive { get; private set; }

    public bool CanInteract { get; private set; }

    public int Score { get; private set; }

    public int CurrentRound { get; private set; }

    public int CurrentCombo { get; private set; }

    public int BestCombo { get; private set; }

    public int TotalHits { get; private set; }

    public int TotalMisses { get; private set; }

    public event Action OnGameStarted;
    public event Action OnGameStopped;

    // --- Estado interno del flujo ---
    // Phase es el modo global del juego; CalStep es el paso concreto dentro de la calibracion.
    private enum Phase { SetupMenu, Calibrating, Starting, Playing, GameOver }
    private enum CalStep { WaitHand, Right, Center, Left, Done }

    // La maquina de fases evita que calibracion, juego y cierre se pisen entre si.
    private Phase _phase = Phase.SetupMenu;
    private CalStep _calStep = CalStep.WaitHand;
    private OVRHand _activeHand;
    private bool _useRightHand = true;

    // --- Estado interno de calibracion ---
    // Los tres puntos capturados definen anchura/profundidad del area de trabajo sobre la mesa.
    private Vector3 _ptRight, _ptCenter, _ptLeft;
    private Vector3 _holdAnchor;
    private float _holdTimer;
    private bool _holdStarted;
    private bool _calibrationStepTransitioning;
    private GameObject _previewSphere;

    // Datos derivados de calibracion: definen el plano de juego en coordenadas reales.
    // Se calculan una vez y luego se reutilizan para colocar plataformas y alinear FusionTracker.
    private float _tableHeight;
    private Vector3 _playCenter;
    private float _playHalfWidth;
    private float _playHalfDepth;
    private Quaternion _playRotation;

    // --- Estado interno de partida ---
    // Instancia reutilizada de plataforma, rutina principal y cronometro acumulado.
    private GameObject _platformInstance;
    private ColorPlatform _platformCP;
    private Coroutine _chaseRoutine;
    private float _elapsedTime;

    // Estos flags separan "la plataforma esta esperando" de "ya llego el cubo correcto".
    // ReportPlacement cambia ambos desde un trigger externo, mientras ChaseGameLoop los consume.
    private bool _waitingForPlacement;
    private bool _correctPlaced;

    // Trigger de mesa y lineas/trails creados durante la partida para feedback visual.
    private TableSurfaceTrigger _tableTrigger;
    private readonly List<GameObject> _trailLines = new();

    // Componentes auxiliares creados o resueltos en runtime para UI y audio.
    private TextMeshPro _floatingTimerText;
    private AudioSource _calibrationCompletePlayer;
    private MemoryGameUiBinder _uiBinder;

    private void Awake()
    {
        // El juego coordina la escena; otros triggers usan Instance para reportar aciertos.
        // Singleton: solo debe existir un MemoryGame activo para evitar puntuaciones duplicadas.
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        ResolveLegacyAudioSources();
        if (detectionRoot != null) detectionRoot.SetActive(false);
        ConfigureOneShotSource(correctSound);
        ConfigureOneShotSource(incorrectSound);
        ConfigureOneShotSource(roundCompleteSound);
        ConfigureOneShotSource(gameOverSound);
        ConfigureOneShotSource(calibrationCompleteSound);
    }

    private void Start()
    {
        // Sube prioridad de rendimiento porque la escena combina MR, vision, particulas y UI.
        OVRPlugin.foveatedRenderingLevel = OVRPlugin.FoveatedRenderingLevel.HighTop;
        OVRPlugin.useDynamicFoveatedRendering = true;
        OVRPlugin.suggestedGpuPerfLevel = OVRPlugin.ProcessorPerformanceLevel.Boost;
        OVRPlugin.suggestedCpuPerfLevel = OVRPlugin.ProcessorPerformanceLevel.SustainedHigh;

        CreatePreviewSphere();
        CreateFloatingTimer();
        UiBinder.WireSettingsToggle();
        UiBinder.WireRestartControl();
        SetSettingsCanvasVisible(false);
        SetText("Select in the main menu which hand you want to calibrate with.");
        SetProgress(0f);
        UpdateHUD(reset: true);

        if (tutorialMedia != null) tutorialMedia.PlayMenu();
    }

    private void Update()
    {
        // Mantiene UI y flujo principal sincronizados con el estado actual del juego.
        UiBinder.SyncSettingsToggleToCanvas();

        // Durante juego, registra cabeza/manos en paralelo al flujo del minijuego.
        if (_phase == Phase.Calibrating)
            UpdateCalibration();

        if (_phase == Phase.Playing && IsGameActive)
        {
            _elapsedTime += Time.deltaTime;
            if (lblTime != null) lblTime.text = FormatTime(_elapsedTime);

            if (BackgroundDataLogger.Instance != null)
            {
                Transform head = Camera.main != null ? Camera.main.transform : null;
                Transform rHand = rightHand != null ? rightHand.transform : null;
                Transform lHand = leftHand != null ? leftHand.transform : null;

                Vector3 currentGazeDir = Vector3.zero;
                if (eyeGaze != null && eyeGaze.EyeTrackingEnabled && eyeGaze.Confidence > 0.5f)
                {
                    currentGazeDir = eyeGaze.transform.forward;
                }
                else if (head != null)
                {
                    currentGazeDir = head.forward;
                }

                BackgroundDataLogger.Instance.LogKinematicsData(head, rHand, lHand, currentGazeDir);
            }
        }
    }

    // Punto preparado para conectar mensajes a UI externa sin acoplar el script a un widget concreto.
    private void SetText(string text) { }

    // Punto preparado para conectar progreso visual de calibracion o contador.
    private void SetProgress(float v) { }

    private static void PlaySound(AudioSource src)
    {
        // Reproduce un AudioSource como one-shot si tiene clip, o directo si no.
        if (src == null) return;
        if (src.clip != null) src.PlayOneShot(src.clip);
        else src.Play();
    }

    // Formatea segundos acumulados como mm:ss para el HUD.
    private static string FormatTime(float seconds)
    {
        int total = Mathf.Max(0, Mathf.FloorToInt(seconds));
        return $"{total / 60:00}:{total % 60:00}";
    }

    private MemoryGameUiBinder UiBinder => _uiBinder ??= new MemoryGameUiBinder(this);

    public void SetSettingsCanvasVisible(bool visible)
    {
        // Wrapper publico para eventos de Unity: la logica de enlace vive en MemoryGameUiBinder.
        UiBinder.SetSettingsCanvasVisible(visible);
    }

    // Abre ajustes desde botones de UI.
    public void OpenSettingsCanvas() => SetSettingsCanvasVisible(true);

    // Cierra ajustes desde botones de UI.
    public void CloseSettingsCanvas() => SetSettingsCanvasVisible(false);

    public void ToggleSettingsCanvas()
    {
        // Invierte la visibilidad actual de ajustes.
        UiBinder.ToggleSettingsCanvas();
    }

    private void UpdateHUD(bool reset = false)
    {
        // Actualiza el HUD de forma defensiva porque algunas escenas no conectan todos los textos.
        if (reset)
        {
            if (lblCoins != null) lblCoins.text = "0";
            if (lblHits != null) lblHits.text = "0";
            if (lblMisses != null) lblMisses.text = "0";
            if (lblTime != null) lblTime.text = "00:00";
        }
        else
        {
            if (lblCoins != null) lblCoins.text = Score.ToString();
            if (lblHits != null) lblHits.text = TotalHits.ToString();
            if (lblMisses != null) lblMisses.text = TotalMisses.ToString();
        }
    }

    // Inicia calibracion usando la mano izquierda.
    public void BeginCalibrationLeftHand() => StartCalibrationFlow(useRightHand: false);

    // Inicia calibracion usando la mano derecha.
    public void BeginCalibrationRightHand() => StartCalibrationFlow(useRightHand: true);

    public void RestartGame()
    {
        // Reiniciar sin calibracion no tiene sentido: el juego necesita area, altura y rotacion de mesa.
        if (_phase == Phase.Starting) return;
        if (_ptCenter == Vector3.zero)
        {
            SetText("<color=#FF6060>Calibrate first before restarting.</color>");
            return;
        }
        if (IsGameActive) StopGame(isWin: false);
        StartGame();
    }

    private void StartCalibrationFlow(bool useRightHand)
    {
        // Al recalibrar, limpia temporales para que el area no herede restos de rondas anteriores.
        // La calibracion define el espacio terapeutico. Sin ella no se conoce mesa, centro ni alcance.
        if (_phase == Phase.Playing || _phase == Phase.Starting) return;
        if (_platformInstance != null) { Destroy(_platformInstance); _platformInstance = null; }
        if (_tableTrigger != null) { Destroy(_tableTrigger.gameObject); _tableTrigger = null; }
        ClearTrailLines();
        HideFloatingTimer();

        _useRightHand = useRightHand;
        _activeHand = useRightHand ? rightHand : leftHand;
        _phase = Phase.Calibrating;
        _calStep = CalStep.WaitHand;
        _calibrationStepTransitioning = false;

        SetText(CalibrationMessage());
        SetProgress(0f);
    }

    private void UpdateCalibration()
    {
        // Cada punto se acepta solo si la mano queda quieta; asi el area refleja alcance real.
        // El usuario captura derecha, centro e izquierda; esos tres puntos construyen el plano de juego.
        if (_calStep == CalStep.Done) return;
        if (_calibrationStepTransitioning) return;
        if (_calStep == CalStep.WaitHand) { if (_activeHand != null && _activeHand.IsTracked && _activeHand.IsDataHighConfidence) StartNextCalibrationStep(); return; }

        bool tutorialAudioPlaying = IsTutorialAudioPlaying();
        // La vista previa sigue la mano siempre, pero solo crece/progresa cuando el tutorial ya no habla.
        if (!TryUpdatePreviewAtActiveHand(allowGrowth: !tutorialAudioPlaying, out Vector3 pos))
        {
            _holdStarted = false;
            _holdTimer = 0f;
            return;
        }

        if (tutorialAudioPlaying) { HoldCalibrationUntilAudioEnds(); return; }

        if (!_holdStarted) { _holdAnchor = pos; _holdStarted = true; _holdTimer = 0f; return; }
        if (Vector3.Distance(pos, _holdAnchor) > stillnessThreshold) { _holdAnchor = pos; _holdTimer = 0f; SetText(CalibrationMessage()); SetProgress(0f); return; }

        _holdTimer += Time.deltaTime;
        SetProgress(Mathf.Clamp01(_holdTimer / holdTime));
        SetText(CalibrationMessage() + $"\n\n<size=80%><color=#FFD060>Hold still... {holdTime - _holdTimer:F1}s</color></size>");

        if (_holdTimer >= holdTime) CaptureCalibrationPoint(pos);
    }

    private void StartNextCalibrationStep()
    {
        // El primer punto real siempre es derecha; WaitHand solo confirma tracking de la mano elegida.
        _calStep = CalStep.Right;
        _calibrationStepTransitioning = false;
        _holdStarted = false;
        _holdTimer = 0f;
        SetText(CalibrationMessage());

        if (tutorialMedia != null) tutorialMedia.PlayCalibrateRight();
        TryUpdatePreviewAtActiveHand(allowGrowth: false, out _);
    }

    private void CaptureCalibrationPoint(Vector3 pos)
    {
        // Guarda derecha, centro e izquierda, validando separacion minima entre puntos.
        if (_calibrationStepTransitioning) return;

        // Derecha, centro e izquierda definen anchura, profundidad y centro del espacio del paciente.
        string stepName = _calStep.ToString();
        switch (_calStep)
        {
            case CalStep.Right:
                _ptRight = pos;
                BeginCalibrationPointTransition();
                StartCoroutine(TransitionToNextCalStep(CalStep.Center, pos));
                break;
            case CalStep.Center:
                if (Vector3.Distance(pos, _ptRight) < minPointSeparation) { RejectCalibrationPoint(stepName); return; }
                _ptCenter = pos;
                BeginCalibrationPointTransition();
                StartCoroutine(TransitionToNextCalStep(CalStep.Left, pos));
                break;
            case CalStep.Left:
                if (Vector3.Distance(pos, _ptCenter) < minPointSeparation || Vector3.Distance(pos, _ptRight) < minPointSeparation) { RejectCalibrationPoint(stepName); return; }
                _ptLeft = pos;
                BeginCalibrationPointTransition();
                StartCoroutine(FinishCalibration(pos));
                break;
        }
    }

    private void BeginCalibrationPointTransition()
    {
        // Bloquea nuevas capturas mientras se reproduce feedback del punto aceptado.
        _calibrationStepTransitioning = true;
        _holdStarted = false;
        _holdTimer = 0f;
        HideCalibrationPreview();
    }

    // Rechaza un punto demasiado cercano y reinicia el temporizador de quietud.
    private void RejectCalibrationPoint(string stepName)
    {
        SetText(CalibrationMessage() + "\n\n<size=80%><color=#FF6060>Too close. Move your hand farther away.</color></size>");
        _holdStarted = false;
        _holdTimer = 0f;
    }

    private IEnumerator TransitionToNextCalStep(CalStep next, Vector3 pos)
    {
        // Pausa breve de feedback para que el usuario note que el punto fue aceptado.
        float feedbackDuration = PlayCalibrationCompleteSound();
        SpawnVFX(calibrationCompleteVFXPrefab, pos, 2f);

        SetText(CalibrationMessage() + "\n\n<size=80%><color=#60FF90>Point captured!</color></size>");
        SetProgress(1f);
        yield return new WaitForSeconds(Mathf.Max(0.7f, feedbackDuration));
        _calStep = next;
        _holdStarted = false;
        _holdTimer = 0f;
        SetText(CalibrationMessage());
        SetProgress(0f);

        if (tutorialMedia != null)
        {
            if (next == CalStep.Center) tutorialMedia.PlayCalibrateCenter();
            else if (next == CalStep.Left) tutorialMedia.PlayCalibrateLeft();
        }

        _calibrationStepTransitioning = false;
        TryUpdatePreviewAtActiveHand(allowGrowth: false, out _);
    }

    private IEnumerator FinishCalibration(Vector3 pos)
    {
        // La calibracion termina creando el area de juego y entrando en una cuenta atras.
        _calStep = CalStep.Done;

        float feedbackDuration = PlayCalibrationCompleteSound();
        SpawnVFX(calibrationCompleteVFXPrefab, pos, 2f);

        SetText("<color=#60FF90>Point captured!</color>");
        SetProgress(1f);
        yield return new WaitForSeconds(Mathf.Max(0.5f, feedbackDuration));
        SetupPlayArea();
        HideCalibrationPreview();
        _calibrationStepTransitioning = false;
        StartCoroutine(CountdownToStart());
    }

    private bool IsTutorialAudioPlaying()
    {
        // Evita capturar puntos mientras la instruccion de audio sigue hablando.
        return tutorialMedia != null && tutorialMedia.IsAudioPlaying;
    }

    private void HoldCalibrationUntilAudioEnds()
    {
        // Mientras suena el tutorial, se muestra vista previa pero no progresa la captura.
        _holdStarted = false;
        _holdTimer = 0f;
        SetProgress(0f);
        SetText(CalibrationMessage() + "\n\n<size=80%><color=#FFD060>Listen to the instruction first...</color></size>");
    }

    private bool TryUpdatePreviewAtActiveHand(bool allowGrowth, out Vector3 pos)
    {
        // Actualiza la esfera de vista previa en la mano activa si el tracking esta disponible.
        pos = Vector3.zero;
        if (_activeHand == null || !_activeHand.IsTracked)
        {
            HideCalibrationPreview();
            return false;
        }

        pos = _activeHand.transform.position;
        UpdatePreviewSphere(pos, allowGrowth);
        return true;
    }

    // Devuelve el mensaje visible correspondiente al paso de calibracion actual.
    private string CalibrationMessage() => _calStep switch
    {
        CalStep.WaitHand => "Raise the selected hand in front of you so it can be detected",
        CalStep.Right => $"Extend your hand as far as possible\nto the <b>RIGHT</b> and hold for {holdTime} seconds",
        CalStep.Center => $"Now extend it to the <b>CENTER</b>\n(in front of you) and hold for {holdTime} seconds",
        CalStep.Left => $"Now extend it to the <b>LEFT</b>\nas far as possible and hold for {holdTime} seconds",
        _ => ""
    };

    private void SetupPlayArea()
    {
        // La altura final prioriza mesa real detectada; si falla, usa altura corregida de la mano.
        float detectedTableY = DetectTableHeight(_ptCenter);

        // Se acepta MRUK/raycast solo si la mesa detectada esta cerca de la mano calibrada.
        // Esto evita que una superficie equivocada muy alta/baja rompa el area de juego.
        if (!float.IsNaN(detectedTableY) && Mathf.Abs(detectedTableY - _ptCenter.y) < 0.20f)
        {
            _tableHeight = detectedTableY;
            Debug.Log($"[MemoryGame] Using detected table: tableY={_tableHeight:F3}");
        }
        else
        {
            _tableHeight = _ptCenter.y - 0.08f;
            Debug.LogWarning(
                $"[MemoryGame] Invalid MRUK/Raycast. Using corrected calibrated hand height: ptCenter.y={_ptCenter.y:F3}, " +
                $"tableY={_tableHeight:F3}, detectedTableY={detectedTableY:F3}");
        }

        _playRotation = Camera.main != null
            ? Quaternion.Euler(0, Camera.main.transform.eulerAngles.y, 0)
            : Quaternion.identity;

        // El area se orienta desde la vista del usuario para que derecha/izquierda coincidan.
        _playCenter = new Vector3(_ptCenter.x, PlatformWorldY, _ptCenter.z);

        // La distancia derecha-izquierda se convierte en ancho y profundidad de juego.
        // Profundidad menor que anchura: la tarea queda alcanzable sobre mesa.
        float width = Vector3.Distance(
            new Vector3(_ptRight.x, 0, _ptRight.z),
            new Vector3(_ptLeft.x, 0, _ptLeft.z)
        );

        _playHalfWidth = width * 0.45f;
        _playHalfDepth = width * 0.25f;

        Debug.Log(
            $"[MemoryGame] ptCenter.y={_ptCenter.y:F3}, tableY={_tableHeight:F3}, " +
            $"platformY={PlatformWorldY:F3}, platformOffset={platformHeightOffset:F3}, " +
            $"correction={tableHeightCorrection:F3}");
    }

    // Altura final de plataforma sumando mesa, offset fisico y correcciones.
    private float PlatformWorldY => _tableHeight + platformHeightOffset + tableHeightCorrection + autoTableHeightCorrection;

    private Vector3 RandomSpawnPosition(Vector3 avoidPos, int maxAttempts = 30)
    {
        // Reintenta para que la siguiente plataforma no caiga encima de la anterior.
        for (int i = 0; i < maxAttempts; i++)
        {
            float rx = UnityEngine.Random.Range(-_playHalfWidth, _playHalfWidth);
            float rz = UnityEngine.Random.Range(-_playHalfDepth, _playHalfDepth);
            Vector3 candidate = _playCenter + _playRotation * new Vector3(rx, 0, rz);
            candidate.y = PlatformWorldY;
            if (Vector3.Distance(candidate, avoidPos) >= minPlatformSeparation) return candidate;
        }
        Vector3 fallback = _playCenter + _playRotation * (Vector3.right * (_playHalfWidth * 0.5f));
        fallback.y = PlatformWorldY;
        return fallback;
    }

    private static void CopyAudioSettings(AudioSource from, AudioSource to)
    {
        // Copia mezcla y parametros espaciales a un AudioSource creado en runtime.
        if (from == null || to == null) return;

        to.outputAudioMixerGroup = from.outputAudioMixerGroup;
        to.volume = from.volume;
        to.pitch = from.pitch;
        to.priority = from.priority;
        to.spatialBlend = from.spatialBlend;
        to.panStereo = from.panStereo;
        to.dopplerLevel = from.dopplerLevel;
        to.rolloffMode = from.rolloffMode;
        to.minDistance = from.minDistance;
        to.maxDistance = from.maxDistance;
    }

    private static float DetectTableHeight(Vector3 worldPos)
    {
        // Primero reutiliza la mesa de fusion; despues prueba MRUK y finalmente raycast fisico.
        FusionTracker[] trackers = FindObjectsByType<FusionTracker>(FindObjectsSortMode.None);
        foreach (var tracker in trackers)
        {
            if (tracker.lockToTable)
            {
                // Si fusion ya calibro mesa, se reutiliza para que juego y tracking hablen del mismo plano.
                Debug.Log($"[MemoryGame] Table taken from FusionTracker: y={tracker.tableWorldHeightY:F3}");
                return tracker.tableWorldHeightY;
            }
        }

        if (MRUKTableHeightUtility.TryGetHighestTableTop(
                out float maxTableTop,
                out _,
                true,
                "MemoryGame/MRUK"))
        {
            Debug.Log($"[MemoryGame] Table detected by MRUK: y={maxTableTop:F3}");
            return maxTableTop;
        }

        Vector3 rayOrigin = worldPos + Vector3.up * 0.3f;
        RaycastHit[] hits = Physics.RaycastAll(rayOrigin, Vector3.down, 1.5f);
        float bestY = float.NaN;

        // Filtra manos y rig para no confundir colliders de tracking con la mesa.
        // Si hay varias superficies, se usa la mas alta porque suele ser la tapa de la mesa.
        foreach (var hit in hits)
        {
            if (hit.collider.isTrigger || hit.normal.y < 0.7f) continue;

            string objName = hit.collider.gameObject.name.ToLower();
            string rootName = hit.collider.transform.root.name.ToLower();
            if (objName.Contains("hand") || rootName.Contains("hand") || rootName.Contains("ovr"))
            {
                continue;
            }

            if (float.IsNaN(bestY) || hit.point.y > bestY) bestY = hit.point.y;
        }

        if (!float.IsNaN(bestY))
        {
            Debug.Log($"[MemoryGame] Table detected by Raycast: y={bestY:F3}");
            return bestY;
        }

        Debug.LogWarning($"[MemoryGame] MRUK and Raycast did not detect a table. Using center point height: y={worldPos.y:F3}");
        return worldPos.y;
    }

    private IEnumerator CountdownToStart()
    {
        // Cuenta atras corta entre calibracion y partida.
        // IEnumerator + yield permite esperar segundos sin congelar Unity.
        _phase = Phase.Starting;

        for (int i = 5; i > 0; i--)
        {
            SetText($"<color=#60FF90>Play area ready!</color>\nThe game starts in... <b>{i}</b>");
            yield return new WaitForSeconds(1f);
        }

        StartGame();
    }

    private void StartGame()
    {
        // Al iniciar, oculta menus, abre logging y activa deteccion/fusion con la mano calibrada.
        // Desde aqui MemoryGame manda la sesion; FusionSystemManager se encarga de vision/BLE.
        _phase = Phase.Playing;
        Score = 0; CurrentRound = 1; CurrentCombo = 0; BestCombo = 0;
        TotalHits = 0; TotalMisses = 0; _elapsedTime = 0f;
        IsGameActive = true; CanInteract = false;

        SetSettingsCanvasVisible(false);
        if (settingsCanvas != null) settingsCanvas.SetActive(false);
        if (tutorialCanvas != null) tutorialCanvas.SetActive(false);

        if (backgroundMusic != null) backgroundMusic.Play();

        if (BackgroundDataLogger.Instance != null)
        {
            // El logger recibe mano y ronda para cruzar rendimiento con lateralidad.
            // La etiqueta RH/LH deja claro con que mano se jugo al abrir los CSV.
            BackgroundDataLogger.Instance.StartLogging(
                $"R{CurrentRound}_{(_useRightHand ? "RH" : "LH")}");
            BackgroundDataLogger.Instance.LogGameEvent("GameStart",
                $"hand={(_useRightHand ? "Right" : "Left")} rounds={totalRounds} targets={targetPerCycle}");
        }

        UpdateHUD();
        if (detectionRoot != null)
        {
            // detectionRoot contiene los sistemas de BLE/vision/fusion que no hacen falta en menu.
            var fusion = FindAnyObjectByType<FusionSystemManager>();
            if (fusion != null) fusion.SetActiveHand(_useRightHand);

            detectionRoot.SetActive(true);
            StartCoroutine(StartDetectionAndChase());
        }
    }

    private IEnumerator StartDetectionAndChase()
    {
        // Espera brevemente a que BLE/YOLO creen trackers antes de bloquear altura de mesa.
        // El orden importa: primero arranca fusion, despues se fijan cubos a la mesa calibrada.
        yield return null;
        var fusion = FindAnyObjectByType<FusionSystemManager>(); if (fusion != null) fusion.StartGame();
        yield return new WaitForSeconds(1.0f);

        FusionTracker[] cubes = FindObjectsByType<FusionTracker>(FindObjectsSortMode.None);
        foreach (var cube in cubes)
        {
            // Durante la partida, todos los cubos comparten el plano de mesa calibrado.
            // Esto evita que cada tracker use una altura distinta al colocar sobre plataformas.
            cube.tableWorldHeightY = _tableHeight;
            cube.tableHeightCorrection = tableHeightCorrection;
            cube.autoTableHeightCorrection = autoTableHeightCorrection;
            cube.lockToTable = true;
            Debug.Log(
                $"[MemoryGame] Cube locked to table: {cube.name}, tableY={cube.tableWorldHeightY:F3}, " +
                $"correction={cube.tableHeightCorrection:F3}, autoCorrection={cube.autoTableHeightCorrection:F3}, " +
                $"lock={cube.lockToTable}");

            if (cubeSpawnVFXPrefab != null)
            {
                Vector3 vfxPos = cube.transform.position + Vector3.down * 0.02f;
                GameObject cubeVfx = Instantiate(cubeSpawnVFXPrefab, vfxPos, cube.transform.rotation, cube.transform);
                cubeVfx.transform.localScale = Vector3.one;
                Destroy(cubeVfx, 4f);
            }
        }

        if (_tableTrigger != null) Destroy(_tableTrigger.gameObject);
        // El trigger de mesa no puntua: solo genera feedback cuando un cubo vuelve a la superficie.
        _tableTrigger = TableSurfaceTrigger.Create(_playCenter, _playHalfWidth, _playHalfDepth, _playRotation, cubeSpawnVFXPrefab);

        // Desde aqui el tracking esta activo y el juego puede pedir colocaciones.
        CreatePlatformInstance();
        _chaseRoutine = StartCoroutine(ChaseGameLoop());
    }

    public void StopGame(bool isWin = false)
    {
        // Punto unico de cierre: detiene corutinas, UI, deteccion y archivos de sesion.
        if (!IsGameActive) return;
        IsGameActive = false; CanInteract = false;

        if (backgroundMusic != null) backgroundMusic.Stop();

        if (_chaseRoutine != null) { StopCoroutine(_chaseRoutine); _chaseRoutine = null; }
        _phase = Phase.GameOver;
        PlaySound(gameOverSound);

        if (tutorialMedia != null) tutorialMedia.StopAllMedia();

        string title = isWin ? "<color=#60FF90>Game Completed!</color>" : "<color=#FF6060>Game Over!</color>";
        SetText($"<b>{title}</b>\n\nFinal score: {Score}\nBest combo: {BestCombo}\n\n<size=80%>Choose a hand to play again</size>");

        if (_platformInstance != null) _platformInstance.SetActive(false);
        ClearTrailLines(); HideFloatingTimer();
        if (_tableTrigger != null) { Destroy(_tableTrigger.gameObject); _tableTrigger = null; }

        if (detectionRoot != null)
        {
            // Se apaga inferencia/vision al terminar para ahorrar rendimiento y evitar datos fuera de partida.
            var fusionMgr = FindAnyObjectByType<FusionSystemManager>();
            if (fusionMgr != null) fusionMgr.StopGame();
        }

        if (BackgroundDataLogger.Instance != null)
        {
            BackgroundDataLogger.Instance.LogGameEvent("GameOver",
                $"win={isWin} elapsed={FormatTime(_elapsedTime)}");
            BackgroundDataLogger.Instance.StopLogging();
        }

        SetSettingsCanvasVisible(false);
        if (settingsCanvas != null) settingsCanvas.SetActive(true);
        if (tutorialCanvas != null) tutorialCanvas.SetActive(true);

        OnGameStopped?.Invoke();
    }

    private void CreatePlatformInstance()
    {
        // La plataforma se crea una vez y se recoloca para evitar instancias repetidas.
        if (platformPrefab == null) return;
        _platformInstance = Instantiate(platformPrefab, _playCenter, _playRotation);
        _platformInstance.SetActive(false);
        _platformCP = _platformInstance.GetComponent<ColorPlatform>();
    }

    private void ShowPlatformAt(Vector3 worldPos, DetectedColor color)
    {
        // Configure limpia el estado interno de ColorPlatform antes de mostrarla.
        if (_platformInstance == null || _platformCP == null) return;

        _platformCP.Configure(color, GetMaterialForColor(color));
        _platformInstance.transform.SetPositionAndRotation(worldPos, _playRotation);
        _platformInstance.SetActive(true);
    }

    // Oculta la plataforma actual sin destruirla.
    private void HidePlatform()
    {
        if (_platformInstance != null) _platformInstance.SetActive(false);
    }

    // Devuelve el material asociado al color objetivo.
    private Material GetMaterialForColor(DetectedColor color) => color switch
    {
        DetectedColor.Green => matGreen,
        DetectedColor.Blue => matBlue,
        _ => matRed
    };

    private static DetectedColor RandomColor()
    {
        // Esta version usa rojo y azul porque son los cubos activos del protocolo.
        int randomVal = UnityEngine.Random.Range(0, 2);
        return randomVal switch
        {
            0 => DetectedColor.Red,
            _ => DetectedColor.Blue
        };
    }

    private static GameObject SpawnVFX(GameObject prefab, Vector3 worldPos, Transform refTransform, bool asChild, float destroyAfter = 5f)
    {
        // El VFX puede ir libre o como hijo; la escala se ajusta para verse consistente.
        if (prefab == null) return null;
        GameObject vfx = Instantiate(prefab, worldPos, Quaternion.identity, null);

        if (refTransform != null && !asChild)
        {
            vfx.transform.localScale = refTransform.lossyScale;
        }
        else if (refTransform != null && asChild)
        {
            vfx.transform.SetParent(refTransform, true);
            vfx.transform.localScale = Vector3.one;
        }
        else
        {
            vfx.transform.localScale = Vector3.one;
        }

        Destroy(vfx, destroyAfter);
        return vfx;
    }

    private static GameObject SpawnVFX(GameObject prefab, Vector3 worldPos, float destroyAfter = 5f)
    {
        // Sobrecarga para efectos simples sin transform de referencia.
        return SpawnVFX(prefab, worldPos, null, false, destroyAfter);
    }

    private float PlayCalibrationCompleteSound()
    {
        // Reproduce el sonido de punto capturado sin cortar la voz del tutorial.
        AudioClip clip = calibrationCompleteClip != null ? calibrationCompleteClip : calibrationCompleteSound != null ? calibrationCompleteSound.clip : null;
        AudioSource player = GetCalibrationCompletePlayer();

        if (player != null && clip != null)
        {
            player.Stop();
            player.PlayOneShot(clip, calibrationCompleteVolume);
            return clip.length;
        }

        if (calibrationCompleteSound != null && calibrationCompleteSound != (tutorialMedia != null ? tutorialMedia.audioSource : null))
        {
            PlaySound(calibrationCompleteSound);
            return calibrationCompleteSound.clip != null ? calibrationCompleteSound.clip.length : 0f;
        }

        return 0f;
    }

    private AudioSource GetCalibrationCompletePlayer()
    {
        // Reutiliza un AudioSource dedicado o crea uno separado del tutorial si hace falta.
        if (_calibrationCompletePlayer != null) return _calibrationCompletePlayer;

        AudioSource tutorialVoice = tutorialMedia != null ? tutorialMedia.audioSource : null;
        if (calibrationCompleteSound != null && calibrationCompleteSound != tutorialVoice)
        {
            _calibrationCompletePlayer = calibrationCompleteSound;
        }
        else
        {
            GameObject go = new GameObject("CalibrationCompleteAudio");
            go.transform.SetParent(transform, false);
            _calibrationCompletePlayer = go.AddComponent<AudioSource>();
            CopyAudioSettings(calibrationCompleteSound != null ? calibrationCompleteSound : tutorialVoice, _calibrationCompletePlayer);
        }

        ConfigureOneShotSource(_calibrationCompletePlayer);
        return _calibrationCompletePlayer;
    }

    private static void ConfigureOneShotSource(AudioSource source)
    {
        // Deja fuentes preparadas para disparos cortos sin loop ni reproduccion automatica.
        if (source == null) return;
        source.playOnAwake = false;
        source.loop = false;
    }

    private void ResolveLegacyAudioSources()
    {
        // Rellena referencias de audio antiguas por nombre de clip para no romper escenas existentes.
        AudioSource[] sources = GetComponents<AudioSource>();
        foreach (AudioSource source in sources)
        {
            string clipName = source.clip != null ? source.clip.name : string.Empty;
            if (string.IsNullOrEmpty(clipName)) continue;

            if (incorrectSound == null && clipName.IndexOf("Mal", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                incorrectSound = source;
            }
            else if (roundCompleteSound == null && clipName.IndexOf("SiguienteRonda", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                roundCompleteSound = source;
            }
            else if (correctSound == null && clipName.IndexOf("ColocadoBien", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                correctSound = source;
            }
        }

        if (calibrationCompleteSound == null)
        {
            calibrationCompleteSound = correctSound;
        }
    }

    private IEnumerator ChaseGameLoop()
    {
        // Bucle principal: muestra plataforma, espera colocacion o timeout y prepara el siguiente objetivo.
        // Este bucle no detecta cubos directamente; ColorPlatform avisa con ReportPlacement.
        if (tutorialMedia != null) tutorialMedia.PlayGameStart();

        SetText("<b>Start!</b>\nPlace the cube with the correct color!");
        yield return new WaitForSeconds(1.5f);

        bool haSonadoTutorialNuevaPlat = false;

        while (IsGameActive)
        {
            int successCount = 0;
            int currentMaxSeconds = initialCountdownSeconds;
            ClearTrailLines(); HideFloatingTimer();

            // Cada ronda reinicia el contador de dificultad; dentro de la ronda se reduce tras cada acierto.
            SetText($"<b>Round {CurrentRound}</b>");
            yield return new WaitForSeconds(0.5f);

            if (BackgroundDataLogger.Instance != null)
                BackgroundDataLogger.Instance.LogGameEvent("RoundStart",
                    $"round={CurrentRound} countdown={initialCountdownSeconds}s");

            DetectedColor nextColor = RandomColor();
            Vector3 nextPos = RandomSpawnPosition(Vector3.one * 9999f);

            while (successCount < targetPerCycle && IsGameActive)
            {
                // currentColor/currentPos son el objetivo activo; nextColor/nextPos se preparan tras un acierto.
                // Preparar el siguiente objetivo antes del orbe permite dibujar la guia sin esperar otro ciclo.
                DetectedColor currentColor = nextColor;
                Vector3 currentPos = nextPos;

                ShowPlatformAt(currentPos, currentColor);

                string colorName = ColorDisplayName(currentColor);
                SetText($"<b>Round {CurrentRound}</b> - {successCount}/{targetPerCycle}\n" +
                        $"Place the <color={ColorHex(currentColor)}>{colorName}</color> cube!");

                _waitingForPlacement = true; _correctPlaced = false; CanInteract = true;
                float timer = currentMaxSeconds;
                int lastDisplayedSecond = -1;

                // Mientras espera, el resultado puede llegar desde ReportPlacement o por timeout.
                while (_waitingForPlacement && timer > 0f && IsGameActive)
                {
                    // El temporizador flotante solo cambia al variar el segundo visible.
                    timer -= Time.deltaTime;
                    int currentSecond = Mathf.CeilToInt(timer);
                    if (currentSecond != lastDisplayedSecond) { UpdateFloatingTimer(currentPos, currentSecond, currentColor); lastDisplayedSecond = currentSecond; }
                    SetProgress(Mathf.Clamp01(timer / currentMaxSeconds));
                    yield return null;
                }

                CanInteract = false;
                if (!IsGameActive) yield break;

                if (_correctPlaced)
                {
                    // Tras un acierto, el orbe guia al siguiente objetivo y el tiempo se endurece.
                    if (!haSonadoTutorialNuevaPlat && tutorialMedia != null)
                    {
                        haSonadoTutorialNuevaPlat = true;
                        tutorialMedia.PlayNewPlatform();
                    }

                    successCount++; UpdateHUD(); HidePlatform(); HideFloatingTimer();
                    Vector3 prevPos = currentPos;
                    nextColor = RandomColor();
                    nextPos = RandomSpawnPosition(prevPos);

                    if (successCount < targetPerCycle)
                    {
                        // El orbe sale de la plataforma anterior y termina donde aparecera la siguiente.
                        yield return new WaitForSeconds(0.1f);
                        yield return StartCoroutine(ShootOrb(prevPos, nextPos, currentColor));
                        currentMaxSeconds = Mathf.Max(minCountdownSeconds, currentMaxSeconds - countdownDecrementStep);
                        yield return new WaitForSeconds(pauseBetweenPlatforms * 0.5f);
                    }
                    else yield return new WaitForSeconds(pauseBetweenPlatforms);
                }
                else
                {
                    // El timeout cuenta como fallo: rompe combo, penaliza y crea otro objetivo.
                    CurrentCombo = 0; Score = Mathf.Max(0, Score - penaltyPerIncorrect);
                    TotalMisses++;
                    PlaySound(incorrectSound); UpdateHUD(); HidePlatform(); HideFloatingTimer();

                    if (BackgroundDataLogger.Instance != null)
                        BackgroundDataLogger.Instance.LogGameEvent("Timeout",
                            CurrentRound, Score, TotalHits, TotalMisses, CurrentCombo, BestCombo,
                            currentColor.ToString(), "", $"limit={currentMaxSeconds}s");

                    // Tras fallar por tiempo, se elige otro objetivo para que el usuario no quede bloqueado.
                    nextColor = RandomColor(); nextPos = RandomSpawnPosition(currentPos);
                    yield return new WaitForSeconds(pauseBetweenPlatforms);
                }
            }
            if (!IsGameActive) yield break;
            CompleteRound();
            if (!IsGameActive) yield break;
            yield return new WaitForSeconds(1.5f);
        }
    }

    private void CreateFloatingTimer()
    {
        // Texto 3D independiente del HUD para que el usuario mire la plataforma objetivo.
        GameObject go = new GameObject("FloatingTimerText");
        _floatingTimerText = go.AddComponent<TextMeshPro>();
        _floatingTimerText.alignment = TextAlignmentOptions.Center;
        _floatingTimerText.fontSize = 3f;
        _floatingTimerText.fontStyle = FontStyles.Bold;
        _floatingTimerText.outlineWidth = 0.15f;
        _floatingTimerText.outlineColor = Color.black;
        go.SetActive(false);
    }

    private void UpdateFloatingTimer(Vector3 platformPos, int secondsLeft, DetectedColor color)
    {
        // El contador flota sobre la plataforma y usa el color del objetivo.
        if (_floatingTimerText == null) return;
        _floatingTimerText.gameObject.SetActive(true);
        _floatingTimerText.text = secondsLeft.ToString();
        _floatingTimerText.color = GetLineColor(color);
        _floatingTimerText.transform.SetPositionAndRotation(platformPos + Vector3.up * 0.15f, _playRotation);
    }

    // Oculta el contador flotante sin destruirlo.
    private void HideFloatingTimer() { if (_floatingTimerText != null) _floatingTimerText.gameObject.SetActive(false); }

    // Punto de registro mantenido para plataformas creadas desde prefab.
    public void RegisterPlatform(ColorPlatform platform) { }

    public void ReportPlacement(DetectedColor cubeColor, DetectedColor platformColor, bool isCorrect)
    {
        // Las plataformas solo reportan; puntuacion y estado viven aqui como fuente unica.
        // ColorPlatform detecta el contacto fisico; MemoryGame decide si suma, penaliza o avanza.
        if (!IsGameActive || !CanInteract) return;
        if (isCorrect)
        {
            // Un acierto cierra la espera actual y deja avanzar al siguiente objetivo.
            _correctPlaced = true; _waitingForPlacement = false;
            CurrentCombo++; if (CurrentCombo > BestCombo) BestCombo = CurrentCombo;
            TotalHits++;
            Score += pointsPerCorrect + Mathf.Max(0, CurrentCombo - 1) * comboBonus;
            PlaySound(correctSound); UpdateHUD();

            if (BackgroundDataLogger.Instance != null)
                BackgroundDataLogger.Instance.LogGameEvent("Hit",
                    CurrentRound, Score, TotalHits, TotalMisses, CurrentCombo, BestCombo,
                    platformColor.ToString(), cubeColor.ToString());
        }
        else
        {
            // Un color incorrecto penaliza, pero mantiene la plataforma para corregir al momento.
            CurrentCombo = 0; Score = Mathf.Max(0, Score - penaltyPerIncorrect);
            TotalMisses++;
            PlaySound(incorrectSound); UpdateHUD();

            if (BackgroundDataLogger.Instance != null)
                BackgroundDataLogger.Instance.LogGameEvent("WrongColor",
                    CurrentRound, Score, TotalHits, TotalMisses, CurrentCombo, BestCombo,
                    platformColor.ToString(), cubeColor.ToString());
        }
    }

    private void CompleteRound()
    {
        // Cada ronda abre una sesion etiquetada para separar datos por mano y numero de ronda.
        PlaySound(roundCompleteSound);

        if (BackgroundDataLogger.Instance != null)
            BackgroundDataLogger.Instance.LogGameEvent("RoundComplete",
                $"round={CurrentRound} score={Score} hits={TotalHits} misses={TotalMisses}");

        if (totalRounds <= 0 || CurrentRound < totalRounds)
        {
            // totalRounds <= 0 deja modo infinito para sesiones abiertas.
            CurrentRound++;
            if (BackgroundDataLogger.Instance != null)
                BackgroundDataLogger.Instance.StartLogging(
                    $"R{CurrentRound}_{(_useRightHand ? "RH" : "LH")}");
            ClearTrailLines(); HidePlatform(); UpdateHUD();
            SetText($"<color=#60FF90>Round completed!</color>\nPreparing round {CurrentRound}...");
        }
        else
        {
            StopGame(isWin: true);
        }
    }

    private IEnumerator ShootOrb(Vector3 from, Vector3 to, DetectedColor color)
    {
        // La trayectoria en arco da direccion sin dejar una linea permanente sobre la mesa.
        ClearTrailLines();
        if (orbVFXPrefab == null) yield break;

        Vector3 startPos = from + Vector3.up * 0.05f;
        Vector3 endPos = to + Vector3.up * 0.05f;

        GameObject orb = Instantiate(orbVFXPrefab, startPos, Quaternion.identity);
        _trailLines.Add(orb);

        float elapsed = 0f;
        while (elapsed < orbTravelDuration && orb != null)
        {
            elapsed += Time.deltaTime;
            float normalizedTime = Mathf.Clamp01(elapsed / orbTravelDuration);
            float smooth = normalizedTime * normalizedTime * (3f - 2f * normalizedTime);
            float arc = Mathf.Sin(smooth * Mathf.PI) * 0.12f;

            orb.transform.position = Vector3.Lerp(startPos, endPos, smooth)
                                     + Vector3.up * arc;
            yield return null;
        }

        ClearTrailLines();
    }

    // Elimina orbes o rastros temporales entre objetivos.
    private void ClearTrailLines()
    {
        foreach (var line in _trailLines)
            if (line != null) Destroy(line);

        _trailLines.Clear();
    }

    // Color de apoyo para temporizador y efectos segun objetivo.
    private static Color GetLineColor(DetectedColor color) => color switch
    {
        DetectedColor.Red => new Color(1.0f, 0.25f, 0.25f, 1f),
        DetectedColor.Green => new Color(0.2f, 0.9f, 0.3f, 1f),
        DetectedColor.Blue => new Color(0.3f, 0.5f, 1.0f, 1f),
        _ => Color.white
    };

    // Nombre visible del color de objetivo.
    private static string ColorDisplayName(DetectedColor c) => c switch
    {
        DetectedColor.Red => "RED",
        DetectedColor.Green => "GREEN",
        DetectedColor.Blue => "BLUE",
        _ => "?"
    };

    // Color HTML usado en los mensajes ricos del HUD.
    private static string ColorHex(DetectedColor c) => c switch
    {
        DetectedColor.Red => "#FF4040",
        DetectedColor.Green => "#40FF60",
        DetectedColor.Blue => "#5080FF",
        _ => "#FFFFFF"
    };

    // Crea la esfera que muestra donde se capturara el punto de calibracion.
    private void CreatePreviewSphere()
    {
        _previewSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _previewSphere.name = "CalibPreview";
        _previewSphere.transform.localScale = Vector3.one * 0.03f;

        var col = _previewSphere.GetComponent<Collider>();
        if (col) Destroy(col);

        var rend = _previewSphere.GetComponent<Renderer>();
        if (rend)
        {
            var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            mat.color = new Color(1f, 0.85f, 0.3f, 0.6f);
            rend.material = mat;
        }

        _previewSphere.SetActive(false);
    }

    // Mueve la esfera de calibracion y la hace crecer al mantener la mano quieta.
    private void UpdatePreviewSphere(Vector3 pos, bool allowGrowth)
    {
        if (_previewSphere == null) return;

        _previewSphere.SetActive(true);
        _previewSphere.transform.position = pos;

        float size = allowGrowth && _holdStarted
            ? Mathf.Lerp(0.03f, 0.05f, _holdTimer / holdTime)
            : 0.03f;

        _previewSphere.transform.localScale = Vector3.one * size;
    }

    // Oculta la vista previa de calibracion.
    private void HideCalibrationPreview()
    {
        if (_previewSphere != null) _previewSphere.SetActive(false);
    }

    public void SetPlatformHeightOffset(float newOffset)
    {
        // Un slider de UI ajusta el plano de mesa sin recalibrar toda el area.
        tableHeightCorrection = newOffset;
        RefreshTableHeightDependentObjects();
    }

    public void SetAutoTableHeightCorrection(float correction)
    {
        // Fusion puede ajustar automaticamente la altura si observa el cubo estable sobre la mesa.
        autoTableHeightCorrection = correction;
        RefreshTableHeightDependentObjects();
    }

    private void RefreshTableHeightDependentObjects()
    {
        // Reubica objetos que dependen de la altura de mesa tras cambiar una correccion.
        if (_phase == Phase.Playing || _phase == Phase.Starting || _phase == Phase.GameOver)
        {
            _playCenter.y = PlatformWorldY;

            if (_platformInstance != null && _platformInstance.activeSelf)
            {
                Vector3 currentPos = _platformInstance.transform.position;
                currentPos.y = PlatformWorldY;
                _platformInstance.transform.position = currentPos;
            }

            if (_floatingTimerText != null && _floatingTimerText.gameObject.activeSelf)
            {
                Vector3 timerPos = _floatingTimerText.transform.position;
                timerPos.y = PlatformWorldY + 0.15f;
                _floatingTimerText.transform.position = timerPos;
            }

            ApplyTableCorrectionToCubes();
        }
    }

    private void ApplyTableCorrectionToCubes()
    {
        // Propaga correcciones de mesa a todos los trackers de cubos activos.
        FusionTracker[] cubes = FindObjectsByType<FusionTracker>(FindObjectsSortMode.None);
        foreach (var cube in cubes)
        {
            cube.tableWorldHeightY = _tableHeight;
            cube.tableHeightCorrection = tableHeightCorrection;
            cube.autoTableHeightCorrection = autoTableHeightCorrection;
        }
    }
}
