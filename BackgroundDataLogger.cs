using System;
using System.Globalization;
using System.IO;
using UnityEngine;

public class BackgroundDataLogger : MonoBehaviour
{
    // Capa de evaluacion cuantitativa. Registra sensores, vision, fusion,
    // juego y movimiento humano para analizar la sesion despues.
    // Cada fuente queda en su CSV y la segmentacion resume agarre, transporte y liberacion.
    // Este logger no decide gameplay ni tracking: solo observa eventos ya resueltos por otros scripts.
    //
    // Para alguien nuevo: cada metodo Log... anade una fila a un archivo CSV.
    // El tiempo de todos los archivos empieza en _sessionStartTime para poder cruzarlos despues.
    public static BackgroundDataLogger Instance { get; private set; }

    // --- Configuracion de salida ---
    // Carpeta base donde Unity guardara las sesiones dentro del almacenamiento persistente.
    [Header("Configuration")]
    [Tooltip("Subcarpeta dentro de Application.persistentDataPath donde se crean las sesiones.")]
    public string folderName = "SessionLogs";

    // --- CSVs de sesion ---
    // Cada AsyncCsvLogger escribe en segundo plano un archivo distinto para no bloquear el frame.
    // Separar fuentes ayuda a analizar datos de frecuencias diferentes sin mezclar columnas.
    private AsyncCsvLogger _sensorCsv;
    private AsyncCsvLogger _visionCsv;
    private AsyncCsvLogger _fusionCsv;
    private AsyncCsvLogger _eventsCsv;
    private AsyncCsvLogger _kinematicsCsv;
    private AsyncCsvLogger _cubeStateCsv;
    private AsyncCsvLogger _visionRawCsv;

    private AsyncCsvLogger _interactionSegmentsCsv;

    private AsyncCsvLogger _calibrationCsv;

    // --- Estado interno de sesion ---
    // _sessionStartTime fija un cero comun para alinear CSVs diferentes en postproceso.
    // _isLogging protege todos los Log... para que no escriban antes de StartLogging.
    private bool _isLogging = false;
    private double _sessionStartTime;

    // --- Estado interno de segmentacion de interacciones ---
    // Detecta cuando empieza/termina una manipulacion para resumir transporte, velocidad y liberacion.
    private bool _prevGripState = false;
    private bool _segmentActive = false;

    // Estado acumulado de una interaccion completa: agarre, transporte y liberacion.
    // Se reinicia al cerrar un segmento o al empezar una sesion nueva.
    private double _segmentStartTime = -1.0;
    private Vector3 _segmentStartPos = Vector3.zero;
    private Vector3 _segmentLastPos = Vector3.zero;

    // Metricas acumuladas mientras el segmento esta activo.
    private float _segmentPathLength = 0f;
    private float _segmentSpeedSum = 0f;
    private int _segmentSampleCount = 0;
    private float _segmentMaxSpeed = 0f;

    // Tiempo relativo de sesion: evita repetir la resta en cada metodo Log...
    private double SessionTimeNow => ExperimentClock.Now - _sessionStartTime;

    private static string Csv(string format, params object[] args)
    {
        // Todos los CSV usan punto decimal, independientemente del idioma/regional del dispositivo.
        return string.Format(CultureInfo.InvariantCulture, format, args);
    }

    private void Awake()
    {
        // Singleton persistente para que el logger sobreviva a cambios de escena.
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
        DontDestroyOnLoad(gameObject);
    }

    // Al destruir el logger, cierra cualquier sesion abierta.
    private void OnDestroy() => StopLogging();

    public void StartLogging(string tag = "")
    {
        // Cada sesion crea una carpeta con CSV separados para sensores, vision, fusion y eventos.
        // Separar archivos evita mezclar datos de frecuencia muy distinta: IMU rapido, eventos puntuales, etc.
        if (_isLogging) StopLogging();

        string suffix = string.IsNullOrEmpty(tag) ? "" : $"_{tag}";
        string autoID = "SESSION_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + suffix;
        string runDir = Path.Combine(Application.persistentDataPath, folderName, autoID);

        try
        {
            Directory.CreateDirectory(runDir);
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataLogger] Error creating session folder: {e.Message}");
            return;
        }

        _sensorCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "sensor_imu.csv"),
            "time_s,tracker_id,qx,qy,qz,qw,raw_grip");

        // Separar por fuente permite analizar latencia, frescura visual y continuidad de fusion.
        _visionCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "vision_data.csv"),
            "time_s,tracker_id," +
            "bbox_x,bbox_y,bbox_w,bbox_h,bbox_score," +
            "kp0_x,kp0_y,kp0_v,kp1_x,kp1_y,kp1_v," +
            "kp2_x,kp2_y,kp2_v,kp3_x,kp3_y,kp3_v," +
            "calib_status");

        _fusionCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "fusion_result.csv"),
            "time_s,tracker_id,pos_x,pos_y,pos_z,rot_y,is_gripping," +
            "down_face_x,down_face_y,down_face_z");

        _eventsCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "game_events.csv"),
            "time_s,event_type,round,score,hits,misses,combo,best_combo,expected_color,placed_color,details");

        _kinematicsCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "patient_kinematics.csv"),
            "time_s," +
            "head_px,head_py,head_pz,head_rx,head_ry,head_rz,head_rw," +
            "handR_px,handR_py,handR_pz,handR_rx,handR_ry,handR_rz,handR_rw," +
            "handL_px,handL_py,handL_pz,handL_rx,handL_ry,handL_rz,handL_rw," +
            "gaze_x,gaze_y,gaze_z");

        _cubeStateCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "cube_state.csv"),
            "time_s,tracker_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w,euler_x,euler_y,euler_z," +
            "vel_x,vel_y,vel_z,speed_mps,ang_vel_deg_s,is_gripping,grip_source,contact_level,raw_grip,imu_delta_deg,down_face_x,down_face_y,down_face_z");

        _visionRawCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "vision_data_all.csv"),
            "time_s,tracker_id," +
            "bbox_x,bbox_y,bbox_w,bbox_h,bbox_score," +
            "kp0_x,kp0_y,kp0_v,kp1_x,kp1_y,kp1_v," +
            "kp2_x,kp2_y,kp2_v,kp3_x,kp3_y,kp3_v," +
            "calib_status");

        _interactionSegmentsCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "interaction_segments.csv"),
            "start_time_s,end_time_s,duration_s,tracker_id," +
            "start_x,start_y,start_z,end_x,end_y,end_z," +
            "path_length_m,avg_speed_mps,max_speed_mps,release_type,final_contact_level");

        _calibrationCsv = new AsyncCsvLogger(
            Path.Combine(runDir, "calibration_log.csv"),
            "time_s,tracker_id,path,raw_offset_deg,abs_error_deg,sym_error_deg,calib_type");

        _sessionStartTime = ExperimentClock.Now;
        _isLogging = true;

        // Los segmentos no deben heredar estado de una ronda anterior aunque el logger siga vivo.
        ResetInteractionSegmentationState();

        Debug.Log($"[DataLogger] Session started: {autoID}");
    }

    public void StopLogging()
    {
        // Antes de cerrar archivos, vuelca cualquier segmento abierto como cierre de sesion.
        if (!_isLogging) return;

        FlushOpenInteractionSegmentAsAborted();

        _sensorCsv?.Dispose();
        _visionCsv?.Dispose();
        _fusionCsv?.Dispose();
        _eventsCsv?.Dispose();
        _kinematicsCsv?.Dispose();
        _cubeStateCsv?.Dispose();
        _visionRawCsv?.Dispose();
        _interactionSegmentsCsv?.Dispose();
        _calibrationCsv?.Dispose();

        _isLogging = false;
        Debug.Log("[DataLogger] Session closed and files saved.");
    }

    public void LogSensorData(string trackerId, Quaternion rawRot, int rawGrip)
    {
        // Guarda el quaternion bruto para revisar conversiones de ejes o deriva del sensor.
        if (!_isLogging) return;
        double t = SessionTimeNow;

        _sensorCsv.EnqueueRawLine(Csv(
            "{0:F4},{1},{2:F4},{3:F4},{4:F4},{5:F4},{6}",
            t, trackerId, rawRot.x, rawRot.y, rawRot.z, rawRot.w, rawGrip));
    }

    public void LogVisionData(string trackerId, Rect bbox, float score,
                              Vector2[] kpts, float[] vis, string calibStatus)
    {
        // Este CSV guarda las detecciones aceptadas por el modo experimental.
        WriteVisionCsvLine(_visionCsv, trackerId, bbox, score, kpts, vis, calibStatus);
    }

    public void LogVisionDataAll(string trackerId, Rect bbox, float score,
                                  Vector2[] kpts, float[] vis, string calibStatus)
    {
        // Este canal guarda todas las detecciones para compararlas con el filtrado experimental.
        WriteVisionCsvLine(_visionRawCsv, trackerId, bbox, score, kpts, vis, calibStatus);
    }

    private void WriteVisionCsvLine(AsyncCsvLogger csv, string trackerId, Rect bbox, float score,
                                    Vector2[] kpts, float[] vis, string calibStatus)
    {
        // Formato compartido por vision_data.csv y vision_data_all.csv.
        // Ambos CSV tienen las mismas columnas; solo cambia que logger recibe la fila.
        if (!_isLogging || csv == null) return;
        double t = SessionTimeNow;

        string line = Csv(
            "{0:F4},{1},{2:F2},{3:F2},{4:F2},{5:F2},{6:F3}",
            t, trackerId, bbox.center.x, bbox.center.y, bbox.width, bbox.height, score);

        for (int i = 0; i < 4; i++)
        {
            if (kpts != null && vis != null && i < kpts.Length && i < vis.Length)
                line += Csv(",{0:F2},{1:F2},{2:F3}", kpts[i].x, kpts[i].y, vis[i]);
            else
                line += ",0,0,0";
        }

        line += $",{calibStatus}";
        csv.EnqueueRawLine(line);
    }

    public void LogFusionData(string trackerId, Vector3 pos, Quaternion rot,
                              bool gripping, Vector3 downFace)
    {
        // Resume la pose fusionada, no los datos brutos que la produjeron.
        if (!_isLogging) return;
        double t = SessionTimeNow;

        _fusionCsv.EnqueueRawLine(Csv(
            "{0:F4},{1},{2:F4},{3:F4},{4:F4},{5:F2},{6},{7:F2},{8:F2},{9:F2}",
            t, trackerId, pos.x, pos.y, pos.z, rot.eulerAngles.y,
            gripping ? 1 : 0, downFace.x, downFace.y, downFace.z));
    }

    public void LogCubeStateData(string trackerId, Vector3 pos, Quaternion rot,
                                  Vector3 velocity, float speedMps, float angularVelocityDegS,
                                  bool gripping, string gripSource, string contactLevel,
                                  int rawGrip, float imuDeltaDeg, Vector3 downFace)
    {
        // El estado de cubo alimenta tanto el CSV de tracking como la segmentacion.
        // Es la fila mas completa: mezcla pose, velocidades, agarre, contacto e IMU para analisis posterior.
        if (!_isLogging) return;

        double t = SessionTimeNow;
        Vector3 e = rot.eulerAngles;

        _cubeStateCsv.EnqueueRawLine(Csv(
            "{0:F4},{1},{2:F4},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F2},{10:F2},{11:F2},{12:F4},{13:F4},{14:F4},{15:F4},{16:F2},{17},{18},{19},{20},{21:F3},{22:F2},{23:F2},{24:F2}",
            t, trackerId,
            pos.x, pos.y, pos.z,
            rot.x, rot.y, rot.z, rot.w,
            e.x, e.y, e.z,
            velocity.x, velocity.y, velocity.z,
            speedMps, angularVelocityDegS,
            gripping ? 1 : 0, gripSource, contactLevel, rawGrip, imuDeltaDeg,
            downFace.x, downFace.y, downFace.z));

        UpdateInteractionSegmentation(trackerId, t, pos, speedMps, gripping, contactLevel);
    }

    public void LogGameEvent(string eventType, int round, int score,
                             int hits, int misses, int combo, int bestCombo,
                             string expectedColor = "", string placedColor = "",
                             string details = "")
    {
        // Los eventos discretos alinean series continuas con aciertos, fallos o rondas.
        if (!_isLogging) return;
        double t = SessionTimeNow;

        _eventsCsv.EnqueueRawLine(Csv(
            "{0:F4},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}",
            t, eventType, round, score, hits, misses, combo, bestCombo,
            expectedColor, placedColor, details));
    }

    public void LogGameEvent(string eventType, string details = "")
    {
        // Sobrecarga de comodidad: toma contadores de MemoryGame si esta disponible.
        if (!_isLogging) return;

        var game = MemoryGame.Instance;
        if (game != null)
        {
            LogGameEvent(eventType, game.CurrentRound, game.Score,
                         game.TotalHits, game.TotalMisses,
                         game.CurrentCombo, game.BestCombo,
                         "", "", details);
        }
        else
        {
            double t = SessionTimeNow;
            _eventsCsv.EnqueueRawLine(Csv(
                "{0:F4},{1},0,0,0,0,0,0,,,{2}", t, eventType, details));
        }
    }

    public void LogKinematicsData(Transform head, Transform handR, Transform handL, Vector3 gazeDir)
    {
        // Acepta transforms nulos para no romper la sesion si se pierde una mano.
        // Cuando falta una referencia se guarda cero/identidad; asi el CSV mantiene siempre las mismas columnas.
        if (!_isLogging) return;
        double t = SessionTimeNow;

        Vector3 hPos = head != null ? head.position : Vector3.zero;
        Quaternion hRot = head != null ? head.rotation : Quaternion.identity;
        Vector3 rPos = handR != null ? handR.position : Vector3.zero;
        Quaternion rRot = handR != null ? handR.rotation : Quaternion.identity;
        Vector3 lPos = handL != null ? handL.position : Vector3.zero;
        Quaternion lRot = handL != null ? handL.rotation : Quaternion.identity;

        _kinematicsCsv.EnqueueRawLine(Csv(
            "{0:F4}," +
            "{1:F4},{2:F4},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4}," +
            "{8:F4},{9:F4},{10:F4},{11:F4},{12:F4},{13:F4},{14:F4}," +
            "{15:F4},{16:F4},{17:F4},{18:F4},{19:F4},{20:F4},{21:F4}," +
            "{22:F4},{23:F4},{24:F4}",
            t,
            hPos.x, hPos.y, hPos.z, hRot.x, hRot.y, hRot.z, hRot.w,
            rPos.x, rPos.y, rPos.z, rRot.x, rRot.y, rRot.z, rRot.w,
            lPos.x, lPos.y, lPos.z, lRot.x, lRot.y, lRot.z, lRot.w,
            gazeDir.x, gazeDir.y, gazeDir.z));
    }

    public void LogCalibrationEvent(string trackerId, string path, float rawOffsetDeg)
    {
        // Guarda error absoluto y simetrico para detectar caras equivalentes mal elegidas.
        if (!_isLogging || _calibrationCsv == null) return;
        double t = SessionTimeNow;

        float absError = Mathf.Abs(rawOffsetDeg);

        float mod90 = absError % 90f;
        float symError = Mathf.Min(mod90, 90f - mod90);

        string calibType = absError >= 15f ? "instant" : "smooth";

        _calibrationCsv.EnqueueRawLine(Csv(
            "{0:F4},{1},{2},{3:F2},{4:F2},{5:F2},{6}",
            t, trackerId, path, rawOffsetDeg, absError, symError, calibType));
    }

    private void ResetInteractionSegmentationState()
    {
        // Limpia todo el estado de segmentacion antes de iniciar o reiniciar una sesion.
        _prevGripState = false;
        _segmentActive = false;

        _segmentStartTime = -1.0;
        _segmentStartPos = Vector3.zero;
        _segmentLastPos = Vector3.zero;

        _segmentPathLength = 0f;
        _segmentSpeedSum = 0f;
        _segmentSampleCount = 0;
        _segmentMaxSpeed = 0f;
    }

    private void UpdateInteractionSegmentation(string trackerId, double timeS, Vector3 pos,
                                                float speedMps, bool gripping, string contactLevel)
    {
        // Un segmento empieza con agarre y termina con liberacion para resumir cada movimiento.
        // En vez de analizar miles de frames, este CSV deja una fila por accion de coger y soltar.
        if (!_prevGripState && gripping)
        {
            // Inicio: primer frame donde el objeto pasa a estar agarrado.
            // Desde aqui se acumulan distancia recorrida y velocidades hasta detectar liberacion.
            _segmentActive = true;
            _segmentStartTime = timeS;
            _segmentStartPos = pos;
            _segmentLastPos = pos;

            _segmentPathLength = 0f;
            _segmentSpeedSum = 0f;
            _segmentSampleCount = 0;
            _segmentMaxSpeed = 0f;
        }

        if (_segmentActive && gripping)
        {
            // Durante el agarre acumula longitud y velocidad para medir calidad del transporte.
            float stepDist = Vector3.Distance(_segmentLastPos, pos);
            _segmentPathLength += stepDist;
            _segmentLastPos = pos;

            _segmentSpeedSum += speedMps;
            _segmentSampleCount++;

            if (speedMps > _segmentMaxSpeed)
                _segmentMaxSpeed = speedMps;
        }

        if (_prevGripState && !gripping && _segmentActive)
        {
            // El frame de liberacion decide si fue colocacion controlada o caida rapida.
            // El tipo exacto se clasifica en FinalizeInteractionSegment con velocidad/contacto.
            FinalizeInteractionSegment(trackerId, timeS, pos, speedMps, contactLevel, "release");
        }

        _prevGripState = gripping;
    }

    private void FinalizeInteractionSegment(string trackerId, double endTimeS, Vector3 endPos,
                                             float releaseSpeed, string contactLevel, string defaultReleaseType)
    {
        // Cerrar el segmento comprime muchas muestras en metricas comparables entre pacientes o rondas.
        // El CSV resultante es de bajo volumen: una fila por manipulacion, no una fila por frame.
        if (!_segmentActive || _interactionSegmentsCsv == null)
            return;

        double duration = Mathf.Max(0f, (float)(endTimeS - _segmentStartTime));
        float avgSpeed = _segmentSampleCount > 0 ? _segmentSpeedSum / _segmentSampleCount : 0f;

        string releaseType = ClassifyReleaseType(releaseSpeed, contactLevel, defaultReleaseType);

        // Esta fila resume todo el gesto: inicio, final, duracion, distancia, velocidad y tipo de liberacion.
        _interactionSegmentsCsv.EnqueueRawLine(Csv(
            "{0:F4},{1:F4},{2:F4},{3},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10:F4},{11:F4},{12:F4},{13},{14}",
            _segmentStartTime,
            endTimeS,
            duration,
            trackerId,
            _segmentStartPos.x, _segmentStartPos.y, _segmentStartPos.z,
            endPos.x, endPos.y, endPos.z,
            _segmentPathLength,
            avgSpeed,
            _segmentMaxSpeed,
            releaseType,
            contactLevel));

        _segmentActive = false;
        _segmentStartTime = -1.0;
        _segmentStartPos = Vector3.zero;
        _segmentLastPos = Vector3.zero;
        _segmentPathLength = 0f;
        _segmentSpeedSum = 0f;
        _segmentSampleCount = 0;
        _segmentMaxSpeed = 0f;
    }

    private void FlushOpenInteractionSegmentAsAborted()
    {
        // Si se cierra la sesion durante un agarre, marca el segmento como final de sesion.
        if (!_segmentActive)
            return;

        FinalizeInteractionSegment(
            trackerId: "unknown",
            endTimeS: SessionTimeNow,
            endPos: _segmentLastPos,
            releaseSpeed: 0f,
            contactLevel: "",
            defaultReleaseType: "session_end");
    }

    private string ClassifyReleaseType(float releaseSpeed, string contactLevel, string fallback)
    {
        // Diferencia colocacion y caida por velocidad final y contacto, no por un evento explicito.
        bool hasContact = HasMeaningfulContact(contactLevel);

        if (hasContact && releaseSpeed < 0.05f)
            return "placement";

        if (releaseSpeed > 0.20f)
            return "drop";

        return fallback;
    }

    private bool HasMeaningfulContact(string contactLevel)
    {
        // Normaliza cadenas de contacto para distinguir ausencia real de contacto.
        if (string.IsNullOrWhiteSpace(contactLevel))
            return false;

        string normalizedContact = contactLevel.Trim().ToLowerInvariant();

        if (normalizedContact == "0" || normalizedContact == "none" || normalizedContact == "no" || normalizedContact == "false" || normalizedContact == "null")
            return false;

        return true;
    }
}
