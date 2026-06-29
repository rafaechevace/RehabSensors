using System;
using System.Collections.Generic;
using System.Linq;
using Meta.XR;
using Meta.XR.BuildingBlocks;
using Meta.XR.BuildingBlocks.AIBlocks;
using Meta.XR.EnvironmentDepth;
using UnityEngine;

public enum DetectedColor { Unknown, Red, Green, Blue, Yellow }

public enum CubeClass
{
    NormalCube,    // cubo sin pegatina visible
    SensorCube,    // cubo con pegatina visible
    Mug,           // taza sin keypoints ni distincion visible/oculta
}

public struct VisualDetection
{
    // Punto de intercambio entre vision y fusion. Ya no expone pixeles crudos,
    // sino posicion 3D, clase, color y calidad de keypoints.
    // FusionSystemManager consume esta estructura; por eso guarda tanto datos 3D como trazas 2D para logging.
    public Vector3 Position;
    public Vector3 Scale;
    public DetectedColor ColorCategory;
    public CubeClass Class;
    public Vector3[] KeypointsWorld;
    public bool[] KeypointWorldValid;
    public bool IsGeometricValid;
    public int ValidKeypointCount;
    public Vector2[] KeypointsPixel;
    public Pose CameraPose;
    public Rect BBox;
    public float Score;

    public bool HasUsableKeypoints =>
        KeypointWorldValid != null && KeypointWorldValid.All(v => v) && IsGeometricValid;

    public bool HasPartialKeypoints => ValidKeypointCount >= 3;
}

[RequireComponent(typeof(YoloPoseAgent), typeof(DepthTextureAccess), typeof(EnvironmentDepthManager))]
public class ObjectDetectionVisualizerV2 : MonoBehaviour
{
    // Convierte detecciones 2D de YOLO en observaciones 3D usables por fusion.
    // Mantiene separadas la percepcion visual y el tracking continuo de objetos.
    // Guarda profundidad/pose, proyecta cajas y keypoints, y emite detecciones ya validadas.
    // Este componente no decide identidad fisica; solo prepara observaciones con calidad suficiente.
    //
    // Lectura rapida:
    // - YoloPoseAgent entrega cajas/keypoints en pixeles.
    // - DepthTextureAccess aporta profundidad para convertir pixeles en posiciones 3D.
    // - FusionSystemManager recibira la lista final por OnVisualDetections.
    // --- Configuracion visual y de debug ---
    // boundingBoxPrefab solo representa la deteccion en escena; no participa en la fusion.
    [Header("Visual Configuration")]
    [SerializeField] private GameObject boundingBoxPrefab;
    [SerializeField] private bool showBoundingBoxes = true;
    public bool showDebugLogs = true;

    // Validacion geometrica para cubos con pegatina: comprueba que los keypoints tengan tamano plausible.
    [Header("Geometric Validation (Cube Only)")]
    public float stickerSizeMeters = 0.03f;
    public float geometryTolerance = 0.012f;

    // Umbrales para decidir si un keypoint de YOLO se puede usar para calibrar orientacion.
    [Header("Keypoint Detection")]
    public float keypointVisibilityThreshold = 0.4f;
    public int minValidKeypoints = 3;

    // Evento de salida: envia observaciones 3D limpias al FusionSystemManager.
    public event Action<List<VisualDetection>> OnVisualDetections;

    // --- Estado interno de proyeccion ---
    // Referencias a los componentes que producen cajas 2D, profundidad y pose de camara.
    private YoloPoseAgent _agent;
    private PassthroughCameraAccess _camera;
    private DepthTextureAccess _depth;
    private int _eyeIndex;

    // Snapshot del ultimo frame de profundidad. Mantenerlo junto evita mezclar pose de un frame con profundidad de otro.
    private FrameSnapshot _frame;

    // Pool de cajas de debug para reutilizar GameObjects en vez de crear/destruir cada frame.
    private readonly List<GameObject> _activeBoxes = new();
    private readonly Queue<GameObject> _boxPool = new();

    private struct FrameSnapshot
    {
        // Captura minima para proyectar con pose y profundidad coherentes entre si.
        public Pose CameraPose;
        public float[] DepthPixels;
        public Matrix4x4[] ViewProjectionMatrices;
    }

    private void Awake()
    {
        // El indice de ojo debe coincidir con la camara passthrough para alinear profundidad y pixel.
        _agent = GetComponent<YoloPoseAgent>();
        _camera = FindAnyObjectByType<PassthroughCameraAccess>();
        _depth = GetComponent<DepthTextureAccess>();
        _eyeIndex = _camera.CameraPosition == PassthroughCameraAccess.CameraPositionType.Left ? 0 : 1;
    }

    private void OnEnable()
    {
        // Profundidad y detecciones llegan por canales distintos y se combinan al procesar el lote.
        _agent.OnBoxesUpdated += HandleDetectionBatch;
        _depth.OnDepthTextureUpdateCPU += HandleDepthFrame;
    }

    private void OnDisable()
    {
        // Desuscribe eventos para no procesar datos mientras el componente esta desactivado.
        _agent.OnBoxesUpdated -= HandleDetectionBatch;
        _depth.OnDepthTextureUpdateCPU -= HandleDepthFrame;
    }

    private void HandleDepthFrame(DepthTextureAccess.DepthFrameData depthFrame)
    {
        // Guarda pose, profundidad y matrices coherentes para proyectar el siguiente lote de YOLO.
        _frame.CameraPose = _camera.GetCameraPose();
        _frame.DepthPixels = depthFrame.DepthTexturePixels.ToArray();
        _frame.ViewProjectionMatrices = depthFrame.ViewProjectionMatrix.ToArray();
    }

    private void HandleDetectionBatch(List<YoloBoxData> batch)
    {
        // Cada caja YOLO se proyecta a mundo, se clasifica por color y se empaqueta para FusionSystemManager.
        // batch puede estar vacio; aun asi se emite una lista para que la fusion sepa que no hubo vision.
        if (_depth != null) _depth.RequestDepthSample();

        RecycleActiveBoxes();

        var detections = new List<VisualDetection>();
        Texture2D readableTexture = AcquireReadableCameraTexture(out bool isTemporary);

        foreach (YoloBoxData box in batch)
        {
            // Primero se intenta obtener una posicion 3D de la caja. Si no hay profundidad valida, se descarta.
            if (!TryProjectToWorld(
                    box.position.x, box.position.y, box.scale.x, box.scale.y,
                    out Vector3 worldPos, out Quaternion worldRot, out Vector3 worldScale))
                continue;

            CubeClass cubeClass = ParseCubeClass(box.label);

            int numKpts = box.keypoints != null ? box.keypoints.Length : 0;
            var kptsWorld = new Vector3[numKpts];
            var kptsValid = new bool[numKpts];
            bool isGeometricValid = false;
            int validCount = 0;

            // Los keypoints solo sirven para orientacion/calibracion; una caja sin keypoints aun puede dar posicion.
            if (numKpts > 0 && box.keypoints != null)
            {
                validCount = ProjectKeypoints(box, numKpts, kptsWorld, kptsValid);

                // La geometria de la pegatina filtra keypoints antes de usarlos para calibrar yaw.
                if (cubeClass == CubeClass.SensorCube && validCount >= 4)
                    isGeometricValid = ValidateEdgeLengths(kptsWorld);
                else if (cubeClass == CubeClass.Mug)
                    isGeometricValid = false;   // no aplica: la politica de taza lo ignora
            }

            var kptsPixel = new Vector2[numKpts];
            if (box.keypoints != null)
                for (int i = 0; i < numKpts; i++)
                    kptsPixel[i] = box.keypoints[i];

            // VisualDetection es el paquete limpio que entiende FusionSystemManager.
            detections.Add(new VisualDetection
            {
                // Desde aqui la deteccion queda en el formato comun del sistema de fusion.
                Position = worldPos,
                Scale = worldScale,
                ColorCategory = ClassifyBoxColor(box, readableTexture),
                Class = cubeClass,
                KeypointsWorld = kptsWorld,
                KeypointWorldValid = kptsValid,
                IsGeometricValid = isGeometricValid,
                ValidKeypointCount = validCount,
                KeypointsPixel = kptsPixel,
                CameraPose = _frame.CameraPose,
                BBox = new Rect(box.position.x, box.position.y,
                                box.scale.x - box.position.x,
                                box.scale.y - box.position.y),
                Score = box.score
            });

            SpawnOrReuseDebugBox(worldPos, worldRot, worldScale);
        }

        if (isTemporary && readableTexture != null) Destroy(readableTexture);
        // Emite incluso listas vacias para que los consumidores midan ausencia de vision.
        OnVisualDetections?.Invoke(detections);
    }

    private static CubeClass ParseCubeClass(string label)
    {
        // Acepta nombres antiguos para que modelos entrenados con etiquetas previas sigan funcionando.
        if (string.IsNullOrEmpty(label)) return CubeClass.NormalCube;
        if (Enum.TryParse<CubeClass>(label, ignoreCase: true, out CubeClass parsedClass))
            return parsedClass;

        if (label.Equals("CuboNormal", StringComparison.OrdinalIgnoreCase)) return CubeClass.NormalCube;
        if (label.Equals("CuboSensor", StringComparison.OrdinalIgnoreCase)) return CubeClass.SensorCube;
        if (label.Equals("cube", StringComparison.OrdinalIgnoreCase)) return CubeClass.NormalCube;
        if (label.Equals("sensor", StringComparison.OrdinalIgnoreCase)) return CubeClass.SensorCube;

        Debug.LogWarning($"[ODV2] Unknown class: '{label}'");
        return CubeClass.NormalCube;
    }

    private int ProjectKeypoints(YoloBoxData box, int numKpts,
                                  Vector3[] kptsWorld, bool[] kptsValid)
    {
        // Si falla la profundidad del keypoint, usa la profundidad central como respaldo controlado.
        // Ese respaldo sacrifica precision local pero evita perder yaw por un pixel de profundidad malo.
        float bboxCenterX = (box.position.x + box.scale.x) * 0.5f;
        float bboxCenterY = (box.position.y + box.scale.y) * 0.5f;
        float centerDepth = SampleDepthAtPixel(new Vector2(bboxCenterX, bboxCenterY));
        const float depthTolerance = 0.08f;

        int validCount = 0;

        for (int i = 0; i < numKpts; i++)
        {
            // Si YOLO cree que el keypoint no es visible, ni se intenta proyectar.
            if (box.visibilities[i] <= keypointVisibilityThreshold)
                continue;

            bool projected = false;
            Vector3 kw = Vector3.zero;

            if (TryProjectPointToWorld(box.keypoints[i], out Vector3 kwPerPixel))
            {
                if (centerDepth > 0f)
                {
                    // Si la profundidad del keypoint difiere mucho de la caja, se usa la profundidad central.
                    float pixelDepth = Vector3.Distance(kwPerPixel, _frame.CameraPose.position);
                    if (Mathf.Abs(pixelDepth - centerDepth) < depthTolerance)
                    { kw = kwPerPixel; projected = true; }
                    else
                        projected = TryProjectPointAtDepth(box.keypoints[i], centerDepth, out kw);
                }
                else { kw = kwPerPixel; projected = true; }
            }
            else if (centerDepth > 0f)
                projected = TryProjectPointAtDepth(box.keypoints[i], centerDepth, out kw);

            if (projected) { kptsWorld[i] = kw; kptsValid[i] = true; validCount++; }
        }

        return validCount;
    }

    private bool ValidateEdgeLengths(Vector3[] kptsWorld)
    {
        // La distancia entre esquinas de pegatina rechaza keypoints mal proyectados con bajo coste.
        if (kptsWorld.Length < 4) return false;

        float topEdge = Vector3.Distance(kptsWorld[0], kptsWorld[1]);
        float bottomEdge = Vector3.Distance(kptsWorld[2], kptsWorld[3]);

        bool topOk = Mathf.Abs(topEdge - stickerSizeMeters) < geometryTolerance;
        bool bottomOk = Mathf.Abs(bottomEdge - stickerSizeMeters) < geometryTolerance;

        if ((!topOk || !bottomOk) && showDebugLogs)
            Debug.Log($"[ODV2] Geo FAIL: top={topEdge:F4} bot={bottomEdge:F4} " +
                      $"expected={stickerSizeMeters:F4}+/-{geometryTolerance:F4}");

        return topOk && bottomOk;
    }

    private void RecycleActiveBoxes()
    {
        // Pool simple para no instanciar ni destruir cajas de depuracion cada frame.
        foreach (GameObject box in _activeBoxes) { box.SetActive(false); _boxPool.Enqueue(box); }
        _activeBoxes.Clear();
    }

    private void SpawnOrReuseDebugBox(Vector3 pos, Quaternion rot, Vector3 scale)
    {
        // Muestra una caja debug reutilizando pool cuando la visualizacion esta activa.
        if (!showBoundingBoxes || boundingBoxPrefab == null) return;
        GameObject box = _boxPool.Count > 0 ? _boxPool.Dequeue() : Instantiate(boundingBoxPrefab);
        box.SetActive(true); box.transform.SetPositionAndRotation(pos, rot);
        box.transform.localScale = scale; _activeBoxes.Add(box);
    }

    private Texture2D AcquireReadableCameraTexture(out bool isTemporary)
    {
        // La clasificacion de color necesita textura legible; si llega RenderTexture, crea una copia temporal.
        isTemporary = false;
        Texture tex = _camera.GetTexture();
        if (tex == null) return null;
        if (tex is RenderTexture rt) { isTemporary = true; return CopyRenderTextureToTexture2D(rt); }
        return tex as Texture2D;
    }

    private static Texture2D CopyRenderTextureToTexture2D(RenderTexture rt)
    {
        // Copia puntual para leer pixeles en CPU; se destruye al terminar el lote.
        RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply(); RenderTexture.active = null; return tex;
    }

    private static DetectedColor ClassifyBoxColor(YoloBoxData box, Texture2D tex)
    {
        // Usa el centro de la caja: basta para cubos grandes de color y evita otro modelo auxiliar.
        if (tex == null) return DetectedColor.Unknown;
        float u = (box.position.x + box.scale.x) * 0.5f / tex.width;
        float v = 1.0f - (box.position.y + box.scale.y) * 0.5f / tex.height;
        Color pixel = tex.GetPixelBilinear(u, v);
        Color.RGBToHSV(pixel, out float hue01, out float saturation, out float value);
        float hue = hue01 * 360f;
        if (saturation < 0.25f || value < 0.2f) return DetectedColor.Unknown;
        if (hue >= 330f || hue <= 30f) return DetectedColor.Red;
        if (hue >= 80f && hue <= 160f) return DetectedColor.Green;
        if (hue >= 170f && hue <= 260f) return DetectedColor.Blue;
        if (hue >= 40f && hue <= 70f) return DetectedColor.Yellow;
        return DetectedColor.Unknown;
    }

    public bool TryProjectToWorld(float xMin, float yMin, float xMax, float yMax,
                                   out Vector3 world, out Quaternion rot, out Vector3 scale)
    {
        // Proyecta el centro con profundidad real y estima escala lanzando rayos por los bordes.
        // La matriz de profundidad esta por ojo; _eyeIndex mantiene consistente raycast y sample.
        // Si algo falla, devuelve false para descartar solo esa deteccion, no todo el lote.
        world = Vector3.zero; rot = Quaternion.identity; scale = Vector3.one;
        if (_frame.ViewProjectionMatrices == null || _frame.DepthPixels == null) return false;

        Texture cam = _camera.GetTexture();
        if (cam == null) return false;

        float centreX = (xMin + xMax) * 0.5f, centreY = (yMin + yMax) * 0.5f;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(centreX / cam.width, 1.0f - centreY / cam.height), _frame.CameraPose);

        Vector4 clip = _frame.ViewProjectionMatrices[_eyeIndex] * new Vector4(
            ray.origin.x + ray.direction.x, ray.origin.y + ray.direction.y,
            ray.origin.z + ray.direction.z, 1f);
        if (clip.w <= 0f) return false;

        Vector2 depthUV = new Vector2(clip.x, clip.y) / clip.w * 0.5f + Vector2.one * 0.5f;
        int texSize = _depth.TextureSize;
        // La textura de profundidad contiene los dos ojos seguidos; _eyeIndex elige el bloque correcto.
        int index = _eyeIndex * texSize * texSize
                  + Mathf.Clamp((int)(depthUV.y * texSize), 0, texSize - 1) * texSize
                  + Mathf.Clamp((int)(depthUV.x * texSize), 0, texSize - 1);

        float depth = _frame.DepthPixels[index];
        if (depth <= 0f || depth > 20f || float.IsInfinity(depth)) return false;

        world = ray.origin + ray.direction * depth;
        rot = Quaternion.LookRotation(world - _frame.CameraPose.position);

        Ray rayL = _camera.ViewportPointToRay(new Vector2((centreX - (xMax - xMin) * 0.5f) / cam.width, 1.0f - centreY / cam.height), _frame.CameraPose);
        Ray rayR = _camera.ViewportPointToRay(new Vector2((centreX + (xMax - xMin) * 0.5f) / cam.width, 1.0f - centreY / cam.height), _frame.CameraPose);
        Ray rayT = _camera.ViewportPointToRay(new Vector2(centreX / cam.width, 1.0f - (centreY - (yMax - yMin) * 0.5f) / cam.height), _frame.CameraPose);
        Ray rayB = _camera.ViewportPointToRay(new Vector2(centreX / cam.width, 1.0f - (centreY + (yMax - yMin) * 0.5f) / cam.height), _frame.CameraPose);

        scale = new Vector3(
            Vector3.Distance(rayL.origin + rayL.direction * depth, rayR.origin + rayR.direction * depth),
            Vector3.Distance(rayT.origin + rayT.direction * depth, rayB.origin + rayB.direction * depth),
            0.01f);
        return true;
    }

    public bool TryProjectToTablePlane(float xMin, float yMin, float xMax, float yMax,
                                       float tableWorldHeightY, out Vector3 world)
    {
        // Evita depender de profundidad cuando ya se sabe que el objeto esta sobre una mesa calibrada.
        // Se usa para objetos bloqueados a mesa: un rayo desde el pixel inferior corta el plano de mesa.
        world = Vector3.zero;
        Texture cam = _camera != null ? _camera.GetTexture() : null;
        if (cam == null) return false;

        float tableContactX = (xMin + xMax) * 0.5f;
        float tableContactY = yMax;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(tableContactX / cam.width, 1.0f - tableContactY / cam.height),
            _frame.CameraPose);

        float dY = ray.direction.y;
        if (Mathf.Abs(dY) < 0.01f) return false;

        float t = (tableWorldHeightY - ray.origin.y) / dY;
        if (t < 0.05f) return false;

        world = ray.origin + ray.direction * t;
        world.y = tableWorldHeightY;
        return true;
    }

    public bool TryProjectPointToWorld(Vector2 pixelPos, out Vector3 worldPos)
    {
        // Los keypoints usan la misma matriz de profundidad que las cajas para mantener el ojo consistente.
        worldPos = Vector3.zero;
        if (_frame.ViewProjectionMatrices == null) return false;
        Texture cam = _camera.GetTexture();
        if (cam == null) return false;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(pixelPos.x / cam.width, 1.0f - pixelPos.y / cam.height), _frame.CameraPose);

        Vector4 clip = _frame.ViewProjectionMatrices[_eyeIndex] * new Vector4(
            ray.origin.x + ray.direction.x, ray.origin.y + ray.direction.y,
            ray.origin.z + ray.direction.z, 1f);
        if (clip.w <= 0f) return false;

        Vector2 depthUV = new Vector2(clip.x, clip.y) / clip.w * 0.5f + Vector2.one * 0.5f;
        int texSize = _depth.TextureSize;
        int index = _eyeIndex * texSize * texSize
                  + Mathf.Clamp((int)(depthUV.y * texSize), 0, texSize - 1) * texSize
                  + Mathf.Clamp((int)(depthUV.x * texSize), 0, texSize - 1);

        float depth = _frame.DepthPixels[index];
        // El limite de 20 m rechaza lecturas espurias que romperian la escala del objeto.
        if (depth <= 0f || depth > 20f || float.IsInfinity(depth)) return false;
        worldPos = ray.origin + ray.direction * depth;
        return true;
    }

    public bool TryProjectPointAtDepth(Vector2 pixelPos, float sharedDepth, out Vector3 worldPos)
    {
        // Respaldo para keypoints: usa el rayo exacto del pixel con la profundidad central de la caja.
        worldPos = Vector3.zero;
        if (sharedDepth <= 0f || sharedDepth > 20f) return false;
        Texture cam = _camera.GetTexture();
        if (cam == null) return false;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(pixelPos.x / cam.width, 1.0f - pixelPos.y / cam.height), _frame.CameraPose);
        worldPos = ray.origin + ray.direction * sharedDepth;
        return true;
    }

    public float SampleDepthAtPixel(Vector2 pixelPos)
    {
        // Devuelve -1 si la profundidad no sirve, dejando al llamador elegir respaldo.
        if (_frame.ViewProjectionMatrices == null || _frame.DepthPixels == null) return -1f;
        Texture cam = _camera.GetTexture();
        if (cam == null) return -1f;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(pixelPos.x / cam.width, 1.0f - pixelPos.y / cam.height), _frame.CameraPose);

        Vector4 clip = _frame.ViewProjectionMatrices[_eyeIndex] * new Vector4(
            ray.origin.x + ray.direction.x, ray.origin.y + ray.direction.y,
            ray.origin.z + ray.direction.z, 1f);
        if (clip.w <= 0f) return -1f;

        Vector2 depthUV = new Vector2(clip.x, clip.y) / clip.w * 0.5f + Vector2.one * 0.5f;
        int texSize = _depth.TextureSize;
        int index = _eyeIndex * texSize * texSize
                  + Mathf.Clamp((int)(depthUV.y * texSize), 0, texSize - 1) * texSize
                  + Mathf.Clamp((int)(depthUV.x * texSize), 0, texSize - 1);

        float depth = _frame.DepthPixels[index];
        return (depth <= 0f || depth > 20f || float.IsInfinity(depth)) ? -1f : depth;
    }

    private sealed class RendererCache : MonoBehaviour
    {
        // Cache local de renderers para evitar buscarlos repetidamente en hijos.
        public Renderer[] Renderers;

        // Captura los renderers al crear el objeto de debug.
        private void Awake() => Renderers = GetComponentsInChildren<Renderer>(true);
    }
}
