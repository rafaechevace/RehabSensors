// ═══════════════════════════════════════════════════════════════════════════
//  ObjectDetectionVisualizerV2.cs
//  Sensor-Fusion Tracking System — AIR Group, ESI-UCLM
// ═══════════════════════════════════════════════════════════════════════════
//
//  PURPOSE
//  -------
//  Bridge between 2D YOLO detections and 3D world-space poses. This script:
//
//    1. Listens to YoloPoseAgent.OnBoxesUpdated for 2D bounding boxes + keypoints.
//    2. Projects each detection into 3D world space using the Meta Quest depth
//       map and passthrough camera intrinsics.
//    3. Classifies each cube's colour from the camera texture (HSV heuristic).
//    4. Validates keypoint geometry (edge lengths vs. known cube size).
//    5. Emits VisualDetection structs via the OnVisualDetections event, consumed
//       by FusionSystemManager → FusionTracker.
//
//  KEYPOINT DEPTH STRATEGY
//  -----------------------
//  Per-pixel depth at cube corners often samples the table behind the cube
//  ("edge bleeding"), causing keypoints to float above the object. The hybrid
//  strategy tries per-pixel depth first; if it differs from the bounding-box
//  centre depth by more than `depthTolerance`, it falls back to shared depth
//  (all keypoints placed at the same distance as the bbox centre).
//
//  This preserves depth variation when the cube is tilted while rejecting
//  edge-bleed artefacts when keypoints land near object boundaries.
//
//  GEOMETRIC VALIDATION
//  --------------------
//  When all 4 keypoints are valid, the distances between top and bottom pairs
//  are compared to the expected edge length (stickerSizeMeters ± geometryTolerance).
//  This flag (IsGeometricValid) is passed to FusionTracker as a quality signal
//  — it does NOT gate detection emission (detections with 3+ keypoints are
//  still forwarded).
//
//  COLOUR CLASSIFICATION
//  ---------------------
//  A single pixel at the bbox centre is sampled from the camera texture and
//  classified by its HSV hue into Red / Green / Blue / Yellow / Unknown.
//  This is deliberately simple and works well under Quest passthrough lighting.
//
//  DEPENDENCIES
//  ------------
//  • YoloPoseAgent — provides 2D detections.
//  • Meta XR DepthTextureAccess — provides the depth map.
//  • Meta XR PassthroughCameraAccess — provides camera texture and intrinsics.
//
// ═══════════════════════════════════════════════════════════════════════════

using System.Collections.Generic;
using System.Linq;
using Meta.XR;
using Meta.XR.BuildingBlocks;
using Meta.XR.BuildingBlocks.AIBlocks;
using Meta.XR.EnvironmentDepth;
using UnityEngine;

// ─────────────────────────────────────────────────────────────────────────
//  Enums
// ─────────────────────────────────────────────────────────────────────────

/// <summary>Logical colour identity assigned to each rehabilitation cube.</summary>
public enum DetectedColor { Unknown, Red, Green, Blue, Yellow }

/// <summary>
/// Detection class from the YOLO model:
/// <see cref="CuboNormal"/> = plain cube (class 0),
/// <see cref="CuboSensor"/> = cube with sticker/IMU (class 1).
/// </summary>
public enum CubeClass { CuboNormal, CuboSensor }

// ─────────────────────────────────────────────────────────────────────────
//  VisualDetection — Per-detection output struct
// ─────────────────────────────────────────────────────────────────────────

/// <summary>
/// A single cube detection projected into 3D world space, ready for
/// consumption by the fusion pipeline.
///
/// <list type="bullet">
///   <item><see cref="Position"/>           — 3D world position (from depth map).</item>
///   <item><see cref="Scale"/>              — Estimated 3D bounding box size.</item>
///   <item><see cref="ColorCategory"/>      — HSV-classified colour.</item>
///   <item><see cref="Class"/>              — Plain cube or sensor cube.</item>
///   <item><see cref="KeypointsWorld"/>     — 4 keypoints in world space.</item>
///   <item><see cref="KeypointWorldValid"/> — Per-keypoint validity flag.</item>
///   <item><see cref="IsGeometricValid"/>   — True if edge lengths match expected size.</item>
///   <item><see cref="ValidKeypointCount"/> — Number of keypoints that projected successfully.</item>
///   <item><see cref="KeypointsPixel"/>     — Raw 2D pixel coordinates (for yaw fallback).</item>
///   <item><see cref="CameraPose"/>         — Camera pose at detection time.</item>
/// </list>
/// </summary>
public struct VisualDetection
{
    public Vector3 Position;
    public Vector3 Scale;
    public DetectedColor ColorCategory;
    public CubeClass Class;

    /// <summary>World-space positions for 4 keypoints (sticker corners).</summary>
    public Vector3[] KeypointsWorld;

    /// <summary>Per-keypoint validity: <c>true</c> if the keypoint projected successfully.</summary>
    public bool[] KeypointWorldValid;

    /// <summary>
    /// <c>true</c> when all 4 keypoints are valid AND edge lengths match
    /// the expected cube size within tolerance.
    /// </summary>
    public bool IsGeometricValid;

    /// <summary>Number of keypoints that were successfully projected (0–4).</summary>
    public int ValidKeypointCount;

    /// <summary>Raw 2D pixel coordinates for direct yaw calculation (bypasses depth).</summary>
    public Vector2[] KeypointsPixel;

    /// <summary>Camera pose at the moment this detection was captured.</summary>
    public Pose CameraPose;

    /// <summary>Pixel-space bounding box (xMin, yMin, width, height).</summary>
    public Rect BBox;

    /// <summary>Detection confidence score (0–1).</summary>
    public float Score;

    /// <summary>Convenience: all 4 keypoints valid AND geometry check passed.</summary>
    public bool HasUsableKeypoints =>
        KeypointWorldValid != null && KeypointWorldValid.Length == 4 &&
        KeypointWorldValid.All(v => v) && IsGeometricValid;

    /// <summary>Convenience: at least 3 keypoints valid (sufficient for partial calibration).</summary>
    public bool HasPartialKeypoints => ValidKeypointCount >= 3;
}

// ─────────────────────────────────────────────────────────────────────────
//  ObjectDetectionVisualizerV2 — 2D → 3D projection + colour classification
// ─────────────────────────────────────────────────────────────────────────

/// <summary>
/// Projects 2D YOLO detections into 3D world space using the Quest depth
/// map and passthrough camera intrinsics. Classifies cube colour via HSV
/// and validates keypoint geometry before emitting <see cref="VisualDetection"/>
/// structs to the fusion pipeline.
/// </summary>
[RequireComponent(typeof(YoloPoseAgent), typeof(DepthTextureAccess), typeof(EnvironmentDepthManager))]
public class ObjectDetectionVisualizerV2 : MonoBehaviour
{
    // ══════════════════════════════════════════════════════════════════
    //  Inspector Configuration
    // ══════════════════════════════════════════════════════════════════

    [Header("Visual Configuration")]
    [SerializeField]
    [Tooltip("Optional prefab instantiated at each detection's world position for debugging.")]
    private GameObject boundingBoxPrefab;

    [SerializeField]
    [Tooltip("Show/hide the debug bounding box prefabs.")]
    private bool showBoundingBoxes = true;

    [Tooltip("Emit debug logs from this component.")]
    public bool showDebugLogs = true;

    [Header("Geometric Validation")]
    [Tooltip("Expected physical edge length of the cube face (metres).\n" +
             "IMPORTANT: use the actual cube edge, not the sticker size!")]
    public float stickerSizeMeters = 0.03f;

    [Tooltip("Tolerance (metres) for edge-length validation.\n" +
             "Set wider than on desktop due to Quest depth map noise.")]
    public float geometryTolerance = 0.012f;

    [Header("Keypoint Detection")]
    [Tooltip("Minimum sigmoid visibility score to accept a keypoint (0–1).\n" +
             "Lower values allow more detections through at the cost of noise.")]
    public float keypointVisibilityThreshold = 0.4f;

    [Tooltip("Minimum valid keypoints to emit a sensor detection.\n" +
             "3 allows partial calibration when one corner is occluded.")]
    public int minValidKeypoints = 3;

    // ══════════════════════════════════════════════════════════════════
    //  Events
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Fired once per YOLO inference cycle with the list of 3D-projected
    /// detections. Consumed by <see cref="FusionSystemManager"/>.
    /// </summary>
    public event System.Action<List<VisualDetection>> OnVisualDetections;

    // ══════════════════════════════════════════════════════════════════
    //  Private Fields
    // ══════════════════════════════════════════════════════════════════

    private YoloPoseAgent _agent;
    private PassthroughCameraAccess _camera;
    private DepthTextureAccess _depth;

    /// <summary>Eye index (0 = left, 1 = right) matching the camera position.</summary>
    private int _eyeIndex;

    /// <summary>Cached depth + camera state captured each depth frame.</summary>
    private FrameSnapshot _frame;

    /// <summary>Active debug box instances currently visible in the scene.</summary>
    private readonly List<GameObject> _activeBoxes = new();

    /// <summary>Object pool for debug bounding box prefabs to avoid per-frame allocation.</summary>
    private readonly Queue<GameObject> _boxPool = new();

    /// <summary>
    /// Per-frame snapshot of the depth map and camera state, captured
    /// in <see cref="HandleDepthFrame"/>.
    /// </summary>
    private struct FrameSnapshot
    {
        /// <summary>Camera world-space pose at the time the depth frame was captured.</summary>
        public Pose CameraPose;

        /// <summary>Linearised depth texture pixels (one float per pixel).</summary>
        public float[] DepthPixels;

        /// <summary>View-projection matrices for each eye (indices 0 and 1).</summary>
        public Matrix4x4[] ViewProjectionMatrices;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Lifecycle
    // ══════════════════════════════════════════════════════════════════

    private void Awake()
    {
        _agent = GetComponent<YoloPoseAgent>();
        _camera = FindAnyObjectByType<PassthroughCameraAccess>();
        _depth = GetComponent<DepthTextureAccess>();
        _eyeIndex = _camera.CameraPosition == PassthroughCameraAccess.CameraPositionType.Left ? 0 : 1;
    }

    private void OnEnable()
    {
        _agent.OnBoxesUpdated += HandleDetectionBatch;
        _depth.OnDepthTextureUpdateCPU += HandleDepthFrame;
    }

    private void OnDisable()
    {
        _agent.OnBoxesUpdated -= HandleDetectionBatch;
        _depth.OnDepthTextureUpdateCPU -= HandleDepthFrame;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Depth Frame Capture
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Caches the latest depth frame and camera pose. Called by the depth
    /// system every time a new CPU-side depth texture is ready.
    /// </summary>
    private void HandleDepthFrame(DepthTextureAccess.DepthFrameData depthFrame)
    {
        _frame.CameraPose = _camera.GetCameraPose();
        _frame.DepthPixels = depthFrame.DepthTexturePixels.ToArray();
        _frame.ViewProjectionMatrices = depthFrame.ViewProjectionMatrix.ToArray();
    }

    // ══════════════════════════════════════════════════════════════════
    //  Main Detection Handler
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Processes a batch of 2D YOLO detections: projects each to 3D world
    /// space, classifies colour, validates keypoint geometry, and emits
    /// <see cref="VisualDetection"/> structs.
    /// </summary>
    private void HandleDetectionBatch(List<YoloBoxData> batch)
    {
        RecycleActiveBoxes();

        var detections = new List<VisualDetection>();
        Texture2D readableTexture = AcquireReadableCameraTexture(out bool isTemporary);

        foreach (YoloBoxData box in batch)
        {
            // Project the 2D bounding box centre to 3D using the depth map.
            if (!TryProjectToWorld(
                    box.position.x, box.position.y, box.scale.x, box.scale.y,
                    out Vector3 worldPos, out Quaternion worldRot, out Vector3 worldScale))
                continue;

            CubeClass cubeClass = box.classId == 1 ? CubeClass.CuboSensor : CubeClass.CuboNormal;
            var kptsWorld = new Vector3[4];
            var kptsValid = new bool[4];
            bool isGeometricValid = false;
            int validCount = 0;

            // ── Keypoint projection (sensor cubes only) ──────────────
            if (cubeClass == CubeClass.CuboSensor && box.keypoints != null)
            {
                validCount = ProjectKeypoints(box, kptsWorld, kptsValid);

                // Geometric validation requires all 4 keypoints.
                if (validCount == 4)
                    isGeometricValid = ValidateEdgeLengths(kptsWorld);
            }

            // Collect raw 2D pixel keypoints for direct yaw fallback.
            var kptsPixel = new Vector2[4];
            if (box.keypoints != null)
            {
                for (int i = 0; i < 4; i++)
                    kptsPixel[i] = box.keypoints[i];
            }

            detections.Add(new VisualDetection
            {
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

        if (isTemporary && readableTexture != null)
            Destroy(readableTexture);

        OnVisualDetections?.Invoke(detections);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Keypoint Projection (Hybrid Depth Strategy)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Projects up to 4 keypoints from 2D pixel coords to 3D world space
    /// using a hybrid depth strategy:
    ///
    /// <para><b>Step 1:</b> Sample per-pixel depth at the keypoint location.
    /// If the depth is close to the bbox centre depth (within
    /// <c>depthTolerance</c>), use per-pixel depth — it preserves true
    /// depth variation when the cube is tilted.</para>
    ///
    /// <para><b>Step 2:</b> If per-pixel depth is too far from centre
    /// (likely edge-bleed sampling the table), fall back to shared depth
    /// (all keypoints placed at the same distance as the bbox centre).</para>
    /// </summary>
    /// <returns>Number of successfully projected keypoints.</returns>
    private int ProjectKeypoints(YoloBoxData box, Vector3[] kptsWorld, bool[] kptsValid)
    {
        // Sample depth at bbox centre for reference.
        float bboxCenterX = (box.position.x + box.scale.x) * 0.5f;
        float bboxCenterY = (box.position.y + box.scale.y) * 0.5f;
        float centerDepth = SampleDepthAtPixel(new Vector2(bboxCenterX, bboxCenterY));

        // Tolerance: how far a keypoint's depth can deviate from centre before
        // we consider it "edge bleed". 8 cm covers cube diagonal + noise for
        // a 3 cm cube at ~50 cm distance.
        const float depthTolerance = 0.08f;

        int validCount = 0;

        for (int i = 0; i < 4; i++)
        {
            if (box.visibilities[i] <= keypointVisibilityThreshold)
                continue;

            bool projected = false;
            Vector3 kw = Vector3.zero;

            // Try per-pixel depth first.
            if (TryProjectPointToWorld(box.keypoints[i], out Vector3 kwPerPixel))
            {
                if (centerDepth > 0f)
                {
                    float pixelDepth = Vector3.Distance(kwPerPixel, _frame.CameraPose.position);
                    float depthDiff = Mathf.Abs(pixelDepth - centerDepth);

                    if (depthDiff < depthTolerance)
                    {
                        // Per-pixel depth is reasonable — use it.
                        kw = kwPerPixel;
                        projected = true;
                    }
                    else
                    {
                        // Edge bleed detected — fall back to shared depth.
                        projected = TryProjectPointAtDepth(box.keypoints[i], centerDepth, out kw);
                    }
                }
                else
                {
                    // No centre depth available — use per-pixel as-is.
                    kw = kwPerPixel;
                    projected = true;
                }
            }
            else if (centerDepth > 0f)
            {
                // Per-pixel failed entirely — use shared depth.
                projected = TryProjectPointAtDepth(box.keypoints[i], centerDepth, out kw);
            }

            if (projected)
            {
                kptsWorld[i] = kw;
                kptsValid[i] = true;
                validCount++;
            }
        }

        return validCount;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Geometric Validation
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Validates that the top edge (KP0↔KP1) and bottom edge (KP2↔KP3) of
    /// the projected keypoints match the expected cube edge length within
    /// <see cref="geometryTolerance"/>.
    /// </summary>
    /// <returns><c>true</c> if both edges match within tolerance.</returns>
    private bool ValidateEdgeLengths(Vector3[] kptsWorld)
    {
        float topEdge = Vector3.Distance(kptsWorld[0], kptsWorld[1]);
        float bottomEdge = Vector3.Distance(kptsWorld[2], kptsWorld[3]);

        bool topOk = Mathf.Abs(topEdge - stickerSizeMeters) < geometryTolerance;
        bool bottomOk = Mathf.Abs(bottomEdge - stickerSizeMeters) < geometryTolerance;

        if (!topOk || !bottomOk)
        {
            if (showDebugLogs)
                Debug.Log($"[ODV2] Geo FAIL: top={topEdge:F4} bot={bottomEdge:F4} " +
                          $"expected={stickerSizeMeters:F4}±{geometryTolerance:F4}");
            return false;
        }

        return true;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Debug Bounding Box Pool
    // ══════════════════════════════════════════════════════════════════

    /// <summary>Recycles all active debug boxes back into the pool.</summary>
    private void RecycleActiveBoxes()
    {
        foreach (GameObject box in _activeBoxes)
        {
            box.SetActive(false);
            _boxPool.Enqueue(box);
        }
        _activeBoxes.Clear();
    }

    /// <summary>Retrieves a box from the pool or instantiates a new one.</summary>
    private void SpawnOrReuseDebugBox(Vector3 pos, Quaternion rot, Vector3 scale)
    {
        if (boundingBoxPrefab == null) return;

        GameObject box = _boxPool.Count > 0 ? _boxPool.Dequeue() : Instantiate(boundingBoxPrefab);
        box.SetActive(true);
        box.transform.SetPositionAndRotation(pos, rot);
        box.transform.localScale = scale;
        _activeBoxes.Add(box);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Camera Texture Helpers
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Returns a CPU-readable version of the camera texture. If the camera
    /// provides a RenderTexture, a temporary Texture2D copy is created
    /// (caller must destroy it when <paramref name="isTemporary"/> is true).
    /// </summary>
    private Texture2D AcquireReadableCameraTexture(out bool isTemporary)
    {
        isTemporary = false;
        Texture tex = _camera.GetTexture();
        if (tex == null) return null;

        if (tex is RenderTexture rt)
        {
            isTemporary = true;
            return CopyRenderTextureToTexture2D(rt);
        }

        return tex as Texture2D;
    }

    /// <summary>Reads a RenderTexture into a new Texture2D (CPU-side).</summary>
    private static Texture2D CopyRenderTextureToTexture2D(RenderTexture rt)
    {
        RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        return tex;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Colour Classification (HSV Heuristic)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Classifies the cube colour by sampling the pixel at the bounding box
    /// centre and mapping its HSV hue to one of the four target colours.
    ///
    /// <para><b>Hue ranges (degrees):</b>
    /// Red = [330, 360] ∪ [0, 30],
    /// Yellow = [40, 70],
    /// Green = [80, 160],
    /// Blue = [170, 260].</para>
    ///
    /// <para>Returns <see cref="DetectedColor.Unknown"/> if saturation or
    /// value is too low (achromatic / dark).</para>
    /// </summary>
    private static DetectedColor ClassifyBoxColor(YoloBoxData box, Texture2D tex)
    {
        if (tex == null) return DetectedColor.Unknown;

        // Sample pixel at bbox centre using bilinear interpolation.
        float u = (box.position.x + box.scale.x) * 0.5f / tex.width;
        float v = 1.0f - (box.position.y + box.scale.y) * 0.5f / tex.height;
        Color pixel = tex.GetPixelBilinear(u, v);

        Color.RGBToHSV(pixel, out float h, out float s, out float val);
        float hue = h * 360f;

        // Reject achromatic or very dark pixels.
        if (s < 0.25f || val < 0.2f) return DetectedColor.Unknown;

        if (hue >= 330f || hue <= 30f) return DetectedColor.Red;
        if (hue >= 80f && hue <= 160f) return DetectedColor.Green;
        if (hue >= 170f && hue <= 260f) return DetectedColor.Blue;
        if (hue >= 40f && hue <= 70f) return DetectedColor.Yellow;

        return DetectedColor.Unknown;
    }

    // ══════════════════════════════════════════════════════════════════
    //  3D Projection — Bounding Box Centre
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Projects a 2D bounding box (xMin, yMin, xMax, yMax) to a 3D world
    /// position using the depth map at the box centre. Also estimates 3D
    /// scale by projecting the box edges at the same depth.
    ///
    /// <para><b>Depth lookup:</b> The centre pixel is cast as a ray through
    /// the passthrough camera, transformed into depth-map UV space via the
    /// view-projection matrix, and used to sample the depth texture.</para>
    /// </summary>
    /// <returns><c>true</c> if a valid depth was found and projection succeeded.</returns>
    public bool TryProjectToWorld(float xMin, float yMin, float xMax, float yMax,
                                   out Vector3 world, out Quaternion rot, out Vector3 scale)
    {
        world = Vector3.zero;
        rot = Quaternion.identity;
        scale = Vector3.one;

        if (_frame.ViewProjectionMatrices == null || _frame.DepthPixels == null) return false;

        Texture cam = _camera.GetTexture();
        if (cam == null) return false;

        float centreX = (xMin + xMax) * 0.5f;
        float centreY = (yMin + yMax) * 0.5f;

        // Cast a ray from the camera through the bbox centre pixel.
        Ray ray = _camera.ViewportPointToRay(
            new Vector2(centreX / cam.width, 1.0f - centreY / cam.height),
            _frame.CameraPose);

        // Transform ray direction into depth-map clip space.
        Vector4 clip = _frame.ViewProjectionMatrices[_eyeIndex] * new Vector4(
            ray.origin.x + ray.direction.x,
            ray.origin.y + ray.direction.y,
            ray.origin.z + ray.direction.z, 1f);

        if (clip.w <= 0f) return false;

        // Convert clip coords to depth-map UV [0–1].
        Vector2 depthUV = new Vector2(clip.x, clip.y) / clip.w * 0.5f + Vector2.one * 0.5f;

        // Sample depth.
        int texSize = DepthTextureAccess.TextureSize;
        int index = _eyeIndex * texSize * texSize
                  + Mathf.Clamp((int)(depthUV.y * texSize), 0, texSize - 1) * texSize
                  + Mathf.Clamp((int)(depthUV.x * texSize), 0, texSize - 1);

        float depth = _frame.DepthPixels[index];
        if (depth <= 0f || depth > 20f || float.IsInfinity(depth)) return false;

        world = ray.origin + ray.direction * depth;
        rot = Quaternion.LookRotation(world - _frame.CameraPose.position);

        // Estimate 3D scale by projecting box edges at the same depth.
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

    // ══════════════════════════════════════════════════════════════════
    //  3D Projection — Single Point (Per-Pixel Depth)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Projects a 2D pixel point to 3D world space using the depth value
    /// at that specific pixel.
    ///
    /// <para><b>Warning:</b> At object edges, the depth sample may hit the
    /// background (table) instead of the object surface. For keypoints,
    /// prefer the hybrid strategy in <see cref="ProjectKeypoints"/>.</para>
    /// </summary>
    public bool TryProjectPointToWorld(Vector2 pixelPos, out Vector3 worldPos)
    {
        worldPos = Vector3.zero;
        if (_frame.ViewProjectionMatrices == null) return false;

        Texture cam = _camera.GetTexture();
        if (cam == null) return false;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(pixelPos.x / cam.width, 1.0f - pixelPos.y / cam.height),
            _frame.CameraPose);

        Vector4 clip = _frame.ViewProjectionMatrices[_eyeIndex] * new Vector4(
            ray.origin.x + ray.direction.x,
            ray.origin.y + ray.direction.y,
            ray.origin.z + ray.direction.z, 1f);

        if (clip.w <= 0f) return false;

        Vector2 depthUV = new Vector2(clip.x, clip.y) / clip.w * 0.5f + Vector2.one * 0.5f;
        int texSize = DepthTextureAccess.TextureSize;
        int index = _eyeIndex * texSize * texSize
                  + Mathf.Clamp((int)(depthUV.y * texSize), 0, texSize - 1) * texSize
                  + Mathf.Clamp((int)(depthUV.x * texSize), 0, texSize - 1);

        float d = _frame.DepthPixels[index];
        if (d <= 0f || d > 20f || float.IsInfinity(d)) return false;

        worldPos = ray.origin + ray.direction * d;
        return true;
    }

    // ══════════════════════════════════════════════════════════════════
    //  3D Projection — Single Point (Shared Depth)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Projects a 2D keypoint to 3D using a shared depth value (typically
    /// from the bbox centre). Avoids edge-bleed artefacts by placing all
    /// keypoints at the same depth plane as the object centre.
    /// </summary>
    public bool TryProjectPointAtDepth(Vector2 pixelPos, float sharedDepth, out Vector3 worldPos)
    {
        worldPos = Vector3.zero;
        if (sharedDepth <= 0f || sharedDepth > 20f) return false;

        Texture cam = _camera.GetTexture();
        if (cam == null) return false;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(pixelPos.x / cam.width, 1.0f - pixelPos.y / cam.height),
            _frame.CameraPose);

        worldPos = ray.origin + ray.direction * sharedDepth;
        return true;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Depth Sampling (No Projection)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Returns the raw depth value (metres) at a given pixel position
    /// without projecting to world space. Used to get bbox centre depth
    /// for the hybrid keypoint strategy.
    /// Returns <c>-1</c> if no valid depth is available.
    /// </summary>
    public float SampleDepthAtPixel(Vector2 pixelPos)
    {
        if (_frame.ViewProjectionMatrices == null || _frame.DepthPixels == null) return -1f;

        Texture cam = _camera.GetTexture();
        if (cam == null) return -1f;

        Ray ray = _camera.ViewportPointToRay(
            new Vector2(pixelPos.x / cam.width, 1.0f - pixelPos.y / cam.height),
            _frame.CameraPose);

        Vector4 clip = _frame.ViewProjectionMatrices[_eyeIndex] * new Vector4(
            ray.origin.x + ray.direction.x,
            ray.origin.y + ray.direction.y,
            ray.origin.z + ray.direction.z, 1f);

        if (clip.w <= 0f) return -1f;

        Vector2 depthUV = new Vector2(clip.x, clip.y) / clip.w * 0.5f + Vector2.one * 0.5f;
        int texSize = DepthTextureAccess.TextureSize;
        int index = _eyeIndex * texSize * texSize
                  + Mathf.Clamp((int)(depthUV.y * texSize), 0, texSize - 1) * texSize
                  + Mathf.Clamp((int)(depthUV.x * texSize), 0, texSize - 1);

        float d = _frame.DepthPixels[index];
        if (d <= 0f || d > 20f || float.IsInfinity(d)) return -1f;

        return d;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Internal Utility
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Helper MonoBehaviour cached on debug-box prefab instances to avoid
    /// calling <c>GetComponentsInChildren</c> every frame.
    /// </summary>
    private sealed class RendererCache : MonoBehaviour
    {
        public Renderer[] Renderers;
        private void Awake() => Renderers = GetComponentsInChildren<Renderer>(true);
    }
}