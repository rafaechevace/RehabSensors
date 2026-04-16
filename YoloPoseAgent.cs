// ═══════════════════════════════════════════════════════════════════════════
//  YoloPoseAgent.cs
//  Sensor-Fusion Tracking System — AIR Group, ESI-UCLM
// ═══════════════════════════════════════════════════════════════════════════
//
//  PURPOSE
//  -------
//  YOLOv8-Pose inference agent that detects rehabilitation cubes (with and
//  without sticker markers) in the Meta Quest passthrough camera feed and
//  extracts 2D bounding boxes plus 4 keypoints per sticker face.
//
//  EXECUTION MODES
//  ---------------
//  1. Immediate Mode — all model layers execute in a single frame. Fastest
//     throughput but causes frame-time spikes on Quest 3.
//  2. Split-Over-Frames Mode (default) — uses ScheduleIterable() to spread
//     model execution across multiple frames, executing `layersPerFrame`
//     layers per frame. Functionally equivalent to
//     UnityInferenceEngineProvider's "Split Over Frames" + "Layers Per Frame"
//     settings, but retains custom keypoint post-processing that the
//     standard NMSCompute shader does not support.
//
//  INPUT PREPROCESSING
//  -------------------
//  The passthrough camera image is letterboxed into a 640×640 RenderTexture
//  (preserving aspect ratio with black bars). Letterbox parameters (_lbScale,
//  _lbPadX, _lbPadY) are stored so that model output coordinates can be
//  reverse-mapped to original pixel coordinates during post-processing.
//
//  OUTPUT POST-PROCESSING
//  ----------------------
//  The model outputs a tensor of shape [1, 6+4*3, numAnchors] where:
//    • Rows 0–3: bbox centre_x, centre_y, width, height
//    • Rows 4–5: class logits (0 = plain cube, 1 = sensor cube with sticker)
//    • Rows 6+: keypoint data (x, y, visibility) × 4 keypoints
//
//  Post-processing applies sigmoid to class scores, reverse-letterboxes all
//  coordinates, and runs greedy NMS to produce the final detection list.
//
//  DATA FLOW
//  ---------
//  YoloPoseAgent → OnBoxesUpdated event → ObjectDetectionVisualizerV2
//    → depth projection → VisualDetection → FusionSystemManager → FusionTracker
//
//  DEPENDENCIES
//  ------------
//  • Unity Inference Engine (Sentis) — for ONNX model loading and execution.
//  • Meta XR Passthrough Camera Access — for the camera feed.
//
// ═══════════════════════════════════════════════════════════════════════════

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Meta.XR.BuildingBlocks;
using Meta.XR.BuildingBlocks.AIBlocks;

// ─────────────────────────────────────────────────────────────────────────
//  YoloBoxData — Per-detection output struct
// ─────────────────────────────────────────────────────────────────────────

/// <summary>
/// Raw detection output from the YOLOv8-Pose model, in original pixel
/// coordinates (after reverse-letterbox).
///
/// <para><b>Coordinate convention:</b>
/// <see cref="position"/> = (xMin, yMin, 0),
/// <see cref="scale"/> = (xMax, yMax, 0).
/// This encodes the bounding box corners for IoU calculation.</para>
///
/// <list type="bullet">
///   <item><see cref="label"/>        — "cube" (class 0) or "sensor" (class 1).</item>
///   <item><see cref="score"/>        — Max of the two sigmoid class scores.</item>
///   <item><see cref="keypoints"/>    — 4 keypoint pixel positions (corners of the sticker).</item>
///   <item><see cref="visibilities"/> — Sigmoid visibility score per keypoint.</item>
/// </list>
/// </summary>
[Serializable]
public struct YoloBoxData
{
    /// <summary>Top-left corner of the bounding box: (xMin, yMin, 0).</summary>
    public Vector3 position;

    /// <summary>Bottom-right corner of the bounding box: (xMax, yMax, 0).</summary>
    public Vector3 scale;

    /// <summary>Orientation placeholder (always identity in 2D detection).</summary>
    public Quaternion rotation;

    /// <summary>Human-readable class name: "cube" or "sensor".</summary>
    public string label;

    /// <summary>Class index: 0 = plain cube, 1 = sensor cube with sticker.</summary>
    public int classId;

    /// <summary>Detection confidence (max sigmoid of the two class logits).</summary>
    public float score;

    /// <summary>
    /// Four keypoint positions in original pixel coordinates.
    /// Layout: [0] top-right, [1] top-left, [2] bottom-left, [3] bottom-right
    /// of the sticker face.
    /// </summary>
    public Vector2[] keypoints;

    /// <summary>Sigmoid visibility score for each keypoint (0–1).</summary>
    public float[] visibilities;
}

// ─────────────────────────────────────────────────────────────────────────
//  YoloPoseAgent — YOLO inference MonoBehaviour
// ─────────────────────────────────────────────────────────────────────────

/// <summary>
/// Runs YOLOv8-Pose inference on the Meta Quest passthrough camera and emits
/// detected bounding boxes + keypoints via the <see cref="OnBoxesUpdated"/> event.
/// Supports both immediate and split-over-frames execution modes.
/// </summary>
public class YoloPoseAgent : MonoBehaviour
{
    // ══════════════════════════════════════════════════════════════════
    //  Inspector Configuration
    // ══════════════════════════════════════════════════════════════════

    [Header("Sentis Model")]
    [Tooltip("Drag-assign the .onnx YOLOv8-Pose model asset here.")]
    public Unity.InferenceEngine.ModelAsset yoloPoseModel;

    [Header("Inference Settings")]
    [Tooltip("Sentis backend: GPUCompute (default) or CPU.")]
    public Unity.InferenceEngine.BackendType backend = Unity.InferenceEngine.BackendType.GPUCompute;

    [Range(0, 1)]
    [Tooltip("Minimum sigmoid class score to keep a detection.")]
    public float minConfidence = 0.6f;

    [Tooltip("IoU threshold for Non-Maximum Suppression.")]
    public float nmsThreshold = 0.45f;

    [Tooltip("When false, inference is paused (useful for profiling).")]
    public bool realtimeInference = true;

    [Header("Performance — Split Over Frames")]
    [Tooltip("Spread inference across multiple frames to reduce per-frame cost.")]
    public bool splitOverFrames = true;

    [Range(1, 200)]
    [Tooltip("Model layers executed per frame when splitting.\n" +
             "Lower = smoother framerate but higher latency.\n" +
             "50 is a good starting point for Quest 3.")]
    public int layersPerFrame = 50;

    [Header("Debug")]
    public bool showDebugLogs = false;

    // ══════════════════════════════════════════════════════════════════
    //  Events
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Fired after each inference cycle with the list of NMS-filtered
    /// detections. Consumed by <see cref="ObjectDetectionVisualizerV2"/>.
    /// </summary>
    public event Action<List<YoloBoxData>> OnBoxesUpdated;

    // ══════════════════════════════════════════════════════════════════
    //  Private Fields
    // ══════════════════════════════════════════════════════════════════

    private Unity.InferenceEngine.Worker _worker;
    private Unity.InferenceEngine.Model _model;
    private Meta.XR.PassthroughCameraAccess _cam;

    /// <summary>640×640 RenderTexture used as the model input after letterboxing.</summary>
    private RenderTexture _captureRT;

    /// <summary>Model input resolution (fixed at 640 for YOLOv8).</summary>
    private const int ImageSize = 640;

    /// <summary>Prevents overlapping inference calls.</summary>
    private bool _busy = false;

    /// <summary>Cached reference to the last source texture (needed across coroutine yields).</summary>
    private Texture _lastSrcTexture;

    // ── Letterbox state (needed to reverse-map output coordinates) ────
    // When the camera image is not square, it is letterboxed into 640×640
    // with black padding bars. These values record the transform so model
    // output coordinates can be mapped back to original pixel space.

    /// <summary>Scale factor applied to the source image during letterboxing.</summary>
    private float _lbScale = 1f;

    /// <summary>Horizontal padding (pixels) added during letterboxing.</summary>
    private float _lbPadX = 0f;

    /// <summary>Vertical padding (pixels) added during letterboxing.</summary>
    private float _lbPadY = 0f;

    // ── Runtime stats (visible in Inspector) ─────────────────────────

    [Header("Runtime Stats (read-only)")]
    [SerializeField] private float _lastInferenceMs;
    [SerializeField] private int _lastDetectionCount;
    [SerializeField] private int _totalModelLayers;

    /// <summary>Wall-clock milliseconds of the last inference cycle.</summary>
    public float LastInferenceMs => _lastInferenceMs;

    /// <summary>Number of detections after NMS in the last cycle.</summary>
    public int LastDetectionCount => _lastDetectionCount;

    // ══════════════════════════════════════════════════════════════════
    //  Lifecycle
    // ══════════════════════════════════════════════════════════════════

    private void Awake()
    {
        _cam = FindAnyObjectByType<Meta.XR.PassthroughCameraAccess>();

        if (yoloPoseModel != null)
        {
            _model = Unity.InferenceEngine.ModelLoader.Load(yoloPoseModel);
            _worker = new Unity.InferenceEngine.Worker(_model, backend);
            _totalModelLayers = _model.layers.Count;

            if (showDebugLogs)
                Debug.Log($"[YoloPoseAgent] Model loaded: {_totalModelLayers} layers, backend={backend}");
        }
        else
        {
            Debug.LogError("[YoloPoseAgent] Falta asignar el modelo ONNX en el inspector.");
        }
    }

    private void Update()
    {
        if (_cam == null || !_cam.IsPlaying || _busy || _worker == null || !realtimeInference)
            return;

        if (splitOverFrames)
            StartCoroutine(RunInferenceSplit());
        else
            RunInferenceImmediate();
    }

    private void OnDestroy()
    {
        _worker?.Dispose();
        if (_captureRT != null) _captureRT.Release();
    }

    // ══════════════════════════════════════════════════════════════════
    //  Immediate Mode (all layers in one frame)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Executes the entire model in a single frame. Lowest latency but
    /// causes a frame-time spike proportional to model complexity.
    /// </summary>
    private void RunInferenceImmediate()
    {
        _busy = true;
        float t0 = Time.realtimeSinceStartup;

        try
        {
            var src = _cam.GetTexture();
            if (src == null) return;
            _lastSrcTexture = src;

            PrepareInputRT(src);

            using var inputTensor = new Unity.InferenceEngine.Tensor<float>(
                new Unity.InferenceEngine.TensorShape(1, 3, ImageSize, ImageSize));
            Unity.InferenceEngine.TextureConverter.ToTensor(
                _captureRT, inputTensor, new Unity.InferenceEngine.TextureTransform());

            _worker.Schedule(inputTensor);

            var outputTensor = _worker.PeekOutput() as Unity.InferenceEngine.Tensor<float>;
            float[] data = outputTensor.DownloadToArray();

            ProcessOutput(data, outputTensor.shape[2], src);
        }
        finally
        {
            _lastInferenceMs = (Time.realtimeSinceStartup - t0) * 1000f;
            _busy = false;
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Split-Over-Frames Mode (coroutine)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Uses <c>ScheduleIterable()</c> to spread model execution across
    /// multiple frames, advancing <see cref="layersPerFrame"/> layers per
    /// frame. Post-processing runs on the frame when execution completes.
    ///
    /// <para><b>Note on readback:</b> <c>DownloadToArray()</c> is used
    /// synchronously because <c>ReadbackAndCloneAsync()</c> returns an
    /// <c>Awaitable</c> incompatible with IEnumerator coroutines. The split
    /// execution above already distributes GPU work, so the final readback
    /// stall is minimal (just copying the small output tensor).</para>
    /// </summary>
    private IEnumerator RunInferenceSplit()
    {
        _busy = true;
        float t0 = Time.realtimeSinceStartup;

        var src = _cam.GetTexture();
        if (src == null) { _busy = false; yield break; }
        _lastSrcTexture = src;

        PrepareInputRT(src);

        var inputTensor = new Unity.InferenceEngine.Tensor<float>(
            new Unity.InferenceEngine.TensorShape(1, 3, ImageSize, ImageSize));
        Unity.InferenceEngine.TextureConverter.ToTensor(
            _captureRT, inputTensor, new Unity.InferenceEngine.TextureTransform());

        // Execute N layers per frame via ScheduleIterable.
        var schedule = _worker.ScheduleIterable(inputTensor);
        int layerCount = 0;

        while (schedule.MoveNext())
        {
            layerCount++;
            if (layerCount % layersPerFrame == 0)
                yield return null;  // Wait until next frame.
        }

        // All layers done — read output synchronously.
        var outputTensor = _worker.PeekOutput() as Unity.InferenceEngine.Tensor<float>;
        float[] data = outputTensor.DownloadToArray();
        ProcessOutput(data, outputTensor.shape[2], _lastSrcTexture);

        inputTensor.Dispose();
        _lastInferenceMs = (Time.realtimeSinceStartup - t0) * 1000f;
        _busy = false;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Input Preprocessing — Letterboxing
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Fits the source camera texture into a 640×640 RenderTexture with
    /// aspect-ratio-preserving letterboxing (black bars). Records the
    /// scale and padding so output coordinates can be reverse-mapped.
    ///
    /// <para>YOLOv8 expects letterboxed (not stretched) input — stretching
    /// would distort keypoint geometry significantly.</para>
    /// </summary>
    private void PrepareInputRT(Texture src)
    {
        // Create or recreate the RenderTexture if needed.
        if (_captureRT == null || _captureRT.width != ImageSize)
        {
            if (_captureRT != null) _captureRT.Release();
            _captureRT = new RenderTexture(ImageSize, ImageSize, 0, RenderTextureFormat.ARGB32);
        }

        float srcW = src.width;
        float srcH = src.height;

        // Scale factor: fit the longest edge into ImageSize.
        _lbScale = Mathf.Min(ImageSize / srcW, ImageSize / srcH);
        float newW = srcW * _lbScale;
        float newH = srcH * _lbScale;
        _lbPadX = (ImageSize - newW) * 0.5f;
        _lbPadY = (ImageSize - newH) * 0.5f;

        if (Mathf.Abs(srcW - srcH) < 2f)
        {
            // Already approximately square — simple blit.
            Graphics.Blit(src, _captureRT);
            _lbScale = ImageSize / srcW;
            _lbPadX = 0f;
            _lbPadY = 0f;
        }
        else
        {
            // Clear to black (letterbox bars), then draw texture in the centre.
            RenderTexture prev = RenderTexture.active;
            RenderTexture.active = _captureRT;
            GL.Clear(true, true, Color.black);

            GL.PushMatrix();
            GL.LoadPixelMatrix(0, ImageSize, ImageSize, 0);
            Graphics.DrawTexture(new Rect(_lbPadX, _lbPadY, newW, newH), src);
            GL.PopMatrix();

            RenderTexture.active = prev;
        }

        if (showDebugLogs && Time.frameCount % 300 == 0)
            Debug.Log($"[YoloPoseAgent] Letterbox: src={srcW}x{srcH} " +
                      $"scale={_lbScale:F3} pad=({_lbPadX:F1},{_lbPadY:F1})");
    }

    // ══════════════════════════════════════════════════════════════════
    //  Output Post-Processing
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Parses the raw model output tensor, applies sigmoid to class scores,
    /// reverse-letterboxes all coordinates, filters by <see cref="minConfidence"/>,
    /// runs NMS, and fires the <see cref="OnBoxesUpdated"/> event.
    ///
    /// <para><b>Tensor layout:</b> shape [1, rows, numAnchors], column-major
    /// per anchor. Rows 0–3 = bbox, 4–5 = class logits, 6+ = keypoints
    /// (x, y, visibility) × 4.</para>
    /// </summary>
    /// <param name="data">Flattened output tensor values.</param>
    /// <param name="numAnchors">Number of detection anchors (tensor dim 2).</param>
    /// <param name="src">Source camera texture (for aspect ratio context).</param>
    private void ProcessOutput(float[] data, int numAnchors, Texture src)
    {
        var proposals = new List<YoloBoxData>();

        // Reverse-letterbox factors: original_pixel = (model_pixel - pad) / _lbScale.
        float padX = _lbPadX;
        float padY = _lbPadY;
        float invScale = 1f / _lbScale;

        for (int a = 0; a < numAnchors; a++)
        {
            // Sigmoid on class logits (rows 4 and 5).
            float s0 = 1f / (1f + Mathf.Exp(-data[4 * numAnchors + a]));
            float s1 = 1f / (1f + Mathf.Exp(-data[5 * numAnchors + a]));

            float score = Mathf.Max(s0, s1);
            if (score < minConfidence) continue;

            int classId = s0 > s1 ? 0 : 1;
            string labelName = classId == 0 ? "cube" : "sensor";

            // Raw bbox in 640×640 letterboxed space.
            float cx = data[0 * numAnchors + a];
            float cy = data[1 * numAnchors + a];
            float w = data[2 * numAnchors + a];
            float h = data[3 * numAnchors + a];

            // Reverse letterbox → original pixel coordinates.
            cx = (cx - padX) * invScale;
            cy = (cy - padY) * invScale;
            w = w * invScale;
            h = h * invScale;

            var box = new YoloBoxData
            {
                position = new Vector3(cx - w / 2, cy - h / 2, 0),  // (xMin, yMin)
                rotation = Quaternion.identity,
                scale = new Vector3(cx + w / 2, cy + h / 2, 0),  // (xMax, yMax)
                label = labelName,
                classId = classId,
                score = score,
                keypoints = new Vector2[4],
                visibilities = new float[4]
            };

            // Parse 4 keypoints (each occupies 3 rows: x, y, visibility).
            for (int k = 0; k < 4; k++)
            {
                int row = 6 + (k * 3);

                float kx = (data[row * numAnchors + a] - padX) * invScale;
                float ky = (data[(row + 1) * numAnchors + a] - padY) * invScale;
                box.keypoints[k] = new Vector2(kx, ky);

                float rawVis = data[(row + 2) * numAnchors + a];
                box.visibilities[k] = 1f / (1f + Mathf.Exp(-rawVis));
            }

            proposals.Add(box);
        }

        var finalBoxes = ApplyNMS(proposals, nmsThreshold);
        _lastDetectionCount = finalBoxes.Count;

        if (showDebugLogs && finalBoxes.Count > 0)
            Debug.Log($"[YoloPoseAgent] {finalBoxes.Count} detections, " +
                      $"{_lastInferenceMs:F1}ms, split={splitOverFrames}");

        OnBoxesUpdated?.Invoke(finalBoxes);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Non-Maximum Suppression (NMS)
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Greedy NMS: sorts proposals by descending score, picks the top one,
    /// removes all proposals with IoU above <paramref name="iouThreshold"/>,
    /// and repeats.
    /// </summary>
    private static List<YoloBoxData> ApplyNMS(List<YoloBoxData> boxes, float iouThreshold)
    {
        boxes = boxes.OrderByDescending(b => b.score).ToList();
        var results = new List<YoloBoxData>();

        while (boxes.Count > 0)
        {
            var current = boxes[0];
            results.Add(current);
            boxes.RemoveAt(0);
            boxes.RemoveAll(b => CalculateIoU(current, b) > iouThreshold);
        }

        return results;
    }

    /// <summary>
    /// Computes Intersection-over-Union between two axis-aligned bounding
    /// boxes encoded as (xMin, yMin) in <c>position</c> and (xMax, yMax) in
    /// <c>scale</c>.
    /// </summary>
    private static float CalculateIoU(YoloBoxData a, YoloBoxData b)
    {
        float xA = Mathf.Max(a.position.x, b.position.x);
        float yA = Mathf.Max(a.position.y, b.position.y);
        float xB = Mathf.Min(a.scale.x, b.scale.x);
        float yB = Mathf.Min(a.scale.y, b.scale.y);

        float interArea = Mathf.Max(0, xB - xA) * Mathf.Max(0, yB - yA);
        float boxAArea = (a.scale.x - a.position.x) * (a.scale.y - a.position.y);
        float boxBArea = (b.scale.x - b.position.x) * (b.scale.y - b.position.y);

        return interArea / (boxAArea + boxBArea - interArea);
    }
}