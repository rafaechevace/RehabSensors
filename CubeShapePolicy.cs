using UnityEngine;

[CreateAssetMenu(menuName = "AIR/Shape Policy/Cube", fileName = "CubeShapePolicy")]
public class CubeShapePolicy : ShapePolicy
{
    // Politica del cubo instrumentado: interpreta keypoints de pegatina y mantiene apoyo por caras.
    // realDimensions permite que el prefab mida lo mismo que el cubo fisico para que mesa y colliders cuadren.
    [Header("Cube Dimensions")]
    [Tooltip("Dimensiones fisicas reales del cubo en metros. X=ancho, Y=alto, Z=fondo.\n" +
             "Ejemplo: 3.0cm x 3.0cm x 3.5cm -> (0.030, 0.030, 0.035)\n" +
             "Usa (0,0,0) para omitir el autoescalado.")]
    public Vector3 realDimensions = Vector3.zero;

    // Propiedades fijas del cubo: cuatro keypoints y clases distintas segun se vea la pegatina.
    public override int KeypointCount => 4;

    public override int MinKeypointsForCalib => 3;

    public override int HighConfidenceKeypointCount => 4;

    public override CubeClass VisibleClass => CubeClass.SensorCube;

    public override CubeClass OccludedClass => CubeClass.NormalCube;

    public override Vector3 ComputeForwardXZ(Vector3[] kpts, bool[] valid, Vector3 bodyCenter)
    {
        // La misma pieza puede aparecer con o sin pegatina visible; aqui solo se usa si hay keypoints.
        // La orientacion visible se estima en XZ para que pequenos errores de profundidad no contaminen yaw.
        int validCount = 0;
        for (int i = 0; i < 4; i++)
            if (valid != null && i < valid.Length && valid[i]) validCount++;

        if (validCount < 3) return Vector3.zero;

        Vector3[] flat = new Vector3[4];
        for (int i = 0; i < 4; i++) flat[i] = new Vector3(kpts[i].x, 0f, kpts[i].z);

        Vector3 forwardDir = Vector3.zero;
        // Los bordes opuestos de la pegatina indican hacia donde mira la cara visible.
        if (valid[2] && valid[1]) forwardDir += (flat[1] - flat[2]);
        if (valid[3] && valid[0]) forwardDir += (flat[0] - flat[3]);

        return forwardDir.sqrMagnitude > 0.0001f ? forwardDir.normalized : Vector3.zero;
    }

    public override Quaternion SnapRestingPose(Quaternion idealRot, ref Vector3 cachedDownAxis)
    {
        // El cubo debe apoyar una cara. La histeresis evita cambios de cara por ruido en los limites.
        Vector3[] localAxes = {
            Vector3.down, Vector3.up, Vector3.left, Vector3.right, Vector3.back, Vector3.forward
        };

        Vector3 currentDown = idealRot * cachedDownAxis;
        float currentDot = Vector3.Dot(currentDown, Vector3.down);

        Vector3 bestAxis = cachedDownAxis;
        float bestDot = currentDot;
        const float hysteresis = 0.15f;

        foreach (Vector3 axis in localAxes)
        {
            Vector3 worldAxis = idealRot * axis;
            float dot = Vector3.Dot(worldAxis, Vector3.down);
            if (dot > bestDot + hysteresis)
            {
                bestDot = dot;
                bestAxis = axis;
            }
        }
        cachedDownAxis = bestAxis;

        Quaternion faceDownBase = Quaternion.FromToRotation(cachedDownAxis, Vector3.down);
        Quaternion remainder = Quaternion.Inverse(faceDownBase) * idealRot;
        // Conserva el yaw del IMU y corrige el resto para que una cara quede apoyada.
        return faceDownBase * Quaternion.Euler(0f, remainder.eulerAngles.y, 0f);
    }

    public override float GetTableCenterYOffset(Transform visualObject, Vector3 restingDownAxis)
    {
        // Calcula cuanto hay que elevar el centro para que la cara inferior toque la mesa.
        float renderedOffset = base.GetTableCenterYOffset(visualObject, restingDownAxis);
        if (renderedOffset > 0.0001f) return renderedOffset;

        if (realDimensions.x <= 0f || realDimensions.y <= 0f || realDimensions.z <= 0f)
            return 0f;

        Vector3 axis = new Vector3(Mathf.Abs(restingDownAxis.x), Mathf.Abs(restingDownAxis.y), Mathf.Abs(restingDownAxis.z));

        if (axis.x >= axis.y && axis.x >= axis.z) return realDimensions.x * 0.5f;
        if (axis.z >= axis.x && axis.z >= axis.y) return realDimensions.z * 0.5f;
        return realDimensions.y * 0.5f;
    }

    public override void ApplyRealDimensionScaling(Transform visualObject)
    {
        // Ajusta el prefab a dimensiones reales para que posicion fusionada y colliders usen escala fisica.
        if (realDimensions.x <= 0 || realDimensions.y <= 0 || realDimensions.z <= 0)
        {
            Debug.LogWarning($"[CubePolicy] Scaling cancelled: realDimensions={realDimensions}");
            return;
        }

        Renderer rend = visualObject.GetComponentInChildren<Renderer>();
        if (rend == null)
        {
            Debug.LogWarning("[CubePolicy] Scaling cancelled: no Renderer found.");
            return;
        }

        Vector3 localSize = rend.localBounds.size;
        Vector3 globalScale = rend.transform.lossyScale;
        // Mide la malla ya escalada para corregir la raiz sin depender de la escala de importacion.
        Vector3 trueMeshSize = new Vector3(
            localSize.x * globalScale.x,
            localSize.y * globalScale.y,
            localSize.z * globalScale.z
        );

        if (trueMeshSize.x <= 0 || trueMeshSize.y <= 0 || trueMeshSize.z <= 0) return;

        Vector3 currentRootScale = visualObject.localScale;

        visualObject.localScale = new Vector3(
            currentRootScale.x * (realDimensions.x / trueMeshSize.x),
            currentRootScale.y * (realDimensions.y / trueMeshSize.y),
            currentRootScale.z * (realDimensions.z / trueMeshSize.z)
        );

        Debug.Log($"[CubePolicy] Scaling OK. Measured mesh {trueMeshSize}. " +
                  $"Target: {realDimensions}. New scale: {visualObject.localScale}");
    }
}
