using UnityEngine;

public abstract class ShapePolicy : ScriptableObject
{
    // Separa la geometria propia de cada objeto de la logica general de fusion.
    // Para anadir un objeto terapeutico nuevo, se cambia la politica y no el tracker.

    // --- Contrato que cada forma debe definir ---
    // Estas propiedades dicen al tracker cuantos keypoints esperar y que clases visuales corresponden.
    public abstract int KeypointCount { get; }

    public abstract int MinKeypointsForCalib { get; }

    public abstract int HighConfidenceKeypointCount { get; }

    public abstract CubeClass VisibleClass { get; }

    public abstract CubeClass OccludedClass { get; }

    // Calcula la direccion frontal sobre XZ a partir de los keypoints visibles.
    public abstract Vector3 ComputeForwardXZ(Vector3[] kpts, bool[] valid, Vector3 bodyCenter);

    public virtual bool CanCalibrateYaw(bool[] valid)
    {
        // Por defecto basta con contar keypoints, aunque cada forma puede endurecer la regla.
        if (valid == null) return false;
        int n = 0;
        for (int i = 0; i < valid.Length; i++) if (valid[i]) n++;
        return n >= MinKeypointsForCalib;
    }

    public virtual bool IsHighConfidence(bool[] valid, bool isGeometricValid)
    {
        // La recalibracion continua se reserva para detecciones completas y geometricamente coherentes.
        if (valid == null) return false;
        int n = 0;
        for (int i = 0; i < valid.Length; i++) if (valid[i]) n++;
        return n >= HighConfidenceKeypointCount && isGeometricValid;
    }

    public abstract Quaternion SnapRestingPose(Quaternion idealRot, ref Vector3 cachedDownAxis);

    // Permite ajustar el prefab a dimensiones fisicas reales cuando la politica las conoce.
    public virtual void ApplyRealDimensionScaling(Transform visualObject) { }

    public virtual float GetTableCenterYOffset(Transform visualObject, Vector3 restingDownAxis)
    {
        // Usa el renderer como referencia para que el centro quede apoyado sobre la mesa.
        Renderer rend = visualObject != null ? visualObject.GetComponentInChildren<Renderer>() : null;
        return rend != null ? visualObject.position.y - rend.bounds.min.y : 0f;
    }

    // Colores usados para depurar keypoints de forma consistente entre politicas.
    public virtual Color[] DebugKeypointColors =>
        new[] { Color.red, Color.green, Color.blue, Color.yellow };
}
