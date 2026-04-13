using UnityEngine;

public class GPS : MonoBehaviour
{
    [Header("Settings")]
    public float updateRateHz = 5f;
    public float posNoiseMeters = 2.5f; // Standard GPS accuracy
    public bool hasFix = true;
    public int satellites = 8;

    [Header("Readings")]
    public float positionX; // For Simulink
    public float positionY; // For Simulink
    public float numSV;
    public float fixFlag;

    // Helpers for Debugging
    public Vector3 perfectPosition;
    public Vector3 noisyPosition;

    private float nextUpdate;
    private System.Random rng = new System.Random();

    void FixedUpdate() // Changed to FixedUpdate for Simulink sync
    {
        if (Time.time < nextUpdate) return;
        nextUpdate = Time.time + (1f / updateRateHz);

        perfectPosition = transform.position;

        if (hasFix)
        {
            // Generate two independent standard normal variables for 2D noise
            float u1 = 1.0f - (float)rng.NextDouble();
            float u2 = 1.0f - (float)rng.NextDouble();

            // Box-Muller transform yields two independent normals
            float z0 = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
            float z1 = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);

            float noiseX = z0 * posNoiseMeters;
            float noiseY = z1 * posNoiseMeters;

            noisyPosition = perfectPosition + new Vector3(noiseX, 0, noiseY);

            positionX = noisyPosition.x;
            positionY = noisyPosition.z; // Unity Z is Map Y
            numSV = satellites;
            fixFlag = 1.0f;
        }
        else
        {
            positionX = 0; positionY = 0; numSV = 0; fixFlag = 0;
        }
    }
}