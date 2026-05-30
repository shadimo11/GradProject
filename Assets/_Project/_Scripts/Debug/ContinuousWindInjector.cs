using UnityEngine;
using System;

[RequireComponent(typeof(Rigidbody))]
public class ContinuousWindInjector : MonoBehaviour
{
    [Header("SIL Determinism")]
    [Tooltip("Fixed seed ensures the exact same wind profile every simulation run.")]
    public int randomSeed = 42;
    private System.Random prng;

    [Header("Stochastic Wind Parameters (Gauss-Markov)")]
    public float meanWindSpeed = 2.0f;       // Constant baseline wind (m/s)
    public Vector3 windDirection = new Vector3(1, 0, 0);
    public float turbulenceIntensity = 1.5f; // sigma (m/s)
    public float timeConstant = 2.0f;        // tau = L / V (seconds)

    [Header("Aerodynamic Plant Parameters")]
    [Tooltip("Lumped Drag Coefficient: 0.5 * rho * Cd * Area")]
    public float lumpedDragCoefficient = 0.05f;
    [Tooltip("Offset of Center of Pressure from Center of Mass (m)")]
    public Vector3 centerOfPressureOffset = new Vector3(0.0f, 0.05f, 0.0f); // e.g., slightly above CoM

    private Rigidbody rb;
    private Vector3 currentTurbulence = Vector3.zero;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        prng = new System.Random(randomSeed);
        windDirection = windDirection.normalized;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Generate Gaussian White Noise using Box-Muller Transform
        Vector3 whiteNoise = new Vector3(
            GenerateGaussianNoise(),
            GenerateGaussianNoise(),
            GenerateGaussianNoise()
        );

        // 2. Compute Discrete Gauss-Markov Difference Equation
        float decay = 1.0f - (dt / timeConstant);
        float noiseGain = turbulenceIntensity * Mathf.Sqrt((2.0f * dt) / timeConstant);

        currentTurbulence.x = (decay * currentTurbulence.x) + (noiseGain * whiteNoise.x);
        currentTurbulence.y = (decay * currentTurbulence.y) + (noiseGain * whiteNoise.y);
        currentTurbulence.z = (decay * currentTurbulence.z) + (noiseGain * whiteNoise.z);

        // 3. Construct total wind vector
        Vector3 totalWindVelocity = (windDirection * meanWindSpeed) + currentTurbulence;

        // 4. Calculate Aerodynamic Force (F = Kd * |V| * V)
        Vector3 dragForce = lumpedDragCoefficient * totalWindVelocity.magnitude * totalWindVelocity;

        // 5. Calculate Aerodynamic Moment (M = r x F)
        // Convert CoP offset from local body space to world space for the cross product
        Vector3 worldCoPOffset = transform.TransformDirection(centerOfPressureOffset);
        Vector3 aerodynamicTorque = Vector3.Cross(worldCoPOffset, dragForce);

        // 6. Apply strictly via ForceMode.Force (Continuous)
        rb.AddForce(dragForce, ForceMode.Force);
        rb.AddTorque(aerodynamicTorque, ForceMode.Force);
    }

    // Box-Muller Transform to map Uniform(0,1) to Normal(0,1)
    private float GenerateGaussianNoise()
    {
        // Prevent exact zero to avoid Log(0) exception
        double u1 = 1.0 - prng.NextDouble();
        double u2 = 1.0 - prng.NextDouble();

        double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
        return (float)randStdNormal;
    }
}