using UnityEngine;

public class DifferentialDriveController : MonoBehaviour
{
    public float wheelRadius = 0.1f;
    public float wheelBase = 0.5f;
    public float maxSpeed = 5.0f;
    public float maxAngularSpeed = 240.0f; // degrees per second
    
    private float leftWheelSpeed = 0f;
    private float rightWheelSpeed = 0f;
    private Rigidbody rb;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            // rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }
    }
    
    // Forward Kinematics: from wheel speeds to robot velocity
    public Vector2 CalculateRobotVelocity()
    {
        float linearVelocity = (rightWheelSpeed + leftWheelSpeed) * wheelRadius / 2;
        float angularVelocity = (rightWheelSpeed - leftWheelSpeed) * wheelRadius / wheelBase;
        
        return new Vector2(linearVelocity, angularVelocity);
    }
    
    // Inverse Kinematics: from desired robot velocity to wheel speeds
    // Pastikan konversi antara angular velocity dalam degrees dan radians konsisten
    public void SetRobotVelocity(float linearVelocity, float angularVelocity)
    {
        // Convert angular velocity from degrees to radians
        float angularVelocityRad = angularVelocity * Mathf.Deg2Rad;
        
        // Calculate wheel speeds
        float wheelBaseHalf = wheelBase / 2;
        rightWheelSpeed = (linearVelocity + angularVelocityRad * wheelBaseHalf) / wheelRadius;
        leftWheelSpeed = (linearVelocity - angularVelocityRad * wheelBaseHalf) / wheelRadius;
        
        // Debug the calculation
        Debug.Log($"SetRobotVelocity - Linear: {linearVelocity}, Angular: {angularVelocity}°/s ({angularVelocityRad}rad/s), " +
                $"Resulting wheel speeds - Left: {leftWheelSpeed}, Right: {rightWheelSpeed}");
        
        // Clamp wheel speeds to maximum
        float maxWheelSpeed = maxSpeed / wheelRadius;
        if (Mathf.Abs(rightWheelSpeed) > maxWheelSpeed || Mathf.Abs(leftWheelSpeed) > maxWheelSpeed)
        {
            Debug.LogWarning($"Wheel speeds exceed maximum! Clamping from L:{leftWheelSpeed}, R:{rightWheelSpeed}");
        }
        
        rightWheelSpeed = Mathf.Clamp(rightWheelSpeed, -maxWheelSpeed, maxWheelSpeed);
        leftWheelSpeed = Mathf.Clamp(leftWheelSpeed, -maxWheelSpeed, maxWheelSpeed);
    }
    
    void FixedUpdate()
    {
        // Get velocities from wheel speeds
        Vector2 velocity = CalculateRobotVelocity();
        
        // DEBUG: Log the calculated and applied velocities
        Debug.Log($"Calculated: Linear={velocity.x}, Angular={velocity.y} rad/s, WheelSpeeds: L={leftWheelSpeed}, R={rightWheelSpeed}");
        
        // Apply movement to the robot in Unity using physics
        Vector3 movement = transform.forward * velocity.x;
        rb.velocity = movement;
        rb.angularVelocity = new Vector3(0, velocity.y, 0); // Remove conversion to Deg2Rad if already in radians
        
        // Debug movement
        if (velocity.magnitude > 0.01f)
        {
            Debug.DrawRay(transform.position, movement, Color.green, 0.1f);
            Debug.DrawRay(transform.position, transform.right * Mathf.Sign(velocity.y), Color.red, 0.1f);
        }
    }
    }