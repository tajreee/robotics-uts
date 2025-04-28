using UnityEngine;
using System.Collections;

public class RobotOdometry : MonoBehaviour
{
    public DifferentialDriveController driveController;
    
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private Vector3 currentPosition;
    private Quaternion currentRotation;
    
    private float distanceTraveled = 0f;
    
    void Start()
    {
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        currentPosition = initialPosition;
        currentRotation = initialRotation;
    }
    
    void Update()
    {
        // Calculate displacement since last frame
        Vector3 displacement = transform.position - currentPosition;
        distanceTraveled += displacement.magnitude;
        
        // Update current position and rotation
        currentPosition = transform.position;
        currentRotation = transform.rotation;
        
        // You can use this data for odometry calculations or position estimation
    }
    
    // Get relative position from start
    public Vector3 GetRelativePosition()
    {
        return currentPosition - initialPosition;
    }
    
    // Get total distance traveled
    public float GetDistanceTraveled()
    {
        return distanceTraveled;
    }
}