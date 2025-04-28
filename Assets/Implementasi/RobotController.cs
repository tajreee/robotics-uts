using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RobotController : MonoBehaviour
{
    public enum RobotState { 
        Initializing, 
        Exploring, 
        PathPlanning,
        NavigatingToFrontier, 
        BombDetected, 
        MissionComplete 
    }
    
    [Header("References")]
    public DifferentialDriveController driveController;
    public GridMap gridMap;
    public FrontierExplorer frontierExplorer;
    public AStarPathfinding pathfinding;
    public BombDetector bombDetector;
    
    [Header("Navigation Settings")]
    public float updateRate = 2.0f; // How often to update exploration logic (seconds)
    public float arrivalThreshold = 0.5f; // How close to target before considering "arrived"
    public float stuckTimeout = 1.0f; // Increase from 5.0f to give more time before considering robot stuck
    public bool debugMode = true;
    
    [Header("Status")]
    public RobotState currentState = RobotState.Initializing;
    public string statusMessage = "Initializing...";
    public Vector3 targetFrontier;
    
    private List<Vector3> currentPath = new List<Vector3>();
    private int currentPathIndex = 0;
    private float lastUpdateTime = 0f;
    private float lastMovementTime = 0f;
    private Vector3 lastPosition;
    private float totalDistance = 0f;
    private float missionStartTime;

    private float stuckRotationTimeout = 0f;
    private bool forceTurnDirection = false;
    
    void Start()
    {
        // Find components if not set
        if (driveController == null) driveController = GetComponent<DifferentialDriveController>();
        if (gridMap == null) gridMap = FindObjectOfType<GridMap>();
        if (frontierExplorer == null) frontierExplorer = GetComponent<FrontierExplorer>();
        if (pathfinding == null) pathfinding = GetComponent<AStarPathfinding>();
        if (bombDetector == null) bombDetector = GetComponent<BombDetector>();
        
        // Check if all components are available
        if (driveController == null || gridMap == null || 
            frontierExplorer == null || pathfinding == null || bombDetector == null)
        {
            Debug.LogError("RobotController: Some required components are missing!");
            return; // Prevent further execution if components are missing
        }
        
        // Configure drive controller
        if (driveController != null)
        {
            driveController.maxSpeed = 5.0f; // Adjust speed to be more manageable
        }
        
        // Initialize tracking variables
        lastPosition = transform.position;
        lastUpdateTime = Time.time;
        lastMovementTime = Time.time;
        missionStartTime = Time.time;
        
        // Start initialization sequence
        StartCoroutine(InitializationSequence());
}
    
    IEnumerator InitializationSequence()
    {
        statusMessage = "Initializing...";
        
        // Wait a short time for Unity physics to stabilize
        yield return new WaitForSeconds(1.0f);
        
        // Initialize the grid
        int robotX, robotZ;
        gridMap.GetXZ(transform.position, out robotX, out robotZ);
        gridMap.SetCellType(robotX, robotZ, GridMap.CellType.Visited);
        
        // Do a 360-degree scan of the environment
        statusMessage = "Scanning environment...";
        yield return StartCoroutine(ScanEnvironment());
        
        // Transition to exploring state
        statusMessage = "Starting exploration...";
        currentState = RobotState.Exploring;
        
        Debug.Log("Robot initialized and ready for exploration");
    }
    
    IEnumerator ScanEnvironment()
    {
        // Rotate in place to scan environment
        for (int i = 0; i < 4; i++)  // 4 quarters
        {
            driveController.SetRobotVelocity(0, 90); // 90 degrees per second
            yield return new WaitForSeconds(1.0f);
            
            // Update sensor data
            UpdateGridMap();
        }
        
        // Stop rotation
        driveController.SetRobotVelocity(0, 0);
        yield return new WaitForSeconds(0.5f);
    }
    
    void Update()
    {
        // Calculate distance traveled
        float distanceMoved = Vector3.Distance(transform.position, lastPosition);
        if (distanceMoved > 0.01f)
        {
            totalDistance += distanceMoved;
            lastPosition = transform.position;
            lastMovementTime = Time.time;
        }
        
        // Check if robot is stuck
        if (currentState == RobotState.NavigatingToFrontier && 
            Time.time - lastMovementTime > stuckTimeout)
        {
            HandleStuckRobot();
        }

        RaycastHit hitInfo;
        if (Physics.Raycast(transform.position, transform.forward, out hitInfo, 2.0f))
        {
            Debug.LogWarning($"Object detected in front: {hitInfo.collider.name} at {hitInfo.point}, distance: {hitInfo.distance}");
            Debug.DrawLine(transform.position, hitInfo.point, Color.red, 0.5f);
        }
        
        // Update the grid map based on current sensor data
        UpdateGridMap();
        
        // Display status
        // Di metode Update() RobotController.cs
        if (debugMode)
        {
            // Visualize robot's current direction
            Debug.DrawRay(transform.position, transform.forward * 3, Color.blue, 0.1f);
            
            // Visualize path if available
            if (currentPath != null && currentPath.Count > 0 && currentPathIndex < currentPath.Count)
            {
                Debug.DrawLine(transform.position, currentPath[currentPathIndex], Color.yellow, 0.1f);
            }
        }
    }
    
    void FixedUpdate()
    {
        // State machine for robot behavior
        switch (currentState)
        {
            case RobotState.Initializing:
                // Handled by coroutine
                break;
                
            case RobotState.Exploring:
                // Update exploration logic at specified rate
                if (Time.time - lastUpdateTime > 1.0f / updateRate)
                {
                    lastUpdateTime = Time.time;
                    FindNewFrontier();
                }
                break;
                
            case RobotState.PathPlanning:
                // State for calculating path to frontier
                statusMessage = "Planning path...";
                CalculatePathToFrontier();
                break;
                
            case RobotState.NavigatingToFrontier:
                // Navigate along the calculated path
                NavigateAlongPath();
                break;
                
            case RobotState.BombDetected:
                // Handle bomb detection
                statusMessage = "BOMB DETECTED! Reporting location.";
                driveController.SetRobotVelocity(0, 0); // Stop moving
                
                // Here you would add code to notify external systems
                // For example, sending coordinates to a command center
                
                // After reporting, continue exploration
                StartCoroutine(WaitAndContinueExploration(3.0f));
                break;
                
            case RobotState.MissionComplete:
                // Mission complete logic
                statusMessage = "Mission complete! Area fully explored.";
                driveController.SetRobotVelocity(0, 0); // Stop the robot
                
                // Display mission statistics
                float missionTime = Time.time - missionStartTime;
                Debug.Log($"Mission completed in {missionTime:F1} seconds");
                Debug.Log($"Total distance traveled: {totalDistance:F2} meters");
                Debug.Log($"Area mapped: {gridMap.GetExplorationPercentage():F1}%");
                Debug.Log($"Bombs found: {bombDetector.GetDetectedBombCount()}/{bombDetector.totalBombs}");
                break;
        }
    }
    
    void UpdateGridMap()
    {
        // Update current position on grid
        int robotX, robotZ;
        gridMap.GetXZ(transform.position, out robotX, out robotZ);
        gridMap.SetCellType(robotX, robotZ, GridMap.CellType.Visited);
        
        // Simulate sensors by casting rays in different directions
        Vector3 position = transform.position;
        float sensorRange = 5.0f; // How far the sensors can detect
        
        for (float angle = 0; angle < 360; angle += 10)
        {
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            Ray ray = new Ray(position + Vector3.up * 0.2f, direction);
            RaycastHit hit;
            
            if (Physics.Raycast(ray, out hit, sensorRange))
            {
                // Something detected - determine if it's an obstacle based on height
                if (hit.collider.gameObject.layer == LayerMask.NameToLayer("Default"))
                {
                    // Check height of hit point relative to ground
                    float heightAboveGround = hit.point.y - transform.position.y;
                    
                    int hitX, hitZ;
                    gridMap.GetXZ(hit.point, out hitX, out hitZ);
                    
                    if (heightAboveGround > 0.5f) {
                        // High obstacle
                        gridMap.SetCellType(hitX, hitZ, GridMap.CellType.Obstacle);
                    } else {
                        // Low obstacle
                        gridMap.SetCellType(hitX, hitZ, GridMap.CellType.LowObstacle);
                    }
                }
                
                // Mark cells along the ray as empty until we hit something
                for (float dist = 0.5f; dist < hit.distance; dist += 0.5f)
                {
                    Vector3 pointOnRay = position + direction * dist;
                    int cellX, cellZ;
                    gridMap.GetXZ(pointOnRay, out cellX, out cellZ);
                    if (gridMap.GetCellType(cellX, cellZ) == GridMap.CellType.Unknown)
                    {
                        gridMap.SetCellType(cellX, cellZ, GridMap.CellType.Empty);
                    }
                }
            }
            else
            {
                // Nothing detected within range, mark cells along ray as empty
                for (float dist = 0.5f; dist < sensorRange; dist += 0.5f)
                {
                    Vector3 pointOnRay = position + direction * dist;
                    int cellX, cellZ;
                    gridMap.GetXZ(pointOnRay, out cellX, out cellZ);
                    if (gridMap.GetCellType(cellX, cellZ) == GridMap.CellType.Unknown)
                    {
                        gridMap.SetCellType(cellX, cellZ, GridMap.CellType.Empty);
                    }
                }
            }
        }
    }
    
    void FindNewFrontier()
    {
        statusMessage = "Searching for frontier...";
        
        // Check if exploration is complete
        float explorationPercentage = gridMap.GetExplorationPercentage();
        if (explorationPercentage > 95.0f || 
            (bombDetector.AreAllBombsFound() && explorationPercentage > 70.0f))
        {
            currentState = RobotState.MissionComplete;
            return;
        }
        
        // Find the nearest frontier point
        Vector2Int frontierPoint = frontierExplorer.GetNearestFrontier(transform.position);
        
        if (frontierPoint.x >= 0 && frontierPoint.y >= 0)
        {
            // We found a frontier to explore
            statusMessage = "Frontier found! Planning path...";
            targetFrontier = gridMap.GetWorldPosition(frontierPoint.x, frontierPoint.y);
            targetFrontier.y = transform.position.y; // Keep same height as robot
            currentState = RobotState.PathPlanning;
        }
        else
        {
            // No frontier found, continue exploring or try different strategy
            statusMessage = "No frontier found. Searching...";
            // Rotate in place to scan for new areas
            driveController.SetRobotVelocity(0, 45); // Rotate slowly
            StartCoroutine(WaitAndContinueExploration(2.0f));
        }
    }
    
    void CalculatePathToFrontier()
    {
        // Get current and target positions on grid
        int startX, startZ, targetX, targetZ;
        gridMap.GetXZ(transform.position, out startX, out startZ);
        gridMap.GetXZ(targetFrontier, out targetX, out targetZ);
        
        // Calculate path
        List<Vector3> path = pathfinding.FindPath(transform.position, targetFrontier);
        
        if (path != null && path.Count > 0)
        {
            // Set the path
            currentPath = path;
            currentPathIndex = 0;
            currentState = RobotState.NavigatingToFrontier;
            statusMessage = "Navigating to frontier...";
        }
        else
        {
            // Path not found, try a different frontier
            statusMessage = "Path to frontier not found.";
            currentState = RobotState.Exploring;
        }
    }
    
    void NavigateAlongPath()
    {
        if (currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
        {
            Debug.LogWarning("No valid path points to follow!");
            // Reset velocity and go back to exploring
            driveController.SetRobotVelocity(0, 0);
            currentState = RobotState.Exploring;
            return;
        }
        
        // Get current target point in path
        Vector3 targetPoint = currentPath[currentPathIndex];
        
        // Calculate direction and distance to target
        Vector3 directionToTarget = targetPoint - transform.position;
        directionToTarget.y = 0; // Ignore height difference
        float distanceToTarget = directionToTarget.magnitude;
        
        // Debug info - show what's happening
        Debug.Log($"Path point {currentPathIndex}/{currentPath.Count}, distance: {distanceToTarget}, target: {targetPoint}");
        
        // Enhanced obstacle detection parameters
        float obstacleDetectionDistance = 3.0f; // Detect obstacles 3 units away to have space to maneuver
        int raycastCount = 8; // Check more directions for better obstacle detection
        
        // Store obstacle detection results
        bool[] obstacleDetected = new bool[raycastCount];
        float[] obstacleDistances = new float[raycastCount];
        
        // Initialize with max distances
        for (int i = 0; i < raycastCount; i++) {
            obstacleDistances[i] = obstacleDetectionDistance;
        }
        
        // Cast rays in multiple directions to detect obstacles
        Vector3 frontDirection = directionToTarget.normalized;
        for (int i = 0; i < raycastCount; i++)
        {
            // Calculate direction at evenly spaced angles (from -135° to 180° in steps)
            float angle = -135f + (270f * i / (raycastCount - 1));
            Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * transform.forward;
            
            // Draw debug ray
            Debug.DrawRay(transform.position, rayDirection * obstacleDetectionDistance, Color.yellow, 0.1f);
            
            RaycastHit hitInfo;
            if (Physics.Raycast(transform.position, rayDirection, out hitInfo, obstacleDetectionDistance))
            {
                if (hitInfo.collider.gameObject.layer == LayerMask.NameToLayer("Default"))
                {
                    Debug.LogWarning($"Obstacle detected at angle {angle}: {hitInfo.collider.name}, distance: {hitInfo.distance}");
                    Debug.DrawRay(transform.position, rayDirection * hitInfo.distance, Color.red, 0.5f);
                    
                    obstacleDetected[i] = true;
                    obstacleDistances[i] = hitInfo.distance;
                }
            }
        }
        
        // Check if we've reached the current point in the path
        if (distanceToTarget < arrivalThreshold)
        {
            Debug.Log($"Reached path point {currentPathIndex}");
            // Move to next point in path
            currentPathIndex++;
            
            if (currentPathIndex >= currentPath.Count)
            {
                // Reached end of path
                Debug.Log("Reached end of path!");
                driveController.SetRobotVelocity(0, 0);
                currentState = RobotState.Exploring;
                return;
            }
        }
        
        // Calculate the angle between robot's forward direction and target direction
        float targetAngle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        Debug.Log($"Angle to target: {targetAngle}");
        
        // Reset rotation timeout when we're making good progress
        if (Mathf.Abs(targetAngle) < 30)
        {
            stuckRotationTimeout = 0f;
        }
        
        // Obstacle avoidance logic
        float rotationSpeed = 0;
        float forwardSpeed = 0;
        
        // Check if front zone has obstacles (rays around the front)
        bool frontBlocked = obstacleDetected[raycastCount/2-1] || obstacleDetected[raycastCount/2] || obstacleDetected[raycastCount/2+1];
        
        bool anyObstacleDetected = false;
        for (int i = 0; i < obstacleDetected.Length; i++)
        {
            if (obstacleDetected[i])
            {
                anyObstacleDetected = true;
                break;
            }
        }

        // Kemudian perbaiki logika obstacle avoidance
        if (frontBlocked || anyObstacleDetected)
        {
            // Temukan arah dengan jarak bebas terbesar
            int bestDirection = -1;
            float maxClearance = 0f;
            
            for (int i = 0; i < raycastCount; i++)
            {
                if (!obstacleDetected[i] && obstacleDistances[i] > maxClearance)
                {
                    maxClearance = obstacleDistances[i];
                    bestDirection = i;
                }
            }
            
            // Jika tidak ada arah yang bebas, coba perbaiki dengan arah dengan jarak terlebar
            if (bestDirection < 0)
            {
                for (int i = 0; i < raycastCount; i++)
                {
                    if (obstacleDistances[i] > maxClearance)
                    {
                        maxClearance = obstacleDistances[i];
                        bestDirection = i;
                    }
                }
            }
            
            if (bestDirection >= 0)
            {
                // Hitung sudut ke arah terbebas (-135° to 135°)
                float clearestAngle = -135f + (270f * bestDirection / (raycastCount - 1));
                
                // Hitung sudut antara arah robot dengan arah terbebas
                float angleToTurn = Mathf.DeltaAngle(transform.eulerAngles.y, transform.eulerAngles.y + clearestAngle);
                
                Debug.Log($"Obstacle avoidance: Turning toward clearest direction at angle {clearestAngle}, clearance: {maxClearance}, angleToTurn: {angleToTurn}");
                
                // Turn toward the clearest direction dengan nilai lebih jelas
                rotationSpeed = Mathf.Sign(angleToTurn) * 60f;
                
                // Adjust forward speed based on obstacle distance dan sudut belok
                if (Mathf.Abs(angleToTurn) > 90f)
                {
                    // Jika sudut belok sangat besar, gerakan mundur dulu
                    forwardSpeed = -1.0f;
                }
                else if (Mathf.Abs(angleToTurn) > 45f)
                {
                    // Jika sudut belok besar, gerakan lambat
                    forwardSpeed = Mathf.Clamp(maxClearance * 0.2f, 0.1f, 1.0f);
                }
                else
                {
                    // Jika sudut belok kecil, gerakan proporsional dengan jarak
                    forwardSpeed = Mathf.Clamp(maxClearance - 0.5f, 0.2f, 3.0f);
                }
            }
            else
            {
                // Semua arah terblokir, mundur dan belok tajam
                forwardSpeed = -2.0f;
                rotationSpeed = 90f;
                Debug.Log("Obstacle avoidance: All directions blocked, backing up");
            }
            
            // Lebih memprioritaskan belok daripada maju ketika ada obstacle
            if (frontBlocked && Mathf.Abs(rotationSpeed) < 30f)
            {
                rotationSpeed = 60f * (Random.value > 0.5f ? 1f : -1f); // Memastikan belok cukup tajam
                forwardSpeed *= 0.5f; // Mengurangi kecepatan maju saat belok
            }
        }
    }
    
    void HandleStuckRobot()
    {
        statusMessage = "Robot appears stuck. Trying to recover...";
        
        // Log the stuck situation
        Debug.LogWarning($"Robot stuck at position {transform.position}, trying recovery");
        
        // Back up and rotate with more variation
        StartCoroutine(ImprovedRecoveryBehavior());
        
        // Reset path finding - go back to exploring
        currentPath.Clear();
        currentState = RobotState.Exploring;
    }

    IEnumerator ImprovedRecoveryBehavior()
    {
        // Stop first
        driveController.SetRobotVelocity(0, 0);
        yield return new WaitForSeconds(0.5f);
        
        // Back up with more force
        driveController.SetRobotVelocity(-1.0f, 0);
        yield return new WaitForSeconds(2.0f);
        
        // Rotate randomly - choose a direction with variation
        float rotationDirection = (Random.value > 0.5f) ? 90 : -90;
        driveController.SetRobotVelocity(0, rotationDirection);
        yield return new WaitForSeconds(2.0f);
        
        // Move forward a bit to get out of the stuck area
        driveController.SetRobotVelocity(10.0f, 0);
        yield return new WaitForSeconds(1.0f);
        
        // Stop
        driveController.SetRobotVelocity(0, 0);
        
        // Reset stuck timer
        lastMovementTime = Time.time;
        lastPosition = transform.position; // Update last position to break out of stuck state
    }
    
    IEnumerator WaitAndContinueExploration(float waitTime)
    {
        yield return new WaitForSeconds(waitTime);
        currentState = RobotState.Exploring;
    }
    
    void OnBombDetected(GameObject bomb)
    {
        // This method is called by BombDetector via SendMessage
        if (currentState != RobotState.BombDetected)
        {
            currentState = RobotState.BombDetected;
            
            // Log detection
            Debug.Log($"BOMB DETECTED at position: {bomb.transform.position}");
        }
    }
    
    void DisplayDebugInfo()
    {
        // This would be displayed in the UI or via gizmos
        string stateInfo = $"State: {currentState}\n";
        string statusInfo = $"Status: {statusMessage}\n";
        string explorationInfo = $"Explored: {gridMap.GetExplorationPercentage():F1}%\n";
        string bombInfo = $"Bombs: {bombDetector.GetDetectedBombCount()}/{bombDetector.totalBombs}\n";
        string distanceInfo = $"Distance: {totalDistance:F2}m\n";
        string timeInfo = $"Time: {(Time.time - missionStartTime):F1}s";
        
        Debug.Log(stateInfo + statusInfo + explorationInfo + bombInfo + distanceInfo + timeInfo);
    }
    
    void OnDrawGizmos()
    {
        if (!debugMode || currentPath == null || currentPath.Count == 0)
            return;
            
        // Draw the current path
        Gizmos.color = Color.yellow;
        for (int i = 0; i < currentPath.Count - 1; i++)
        {
            Gizmos.DrawLine(currentPath[i], currentPath[i + 1]);
        }
        
        // Draw current target point
        if (currentPathIndex < currentPath.Count)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(currentPath[currentPathIndex], 0.2f);
        }
        
        // Draw frontier target if available
        if (targetFrontier != Vector3.zero)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(targetFrontier, 0.3f);
        }
    }
}