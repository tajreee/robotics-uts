using UnityEngine;
using System.Collections.Generic;

public class FrontierExplorer : MonoBehaviour
{
    public GridMap gridMap;
    public float frontierThreshold = 1.0f;
    public bool debugFrontiers = true;
    
    private List<Vector2Int> frontierCells = new List<Vector2Int>();
    private List<Vector2Int> neighborOffsets = new List<Vector2Int>
    {
        new Vector2Int(1, 0),   // right
        new Vector2Int(-1, 0),  // left
        new Vector2Int(0, 1),   // up
        new Vector2Int(0, -1),  // down
        new Vector2Int(1, 1),   // up-right
        new Vector2Int(-1, 1),  // up-left
        new Vector2Int(1, -1),  // down-right
        new Vector2Int(-1, -1)  // down-left
    };
    
    void Start()
    {
        if (gridMap == null)
        {
            gridMap = GetComponent<GridMap>();
            if (gridMap == null)
            {
                gridMap = FindObjectOfType<GridMap>();
                if (gridMap == null)
                {
                    Debug.LogError("FrontierExplorer: GridMap reference not set and couldn't be found!");
                }
            }
        }
    }
    
    // Find frontiers (boundaries between explored and unexplored areas)
    public void FindFrontiers()
    {
        frontierCells.Clear();
        
        // Start by marking the robot's current position as visited
        int robotX, robotZ;
        gridMap.GetXZ(transform.position, out robotX, out robotZ);
        gridMap.SetCellType(robotX, robotZ, GridMap.CellType.Visited);
        
        // Initially set adjacent cells as empty if they're unknown
        foreach (Vector2Int offset in neighborOffsets)
        {
            int nx = robotX + offset.x;
            int nz = robotZ + offset.y;
            
            if (nx >= 0 && nz >= 0 && nx < gridMap.width && nz < gridMap.height)
            {
                if (gridMap.GetCellType(nx, nz) == GridMap.CellType.Unknown)
                {
                    gridMap.SetCellType(nx, nz, GridMap.CellType.Empty);
                }
            }
        }
        
        // Find frontier cells
        for (int x = 0; x < gridMap.width; x++)
        {
            for (int z = 0; z < gridMap.height; z++)
            {
                if (IsFrontierCell(x, z))
                {
                    frontierCells.Add(new Vector2Int(x, z));
                }
            }
        }
        
        Debug.Log($"Found {frontierCells.Count} frontier cells");
        
        // Debug visualization
        if (debugFrontiers)
        {
            foreach (Vector2Int frontier in frontierCells)
            {
                Vector3 worldPos = gridMap.GetWorldPosition(frontier.x, frontier.y) + new Vector3(gridMap.cellSize/2, 0.1f, gridMap.cellSize/2);
                Debug.DrawRay(worldPos, Vector3.up * 2f, Color.yellow, 0.5f);
            }
        }
        
        // If no frontiers found, force create some around robot for initial exploration
        if (frontierCells.Count == 0)
        {
            Debug.Log("No frontiers found. Creating initial exploration points.");
            CreateInitialFrontiers(robotX, robotZ);
        }
    }
    
    // Create initial frontiers for exploration if none are found
    private void CreateInitialFrontiers(int robotX, int robotZ)
    {
        int[] distances = { 3, 6, 9 }; // Distances from robot to create frontiers
        
        foreach (int distance in distances)
        {
            // Create frontiers in cardinal directions
            frontierCells.Add(new Vector2Int(robotX + distance, robotZ));
            frontierCells.Add(new Vector2Int(robotX - distance, robotZ));
            frontierCells.Add(new Vector2Int(robotX, robotZ + distance));
            frontierCells.Add(new Vector2Int(robotX, robotZ - distance));
        }
        
        Debug.Log($"Created {frontierCells.Count} initial frontiers for exploration");
    }
    
    // Check if a cell is a frontier (empty cell that borders unknown territory)
    private bool IsFrontierCell(int x, int z)
    {
        GridMap.CellType cellType = gridMap.GetCellType(x, z);
        
        // A frontier cell should be empty or visited (not an obstacle)
        if (cellType != GridMap.CellType.Empty && 
            cellType != GridMap.CellType.Visited)
        {
            return false;
        }
        
        // Count unknown neighbors
        int unknownNeighbors = 0;
        foreach (Vector2Int offset in neighborOffsets)
        {
            int nx = x + offset.x;
            int nz = z + offset.y;
            
            if (nx >= 0 && nz >= 0 && nx < gridMap.width && nz < gridMap.height)
            {
                if (gridMap.GetCellType(nx, nz) == GridMap.CellType.Unknown)
                {
                    unknownNeighbors++;
                }
            }
        }
        
        // Consider a cell a frontier if it has at least 2 unknown neighbors
        return unknownNeighbors >= 2;
    }
    
    // Get the nearest frontier to explore
    public Vector2Int GetNearestFrontier(Vector3 currentPosition)
    {
        if (frontierCells.Count == 0)
        {
            FindFrontiers();
            if (frontierCells.Count == 0)
            {
                Debug.LogWarning("No frontiers available for exploration!");
                return new Vector2Int(-1, -1);
            }
        }
        
        // Get robot's grid position
        int robotX, robotZ;
        gridMap.GetXZ(currentPosition, out robotX, out robotZ);
        Vector2Int robotPos = new Vector2Int(robotX, robotZ);
        
        // Find nearest frontier
        Vector2Int nearestFrontier = frontierCells[0];
        float minDistance = Vector2Int.Distance(robotPos, nearestFrontier);
        
        foreach (Vector2Int frontier in frontierCells)
        {
            float distance = Vector2Int.Distance(robotPos, frontier);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestFrontier = frontier;
            }
        }
        
        Debug.Log($"Nearest frontier at grid ({nearestFrontier.x}, {nearestFrontier.y}), distance: {minDistance}");
        return nearestFrontier;
    }
    
    // Get frontier count
    public int GetFrontierCount()
    {
        return frontierCells.Count;
    }
}