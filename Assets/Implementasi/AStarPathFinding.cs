using UnityEngine;
using System.Collections.Generic;

public class AStarPathfinding : MonoBehaviour
{
    private GridMap gridMap;
    
    // Use class instead of struct to avoid cyclic reference error
    private class PathNode
    {
        public int x;
        public int z;
        public int gCost; // Cost from start
        public int hCost; // Heuristic cost to target
        public int fCost => gCost + hCost;
        public PathNode parent;
    }
    
    void Start()
    {
        gridMap = GetComponent<GridMap>();
        if (gridMap == null)
        {
            gridMap = FindObjectOfType<GridMap>();
            if (gridMap == null)
            {
                Debug.LogError("AStarPathfinding: GridMap reference not set!");
            }
        }
    }
    
    // Find path from start to target
    public List<Vector3> FindPath(Vector3 startWorldPos, Vector3 targetWorldPos)
    {
        int startX, startZ, targetX, targetZ;
        gridMap.GetXZ(startWorldPos, out startX, out startZ);
        gridMap.GetXZ(targetWorldPos, out targetX, out targetZ);
        
        Debug.Log($"Finding path from ({startX}, {startZ}) to ({targetX}, {targetZ})");
        
        List<PathNode> openList = new List<PathNode>();
        HashSet<string> closedSet = new HashSet<string>();
        
        PathNode startNode = new PathNode
        {
            x = startX,
            z = startZ,
            gCost = 0,
            hCost = CalculateHCost(startX, startZ, targetX, targetZ),
            parent = null
        };
        
        openList.Add(startNode);
        
        while (openList.Count > 0)
        {
            // Get node with lowest fCost
            PathNode currentNode = openList[0];
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].fCost < currentNode.fCost ||
                    (openList[i].fCost == currentNode.fCost && openList[i].hCost < currentNode.hCost))
                {
                    currentNode = openList[i];
                }
            }
            
            openList.Remove(currentNode);
            string nodeKey = $"{currentNode.x},{currentNode.z}";
            closedSet.Add(nodeKey);
            
            // Check if reached target
            if (currentNode.x == targetX && currentNode.z == targetZ)
            {
                return ReconstructPath(currentNode);
            }
            
            // Check neighbors (8 directions)
            foreach (Vector2Int dir in new List<Vector2Int>
            {
                new Vector2Int(0, 1),   // Up
                new Vector2Int(1, 0),   // Right
                new Vector2Int(0, -1),  // Down
                new Vector2Int(-1, 0),  // Left
                new Vector2Int(1, 1),   // Up-Right
                new Vector2Int(-1, 1),  // Up-Left
                new Vector2Int(1, -1),  // Down-Right
                new Vector2Int(-1, -1)  // Down-Left
            })
            {
                int neighborX = currentNode.x + dir.x;
                int neighborZ = currentNode.z + dir.y;
                string neighborKey = $"{neighborX},{neighborZ}";
                
                // Skip if outside grid or not walkable or in closed set
                if (neighborX < 0 || neighborZ < 0 || 
                    neighborX >= gridMap.width || neighborZ >= gridMap.height ||
                    !IsWalkable(neighborX, neighborZ) ||
                    closedSet.Contains(neighborKey))
                {
                    continue;
                }
                
                // Calculate movement cost (diagonal movement costs more)
                int moveCost = (dir.x != 0 && dir.y != 0) ? 14 : 10; // Diagonal vs straight
                int newGCost = currentNode.gCost + moveCost;
                
                // Find if neighbor is in open list
                PathNode existingNode = null;
                foreach (PathNode node in openList)
                {
                    if (node.x == neighborX && node.z == neighborZ)
                    {
                        existingNode = node;
                        break;
                    }
                }
                
                if (existingNode == null || newGCost < existingNode.gCost)
                {
                    // Create or update node
                    PathNode neighborNode = new PathNode
                    {
                        x = neighborX,
                        z = neighborZ,
                        gCost = newGCost,
                        hCost = CalculateHCost(neighborX, neighborZ, targetX, targetZ),
                        parent = currentNode
                    };
                    
                    if (existingNode != null)
                    {
                        openList.Remove(existingNode);
                    }
                    
                    openList.Add(neighborNode);
                }
            }
        }
        
        Debug.LogWarning("No path found!");
        // No path found - return direct line to target as fallback
        return new List<Vector3> { startWorldPos, targetWorldPos };
    }
    
    private int CalculateHCost(int x1, int z1, int x2, int z2)
    {
        // Manhattan distance with diagonal movement allowed
        int dx = Mathf.Abs(x2 - x1);
        int dz = Mathf.Abs(z2 - z1);
        return 10 * (dx + dz) + (14 - 2 * 10) * Mathf.Min(dx, dz);
    }
    
    private bool IsWalkable(int x, int z)
    {
        GridMap.CellType cellType = gridMap.GetCellType(x, z);
        return cellType == GridMap.CellType.Empty || 
               cellType == GridMap.CellType.Visited || 
               cellType == GridMap.CellType.LowObstacle || 
               cellType == GridMap.CellType.Unknown || 
               cellType == GridMap.CellType.BombDetected;
    }
    
    private List<Vector3> ReconstructPath(PathNode endNode)
    {
        List<Vector3> path = new List<Vector3>();
        PathNode currentNode = endNode;
        
        while (currentNode != null)
        {
            Vector3 worldPos = gridMap.GetWorldPosition(currentNode.x, currentNode.z) + 
                              new Vector3(gridMap.cellSize/2, 0, gridMap.cellSize/2); // Center of cell
            path.Add(worldPos);
            currentNode = currentNode.parent;
        }
        
        path.Reverse(); // Start to end
        
        // Debug path
        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i+1], Color.blue, 3f);
        }
        
        Debug.Log($"Path found with {path.Count} waypoints");
        return path;
    }
}