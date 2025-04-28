using UnityEngine;
using System.Collections.Generic;

public class GridMap : MonoBehaviour
{
    public int width = 50;
    public int height = 50;
    public float cellSize = 0.5f;
    public Vector3 originPosition;
    public bool visualizeGrid = true;
    
    public enum CellType { Unknown, Empty, Obstacle, LowObstacle, Visited, BombDetected }
    
    private CellType[,] gridData;
    private Dictionary<CellType, Color> cellColors = new Dictionary<CellType, Color>() {
        { CellType.Unknown, Color.gray },
        { CellType.Empty, Color.white },
        { CellType.Obstacle, Color.black },
        { CellType.LowObstacle, Color.blue },
        { CellType.Visited, Color.green },
        { CellType.BombDetected, Color.red }
    };
    
    void Awake()
    {
        Debug.Log("GridMap initialized with dimensions: " + width + "x" + height);
        gridData = new CellType[width, height];
        
        // Set origin to robot's initial position if not specified
        if (originPosition == Vector3.zero)
        {
            originPosition = transform.position - new Vector3(width * cellSize / 2, 0, height * cellSize / 2);
        }
        
        // Initialize all cells as Unknown
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < height; z++)
            {
                gridData[x, z] = CellType.Unknown;
            }
        }
    }
    
    // Get world position from grid coordinates
    public Vector3 GetWorldPosition(int x, int z)
    {
        return new Vector3(x * cellSize, 0, z * cellSize) + originPosition;
    }
    
    // Get grid coordinates from world position
    public void GetXZ(Vector3 worldPosition, out int x, out int z)
    {
        x = Mathf.FloorToInt((worldPosition - originPosition).x / cellSize);
        z = Mathf.FloorToInt((worldPosition - originPosition).z / cellSize);
        x = Mathf.Clamp(x, 0, width - 1);
        z = Mathf.Clamp(z, 0, height - 1);
    }
    
    // Debug to check if grid position is calculated correctly
    public Vector3 DebugGridPosition(Vector3 worldPosition)
    {
        int x, z;
        GetXZ(worldPosition, out x, out z);
        Debug.Log($"World position {worldPosition} maps to grid ({x}, {z})");
        return GetWorldPosition(x, z);
    }
    
    // Set cell type at specific grid coordinates
    public void SetCellType(int x, int z, CellType type)
    {
        if (x >= 0 && z >= 0 && x < width && z < height)
        {
            gridData[x, z] = type;
            // Debug.Log($"Cell at ({x}, {z}) set to {type}");
        }
        else
        {
            Debug.LogWarning($"Attempted to set cell out of bounds: ({x}, {z})");
        }
    }
    
    // Get cell type at specific grid coordinates
    public CellType GetCellType(int x, int z)
    {
        if (x >= 0 && z >= 0 && x < width && z < height)
        {
            return gridData[x, z];
        }
        return CellType.Obstacle; // Out of bounds is considered obstacle
    }
    
    // Calculate percentage of explored area
    public float GetExplorationPercentage()
    {
        int exploredCells = 0;
        int totalCells = width * height;
        
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < height; z++)
            {
                if (gridData[x, z] != CellType.Unknown)
                {
                    exploredCells++;
                }
            }
        }
        
        float percentage = (float)exploredCells / totalCells;
        Debug.Log($"Exploration progress: {percentage * 100}% ({exploredCells}/{totalCells} cells)");
        return percentage;
    }
    
    void OnDrawGizmos()
    {
        if (!visualizeGrid || gridData == null) return;
        
        // Draw grid cells with appropriate colors
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < height; z++)
            {
                Vector3 position = GetWorldPosition(x, z) + new Vector3(cellSize / 2, 0.01f, cellSize / 2);
                Color cellColor = cellColors[gridData[x, z]];
                cellColor.a = 0.5f; // Semi-transparent
                Gizmos.color = cellColor;
                Gizmos.DrawCube(position, new Vector3(cellSize * 0.9f, 0.02f, cellSize * 0.9f));
            }
        }
        
        // Draw grid outline
        Gizmos.color = Color.black;
        Gizmos.DrawWireCube(
            originPosition + new Vector3(width * cellSize / 2, 0, height * cellSize / 2), 
            new Vector3(width * cellSize, 0.1f, height * cellSize)
        );
    }
}