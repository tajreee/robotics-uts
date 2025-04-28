using UnityEngine;
using System.Collections.Generic;

public class BombDetector : MonoBehaviour
{
    public float detectionRange = 5f;
    public LayerMask bombLayer;
    public GridMap gridMap;
    public int totalBombs = 3; // Total number of bombs to find
    
    private List<GameObject> detectedBombs = new List<GameObject>();
    
    void Start()
    {
        if (gridMap == null)
        {
            gridMap = FindObjectOfType<GridMap>();
            if (gridMap == null)
            {
                Debug.LogError("BombDetector: GridMap reference not set!");
            }
        }
        
        // If bombLayer is not set, try to find the layer with bombs
        if (bombLayer == 0)
        {
            bombLayer = LayerMask.GetMask("Default");
        }
    }
    
    void Update()
    {
        DetectBombs();
    }
    
    private void DetectBombs()
    {
        // Cast rays in multiple directions to detect bombs
        for (int i = 0; i < 36; i++)
        {
            float angle = i * 10; // 36 directions, 10 degrees apart
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            
            Debug.DrawRay(transform.position, direction * detectionRange, Color.yellow, 0.1f);
            
            RaycastHit[] hits = Physics.RaycastAll(transform.position, direction, detectionRange, bombLayer);
            foreach (RaycastHit hit in hits)
            {
                if (hit.collider.CompareTag("Bombs"))
                {
                    // Check if this bomb was already detected
                    if (!detectedBombs.Contains(hit.collider.gameObject))
                    {
                        Debug.Log($"Bomb detected at {hit.point}!");
                        // New bomb detected!
                        detectedBombs.Add(hit.collider.gameObject);
                        RecordBombLocation(hit.collider.gameObject);
                    }
                }
            }
        }
    }
    
    private void RecordBombLocation(GameObject bomb)
    {
        Vector3 bombPosition = bomb.transform.position;
        
        // Update grid map to mark bomb location
        int bombX, bombZ;
        gridMap.GetXZ(bombPosition, out bombX, out bombZ);
        gridMap.SetCellType(bombX, bombZ, GridMap.CellType.BombDetected);
        
        Debug.Log($"Bomb detected at position: {bombPosition}. Grid coordinates: {bombX}, {bombZ}");
        
        // Notify any listeners that a bomb was found
        SendMessage("OnBombDetected", bomb, SendMessageOptions.DontRequireReceiver);
    }
    
    // Check if all bombs have been found
    public bool AreAllBombsFound()
    {
        return detectedBombs.Count >= totalBombs;
    }
    
    // Get number of detected bombs
    public int GetDetectedBombCount()
    {
        return detectedBombs.Count;
    }
    
    // Draw detection range in editor
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, detectionRange);
    }
}