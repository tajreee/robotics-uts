using UnityEngine;
using System.Collections.Generic; // Diperlukan untuk algoritma garis nanti

public class GridMappingSystem : MonoBehaviour
{
    // Enum untuk status setiap sel
    public enum CellStatus { Unknown, Free, Obstacle, Visited }

    [Header("Pengaturan Grid")]
    [Tooltip("Ukuran fisik area pemetaan dalam satuan dunia (meter). X=lebar, Y=panjang.")]
    public Vector2 worldSize = new Vector2(50f, 50f);
    [Tooltip("Ukuran setiap sel grid dalam satuan dunia (meter).")]
    public float cellSize = 0.5f;
    [Tooltip("Transform robot untuk melacak posisinya.")]
    public Transform robotTransform; // Seret GameObject robot ke sini di Inspector

    [Header("Pengaturan Update")]
    [Tooltip("Seberapa sering peta diperbarui berdasarkan posisi robot (detik).")]
    public float positionUpdateRate = 0.2f;

    [Header("Debugging & Visualisasi")]
    public bool showGizmos = true;
    public Color unknownColor = Color.gray;
    public Color freeColor = Color.white;
    public Color obstacleColor = Color.red;
    public Color visitedColor = Color.blue;
    public Color robotPathColor = Color.yellow; // Untuk jalur yang dilalui

    // Data Grid Internal
    private CellStatus[,] grid;
    private List<Vector2Int> robotPath = new List<Vector2Int>(); // Melacak jejak robot
    private int gridSizeX, gridSizeY;
    private Vector3 worldBottomLeft; // Posisi dunia untuk grid[0, 0]
    private float lastPositionUpdateTime;

    // Properti untuk akses dari luar (opsional)
    public int GridSizeX => gridSizeX;
    public int GridSizeY => gridSizeY;
    public float CellSize => cellSize;


    void Awake()
    {
        if (robotTransform == null)
        {
            Debug.LogError("Robot Transform belum di-assign ke GridMappingSystem!", this);
            enabled = false;
            return;
        }

        if (cellSize <= 0)
        {
             Debug.LogError("Cell Size harus lebih besar dari 0!", this);
             enabled = false;
             return;
        }

        // Hitung dimensi grid dalam jumlah sel
        gridSizeX = Mathf.RoundToInt(worldSize.x / cellSize);
        gridSizeY = Mathf.RoundToInt(worldSize.y / cellSize);

        // Hitung posisi dunia dari sudut kiri bawah grid
        // Asumsi GameObject ini berada di TENGAH area pemetaan
        worldBottomLeft = transform.position - Vector3.right * worldSize.x / 2 - Vector3.forward * worldSize.y / 2;

        // Inisialisasi grid dengan status Unknown
        grid = new CellStatus[gridSizeX, gridSizeY];
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                grid[x, y] = CellStatus.Unknown;
            }
        }

        Debug.Log($"Grid Map Initialized: Size {gridSizeX}x{gridSizeY}, Cell Size: {cellSize}");
    }

    void Update()
    {
        // Update peta berdasarkan posisi robot secara berkala
        if (Time.time - lastPositionUpdateTime > positionUpdateRate)
        {
            UpdateMapBasedOnRobotPosition();
            lastPositionUpdateTime = Time.time;
        }
    }

    // --- Fungsi Konversi Koordinat ---

    /// <summary>
    /// Konversi posisi dunia (Vector3) ke koordinat grid (Vector2Int).
    /// </summary>
    public Vector2Int WorldToGrid(Vector3 worldPosition)
    {
        Vector3 localPos = worldPosition - worldBottomLeft;
        int x = Mathf.Clamp(Mathf.FloorToInt(localPos.x / cellSize), 0, gridSizeX - 1);
        int y = Mathf.Clamp(Mathf.FloorToInt(localPos.z / cellSize), 0, gridSizeY - 1); // Pakai Z untuk kedalaman/panjang di Unity
        return new Vector2Int(x, y);
    }

    /// <summary>
    /// Konversi koordinat grid (int x, int y) ke posisi dunia (Vector3, pusat sel).
    /// </summary>
    public Vector3 GridToWorld(int x, int y)
    {
        float worldX = worldBottomLeft.x + (x + 0.5f) * cellSize;
        float worldZ = worldBottomLeft.z + (y + 0.5f) * cellSize; // Pakai Z
        // Y (ketinggian) bisa diambil dari worldBottomLeft atau ground level
        return new Vector3(worldX, transform.position.y, worldZ);
    }

    public Vector3 GridToWorld(Vector2Int gridPos)
    {
        return GridToWorld(gridPos.x, gridPos.y);
    }

     /// <summary>
    /// Cek apakah koordinat grid berada dalam batas peta.
    /// </summary>
    public bool IsWithinGrid(int x, int y)
    {
        return x >= 0 && x < gridSizeX && y >= 0 && y < gridSizeY;
    }

    public bool IsWithinGrid(Vector2Int gridPos)
    {
        return IsWithinGrid(gridPos.x, gridPos.y);
    }


    // --- Fungsi Update Peta ---

    /// <summary>
    /// Update peta berdasarkan posisi robot saat ini. Menandai sel sebagai Visited/Free.
    /// </summary>
    private void UpdateMapBasedOnRobotPosition()
    {
        Vector2Int robotGridPos = WorldToGrid(robotTransform.position);
        if (IsWithinGrid(robotGridPos))
        {
            // Tandai sel saat ini sebagai Visited (dan juga Free)
             if (grid[robotGridPos.x, robotGridPos.y] != CellStatus.Obstacle) // Jangan timpa obstacle
             {
                  grid[robotGridPos.x, robotGridPos.y] = CellStatus.Visited;
             }

            // Tambahkan ke jejak jika belum ada atau berbeda dari terakhir
            if (robotPath.Count == 0 || robotPath[robotPath.Count - 1] != robotGridPos)
            {
                robotPath.Add(robotGridPos);
            }
        }
    }

    /// <summary>
    /// Mengubah status sel pada koordinat grid tertentu.
    /// </summary>
    public void UpdateCellStatus(int x, int y, CellStatus status)
    {
        if (IsWithinGrid(x, y))
        {
            // Logika tambahan: Jangan ubah Obstacle jadi Free/Visited,
            // tapi Free/Visited bisa jadi Obstacle. Unknown bisa jadi apa saja.
            if (grid[x, y] == CellStatus.Obstacle && (status == CellStatus.Free || status == CellStatus.Visited))
            {
                 return; // Jangan timpa Obstacle dengan Free/Visited
            }
             if (grid[x, y] == CellStatus.Visited && status == CellStatus.Free)
            {
                return; // Jangan ubah Visited kembali jadi Free (Visited lebih spesifik)
            }

            grid[x, y] = status;
        }
    }
     public void UpdateCellStatus(Vector2Int gridPos, CellStatus status)
    {
        UpdateCellStatus(gridPos.x, gridPos.y, status);
    }

     /// <summary>
    /// Mengubah status sel pada posisi dunia tertentu.
    /// </summary>
    public void UpdateCellStatus(Vector3 worldPosition, CellStatus status)
    {
        Vector2Int gridPos = WorldToGrid(worldPosition);
        UpdateCellStatus(gridPos, status);
    }

    /// <summary>
    /// Menandai semua sel yang dilalui garis lurus antara dua titik dunia.
    /// PENTING: Ini menggunakan metode stepping sederhana, bukan Bresenham.
    /// </summary>
    public void MarkLineAs(Vector3 worldStart, Vector3 worldEnd, CellStatus status)
    {
        Vector2Int startGrid = WorldToGrid(worldStart);
        Vector2Int endGrid = WorldToGrid(worldEnd);

        Vector3 direction = (worldEnd - worldStart).normalized;
        float distance = Vector3.Distance(worldStart, worldEnd);
        float stepSize = cellSize * 0.5f; // Ukuran langkah lebih kecil dari sel

        Vector3 currentPos = worldStart;
        Vector2Int lastGridPos = startGrid;

        // Tandai sel awal
        UpdateCellStatus(startGrid, status);

        int maxSteps = Mathf.CeilToInt(distance / stepSize) + 2; // Batas langkah
        int stepCount = 0;

        while (Vector3.Distance(currentPos, worldStart) < distance && stepCount < maxSteps)
        {
            currentPos += direction * stepSize;
            Vector2Int currentGridPos = WorldToGrid(currentPos);

            // Hanya update jika pindah ke sel grid baru
            if (currentGridPos != lastGridPos && IsWithinGrid(currentGridPos))
            {
                 // Periksa sel antara lastGridPos dan currentGridPos (untuk diagonal) - Opsional tapi lebih baik
                 MarkIntermediateCells(lastGridPos, currentGridPos, status);

                UpdateCellStatus(currentGridPos, status);
                lastGridPos = currentGridPos;

                 // Jika sudah mencapai sel tujuan, berhenti
                 if(currentGridPos == endGrid) break;
            }
            stepCount++;
        }

        // Tandai sel akhir (jika belum)
         UpdateCellStatus(endGrid, status);
    }

    // Helper untuk MarkLineAs - mengisi sel di antara langkah diagonal (sederhana)
    private void MarkIntermediateCells(Vector2Int from, Vector2Int to, CellStatus status)
    {
        int dx = to.x - from.x;
        int dy = to.y - from.y;

        // Jika bergerak diagonal (dx != 0 dan dy != 0)
        if (Mathf.Abs(dx) > 0 && Mathf.Abs(dy) > 0)
        {
            // Cek dua sel tetangga yang mungkin terlewati
            UpdateCellStatus(from.x + dx, from.y, status);
            UpdateCellStatus(from.x, from.y + dy, status);
        }
    }


    /// <summary>
    /// Menandai sel di posisi dunia sebagai Obstacle.
    /// </summary>
    public void MarkCellAsObstacle(Vector3 worldPosition)
    {
        UpdateCellStatus(worldPosition, CellStatus.Obstacle);
    }

    /// <summary>
    /// Menandai sel di posisi dunia sebagai Free (jika belum diketahui atau obstacle).
    /// </summary>
     public void MarkCellAsFree(Vector3 worldPosition)
    {
        Vector2Int gridPos = WorldToGrid(worldPosition);
        if (IsWithinGrid(gridPos) && grid[gridPos.x, gridPos.y] == CellStatus.Unknown)
        {
            UpdateCellStatus(gridPos, CellStatus.Free);
        }
    }

    // --- Fungsi Akses Peta ---

     /// <summary>
    /// Mendapatkan status sel pada koordinat grid.
    /// </summary>
    public CellStatus GetCellStatus(int x, int y)
    {
        if (IsWithinGrid(x, y))
        {
            return grid[x, y];
        }
        return CellStatus.Unknown; // Atau status lain untuk di luar batas
    }
     public CellStatus GetCellStatus(Vector2Int gridPos)
    {
        return GetCellStatus(gridPos.x, gridPos.y);
    }
      public CellStatus GetCellStatus(Vector3 worldPosition)
    {
        return GetCellStatus(WorldToGrid(worldPosition));
    }

    /// <summary>
    /// Mendapatkan daftar tetangga yang valid dan walkable dari sebuah sel grid.
    /// </summary>
    /// <param name="gridPos">Koordinat sel asal.</param>
    /// <param name="includeDiagonals">Apakah menyertakan tetangga diagonal?</param>
    /// <returns>List Vector2Int berisi koordinat tetangga yang walkable.</returns>
    public List<Vector2Int> GetNeighbors(Vector2Int gridPos, bool includeDiagonals = true)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();
        int[] dx = { 0, 0, 1, -1, 1, 1, -1, -1 }; // Offset x (orthogonal + diagonal)
        int[] dy = { 1, -1, 0, 0, 1, -1, 1, -1 }; // Offset y (orthogonal + diagonal)
        int limit = includeDiagonals ? 8 : 4; // Batas loop (4 untuk orthogonal, 8 untuk diagonal)

        for (int i = 0; i < limit; i++)
        {
            Vector2Int neighborPos = new Vector2Int(gridPos.x + dx[i], gridPos.y + dy[i]);

            // Cek apakah tetangga di dalam batas grid
            if (!IsWithinGrid(neighborPos)) continue;

            // Cek apakah tetangga walkable (bukan Obstacle)
            CellStatus status = GetCellStatus(neighborPos);
            if (status != CellStatus.Obstacle) // Bisa dilewati jika Free, Visited, atau bahkan Unknown (tergantung strategi A*)
            {
                // --- Penting untuk A* diagonal ---
                // Jika bergerak diagonal, pastikan tidak memotong sudut obstacle
                if (i >= 4) // Jika ini gerakan diagonal
                {
                     // Cek 2 sel orthogonal pembentuk sudut diagonal
                     CellStatus cornerCheck1 = GetCellStatus(gridPos.x + dx[i], gridPos.y);
                     CellStatus cornerCheck2 = GetCellStatus(gridPos.x, gridPos.y + dy[i]);
                     if (cornerCheck1 == CellStatus.Obstacle || cornerCheck2 == CellStatus.Obstacle)
                     {
                          continue; // Jangan izinkan potong sudut obstacle
                     }
                }
                 // --- Akhir Pengecekan Diagonal ---

                neighbors.Add(neighborPos);
            }
        }
        return neighbors;
    }


    // --- Pelacakan Progres Eksplorasi ---
    /// <summary>
    /// Menghitung persentase area yang sudah diketahui (bukan Unknown).
    /// </summary>
    public float GetExplorationPercentage()
    {
        if (gridSizeX == 0 || gridSizeY == 0) return 0f;

        int knownCells = 0;
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                if (grid[x, y] != CellStatus.Unknown)
                {
                    knownCells++;
                }
            }
        }
        return (float)knownCells / (gridSizeX * gridSizeY) * 100f;
    }


    // --- Visualisasi Gizmos ---
    void OnDrawGizmos()
    {
        if (!showGizmos || grid == null) return;

        // Gambar batas luar area peta
        Vector3 center = transform.position; // Asumsi di tengah
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(center, new Vector3(worldSize.x, 0.1f, worldSize.y)); // Pakai Z untuk panjang

        // Gambar setiap sel grid
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                switch (grid[x, y])
                {
                    case CellStatus.Free: Gizmos.color = freeColor; break;
                    case CellStatus.Obstacle: Gizmos.color = obstacleColor; break;
                    case CellStatus.Visited: Gizmos.color = visitedColor; break;
                    case CellStatus.Unknown: Gizmos.color = unknownColor; break;
                }
                // Buat warna sedikit transparan
                Color cellColor = Gizmos.color;
                cellColor.a = 0.4f;
                Gizmos.color = cellColor;

                Vector3 cellCenter = GridToWorld(x, y);
                Gizmos.DrawCube(cellCenter, new Vector3(cellSize, 0.05f, cellSize));
            }
        }

         // Gambar jejak robot
        if (robotPath.Count > 1)
        {
            Gizmos.color = robotPathColor;
            for (int i = 0; i < robotPath.Count - 1; i++)
            {
                Gizmos.DrawLine(GridToWorld(robotPath[i]), GridToWorld(robotPath[i + 1]));
            }
        }
    }
}