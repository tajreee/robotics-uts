using UnityEngine;
using System.Collections.Generic;
using System.Linq;

[RequireComponent(typeof(PathFollower))]
[RequireComponent(typeof(DDMRController))]
[RequireComponent(typeof(ObstacleAvoider))]
public class ExplorationController : MonoBehaviour
{

    // Enum untuk state robot
    public enum ExplorationState { Idle, FindingFrontier, MovingToFrontier, MovingToTarget, ExplorationComplete, ReturningToBase } // Tambahkan state sesuai kebutuhan

    [Header("Status")]
    [SerializeField] private ExplorationState currentState = ExplorationState.Idle;

    [Header("Pengaturan Eksplorasi")]
    [Tooltip("Jumlah maksimum frontier yang dipertimbangkan (untuk performa).")]
    public int maxFrontiersToConsider = 50;
    [Tooltip("Jarak minimum dari robot ke frontier agar dianggap valid (mencegah target terlalu dekat).")]
    public float minFrontierDistance = 1.0f;


    // Referensi Komponen
    private GridMappingSystem gridMappingSystem;
    private PathFollower pathFollower;
    private DDMRController ddmrController;
    private ObstacleAvoider obstacleAvoider; // Mungkin tidak perlu direct call jika PathFollower/Avoider handle interaction

    private Vector2Int currentTargetGridPos; // Posisi grid tujuan saat ini (frontier atau target lain)
    private int explorationFailCounter = 0; // Menghitung kegagalan mencari frontier/path
    private const int MAX_EXPLORATION_FAILS = 5; // Batas kegagalan sebelum menyerah

    void Start()
    {
        // Dapatkan referensi
        pathFollower = GetComponent<PathFollower>();
        ddmrController = GetComponent<DDMRController>();
        obstacleAvoider = GetComponent<ObstacleAvoider>(); // Ambil jika perlu cek IsAvoiding
        gridMappingSystem = FindObjectOfType<GridMappingSystem>(); // Cari map

        if (gridMappingSystem == null) { Debug.LogError("ExplorationController: GridMappingSystem tidak ditemukan!", this); enabled = false; return; }

        // Setup event handler dari PathFollower
        pathFollower.OnPathCompleted += HandlePathCompleted;
        pathFollower.OnPathInterrupted += HandlePathInterrupted;

        // Mulai eksplorasi
        ChangeState(ExplorationState.FindingFrontier); // Langsung mulai cari frontier
    }

    void Update()
    {
        // State Machine Sederhana
        switch (currentState)
        {
            case ExplorationState.Idle:
                // Mungkin menunggu perintah atau timer sebelum mulai lagi
                ChangeState(ExplorationState.FindingFrontier); // Coba lagi cari frontier
                break;

            case ExplorationState.FindingFrontier:
                // Logika ini dieksekusi sekali saat masuk state
                // (Implementasi sebenarnya di ChangeState atau fungsi terpisah)
                break; // Tunggu hasil async atau panggil di frame berikutnya

            case ExplorationState.MovingToFrontier:
            case ExplorationState.MovingToTarget:
            case ExplorationState.ReturningToBase:
                // Pergerakan ditangani oleh PathFollower
                // Di sini bisa memonitor progres atau kondisi lain
                 // Contoh: Cek jika peta di jalur berubah jadi obstacle
                CheckPathObstruction();
                break;

            case ExplorationState.ExplorationComplete:
                // Semua area dijelajahi atau tidak ada frontier lagi
                ddmrController.Stop(); // Hentikan robot
                Debug.Log("Eksplorasi Selesai!");
                // Bisa tambahkan logika kembali ke base
                break;
        }

         // Debugging Tampilan State
         // Debug.Log($"Current State: {currentState}");
    }

    // --- Manajemen State ---
    void ChangeState(ExplorationState newState)
    {
        if (currentState == newState) return;

        Debug.Log($"Changing State from {currentState} to {newState}");
        currentState = newState;

        // Aksi saat memasuki state baru
        switch (newState)
        {
            case ExplorationState.FindingFrontier:
                pathFollower.StopFollowing(); // Hentikan gerakan sebelumnya
                // Cari frontier dan path secara asynchronous atau di frame berikutnya
                StartCoroutine(FindAndNavigateToFrontier());
                break;

            case ExplorationState.MovingToFrontier:
            case ExplorationState.MovingToTarget:
            case ExplorationState.ReturningToBase:
                // Path seharusnya sudah di-set oleh proses sebelumnya
                 if (!pathFollower.IsFollowingPath) {
                      Debug.LogWarning($"Masuk state {newState} tapi PathFollower tidak aktif!");
                      // Mungkin kembali ke FindingFrontier jika path tidak valid
                      ChangeState(ExplorationState.FindingFrontier);
                 }
                break;

             case ExplorationState.ExplorationComplete:
                 pathFollower.StopFollowing();
                 ddmrController.Stop();
                 break;

             case ExplorationState.Idle:
                  pathFollower.StopFollowing();
                  ddmrController.Stop();
                  break;

        }
    }

    // --- Logika Eksplorasi Frontier ---

    System.Collections.IEnumerator FindAndNavigateToFrontier()
    {
        List<Vector2Int> frontiers = DetectFrontiers();
        if (frontiers == null || frontiers.Count == 0)
        {
            Debug.Log("Tidak ada frontier ditemukan.");
            explorationFailCounter++;
             if(explorationFailCounter >= MAX_EXPLORATION_FAILS) {
                 ChangeState(ExplorationState.ExplorationComplete);
             } else {
                 // Tunggu sebentar lalu coba lagi
                 yield return new WaitForSeconds(2.0f);
                 ChangeState(ExplorationState.FindingFrontier); // Coba lagi
             }
            yield break; // Hentikan coroutine
        }

        Debug.Log($"Ditemukan {frontiers.Count} sel frontier.");

        Vector2Int? chosenFrontier = SelectClosestReachableFrontier(frontiers);

        if (!chosenFrontier.HasValue)
        {
            Debug.LogWarning("Tidak ada frontier yang bisa dijangkau.");
             explorationFailCounter++;
             if(explorationFailCounter >= MAX_EXPLORATION_FAILS) {
                 ChangeState(ExplorationState.ExplorationComplete);
             } else {
                 yield return new WaitForSeconds(2.0f);
                 ChangeState(ExplorationState.FindingFrontier); // Coba lagi
             }
            yield break;
        }

        currentTargetGridPos = chosenFrontier.Value;
        Vector2Int startPos = gridMappingSystem.WorldToGrid(transform.position);

        // Cari Path menggunakan A*
        List<Vector2Int> path = AStarPathfinder.FindPath(startPos, currentTargetGridPos, gridMappingSystem);

        if (path != null && path.Count > 1)
        {
            explorationFailCounter = 0; // Reset counter jika berhasil dapat path
            pathFollower.SetPath(path);
            ChangeState(ExplorationState.MovingToFrontier);
        }
        else
        {
            Debug.LogWarning($"Gagal membuat path ke frontier {currentTargetGridPos}.");
            explorationFailCounter++;
              if(explorationFailCounter >= MAX_EXPLORATION_FAILS) {
                 ChangeState(ExplorationState.ExplorationComplete);
             } else {
                 // Tandai frontier ini sebagai tidak terjangkau? (opsional)
                 yield return new WaitForSeconds(1.0f);
                 ChangeState(ExplorationState.FindingFrontier); // Coba cari frontier lain
             }
        }
         yield return null; // Coroutine selesai untuk frame ini
    }


    List<Vector2Int> DetectFrontiers()
    {
        List<Vector2Int> frontiers = new List<Vector2Int>();
        HashSet<Vector2Int> visitedForFrontier = new HashSet<Vector2Int>(); // Optimasi agar tidak cek sel yg sama berulang

        // Pindai seluruh grid (optimasi bisa dilakukan nanti)
        for (int x = 0; x < gridMappingSystem.GridSizeX; x++)
        {
            for (int y = 0; y < gridMappingSystem.GridSizeY; y++)
            {
                Vector2Int currentPos = new Vector2Int(x, y);
                GridMappingSystem.CellStatus status = gridMappingSystem.GetCellStatus(currentPos);

                // Frontier adalah sel Free/Visited yang bertetangga dengan Unknown
                if (status == GridMappingSystem.CellStatus.Free || status == GridMappingSystem.CellStatus.Visited)
                {
                    // Cek tetangga
                    bool hasUnknownNeighbor = false;
                    foreach (Vector2Int neighbor in gridMappingSystem.GetNeighbors(currentPos, false)) // Cek tetangga orthogonal saja cukup
                    {
                        if (gridMappingSystem.GetCellStatus(neighbor) == GridMappingSystem.CellStatus.Unknown)
                        {
                            hasUnknownNeighbor = true;
                            break;
                        }
                    }

                    if (hasUnknownNeighbor)
                    {
                        frontiers.Add(currentPos);
                    }
                }
            }
        }
        return frontiers;
    }

    Vector2Int? SelectClosestReachableFrontier(List<Vector2Int> frontiers)
    {
        Vector2Int robotGridPos = gridMappingSystem.WorldToGrid(transform.position);
        Vector2Int? bestTarget = null;
        float minCost = float.MaxValue; // Gunakan cost A* jika memungkinkan, atau jarak

        // Urutkan frontier berdasarkan jarak lurus (estimasi awal)
        frontiers.Sort((a, b) =>
            Vector2.Distance(robotGridPos, a).CompareTo(Vector2.Distance(robotGridPos, b))
        );

        int consideredCount = 0;
        foreach (Vector2Int frontier in frontiers)
        {
             // Jangan pilih frontier terlalu dekat
             if (Vector2.Distance(robotGridPos, frontier) * gridMappingSystem.CellSize < minFrontierDistance) continue;

             // --- Cek Keterjangkauan (Idealnya pakai A* singkat, tapi mahal) ---
             // Opsi 1: Anggap terjangkau jika statusnya Free/Visited (simplifikasi)
             // Opsi 2: Cek garis lurus pakai raycast grid (lebih baik)
             // Opsi 3: Jalankan A* penuh (paling akurat tapi lambat jika banyak frontier)

             // Kita pakai Opsi 1 dulu untuk kesederhanaan:
              bestTarget = frontier; // Pilih yang terdekat secara lurus
              break; // Langsung ambil yang terdekat, A* nanti akan konfirmasi

            /* // Opsi 3 (lebih lambat):
            List<Vector2Int> tempPath = AStarPathfinder.FindPath(robotGridPos, frontier, gridMappingSystem);
            if (tempPath != null && tempPath.Count > 1) {
                // Hitung cost path? Atau ambil yg pertama ditemukan?
                bestTarget = frontier;
                break; // Ambil frontier terdekat yg ada path nya
            }
            */

             consideredCount++;
             if(consideredCount >= maxFrontiersToConsider) break; // Batasi jumlah pengecekan
        }

        return bestTarget;
    }


    // --- Penanganan Event PathFollower ---
    void HandlePathCompleted()
    {
        Debug.Log("ExplorationController: Path completed.");
        // Jika sedang menuju frontier, cari frontier berikutnya
        if (currentState == ExplorationState.MovingToFrontier)
        {
            ChangeState(ExplorationState.FindingFrontier);
        }
        // Jika sedang menuju target lain, mungkin Idle atau state berikutnya
        else if (currentState == ExplorationState.MovingToTarget || currentState == ExplorationState.ReturningToBase)
        {
             // Misi ke target selesai, kembali Idle atau state lain
             ChangeState(ExplorationState.Idle); // Contoh: Kembali idle
        }
    }

    void HandlePathInterrupted()
    {
        Debug.LogWarning("ExplorationController: Path interrupted (likely blocked).");
        // Path follower gagal, mungkin perlu replanning
         ChangeState(ExplorationState.FindingFrontier); // Coba cari rute baru
    }

    // --- Pengecekan Path Obstruction ---
    void CheckPathObstruction() {
         if(pathFollower.IsFollowingPath && pathFollower.currentPath != null) {
              // Periksa beberapa waypoint ke depan di path
              int checkLimit = Mathf.Min(pathFollower.currentWaypointIndex + 3, pathFollower.currentPath.Count);
              for(int i = pathFollower.currentWaypointIndex; i < checkLimit; i++) {
                   if(gridMappingSystem.GetCellStatus(pathFollower.currentPath[i]) == GridMappingSystem.CellStatus.Obstacle) {
                        Debug.LogWarning($"Obstacle detected on planned path at {pathFollower.currentPath[i]}! Replanning...");
                        HandlePathInterrupted(); // Trigger replan
                        break;
                   }
              }
         }
    }

    // --- Fungsi untuk Re-routing Manual ---

    /// <summary>
    /// Memerintahkan robot untuk pergi ke posisi dunia target.
    /// </summary>
    public void GoToTarget(Vector3 worldTargetPosition, ExplorationState targetState = ExplorationState.MovingToTarget)
    {
        Vector2Int targetGridPos = gridMappingSystem.WorldToGrid(worldTargetPosition);
        Vector2Int startGridPos = gridMappingSystem.WorldToGrid(transform.position);

        Debug.Log($"Attempting to route from {startGridPos} to target {targetGridPos}");

        List<Vector2Int> path = AStarPathfinder.FindPath(startGridPos, targetGridPos, gridMappingSystem);

        if (path != null && path.Count > 1)
        {
            pathFollower.SetPath(path);
            ChangeState(targetState); // Masuk ke state menuju target
        }
        else
        {
            Debug.LogError($"Failed to find path to target position {worldTargetPosition} (Grid: {targetGridPos})");
            // Mungkin kembali ke state sebelumnya atau Idle
            ChangeState(ExplorationState.Idle);
        }
    }

    // TODO: Tambahkan fungsi OnBombFound(Vector3 bombPos) yang akan memanggil GoToTarget()
    // ke lokasi bom berikutnya atau kembali ke base.

} // Akhir Class