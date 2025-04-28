using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(DDMRController))]
[RequireComponent(typeof(ObstacleAvoider))]
public class PathFollower : MonoBehaviour
{
    [Header("Pengaturan Path Following")]
    [Tooltip("Jarak (meter) ke waypoint agar dianggap tercapai.")]
    public float waypointReachedThreshold = 0.3f;
    [Tooltip("Kecepatan linear maksimum saat mengikuti jalur.")]
    public float maxPathSpeed = 1.0f;
    [Tooltip("Seberapa cepat robot berbelok untuk menghadap waypoint (0-1).")]
    [Range(0.1f, 5.0f)]
    public float turnSpeedFactor = 2.0f; // Pengali untuk kecepatan belok DDMR

    [Header("Status (Read Only)")]
    [SerializeField] public List<Vector2Int> currentPath = null;
    [SerializeField] public int currentWaypointIndex = -1;
    [SerializeField] public bool isFollowingPath = false;
    [SerializeField] public Vector3 targetWorldPosition;

    // Referensi Komponen
    private DDMRController ddmrController;
    private ObstacleAvoider obstacleAvoider;
    private GridMappingSystem gridMappingSystem; // Diperlukan untuk konversi Grid ke World

    public bool IsFollowingPath => isFollowingPath;
    public System.Action OnPathCompleted; // Event saat path selesai
    public System.Action OnPathInterrupted; // Event jika path terganggu (misal kena obstacle tak terduga)

    void Awake()
    {
        ddmrController = GetComponent<DDMRController>();
        obstacleAvoider = GetComponent<ObstacleAvoider>();
        // Cari GridMappingSystem di scene
        gridMappingSystem = FindObjectOfType<GridMappingSystem>();
        if (gridMappingSystem == null)
        {
            Debug.LogError("PathFollower: GridMappingSystem tidak ditemukan!", this);
            enabled = false;
        }
    }

    /// <summary>
    /// Memberikan jalur baru untuk diikuti oleh robot.
    /// </summary>
    public void SetPath(List<Vector2Int> newPath)
    {
        if (newPath == null || newPath.Count <= 1) // Perlu minimal 2 titik (start & target)
        {
            Debug.LogWarning("PathFollower: Jalur tidak valid atau terlalu pendek.");
            StopFollowing();
            return;
        }

        currentPath = newPath;
        currentWaypointIndex = 1; // Mulai dari titik KEDUA (indeks 1), karena indeks 0 adalah posisi start
        targetWorldPosition = gridMappingSystem.GridToWorld(currentPath[currentWaypointIndex]);
        isFollowingPath = true;
        Debug.Log($"PathFollower: Memulai jalur baru dengan {currentPath.Count} waypoints.");
    }

    /// <summary>
    /// Menghentikan proses mengikuti jalur.
    /// </summary>
    public void StopFollowing()
    {
        isFollowingPath = false;
        currentPath = null;
        currentWaypointIndex = -1;
        // Perintahkan robot berhenti jika tidak ada kontrol lain
        // ddmrController.Stop(); // Mungkin dikontrol oleh ExplorationController
        Debug.Log("PathFollower: Berhenti mengikuti jalur.");
    }

    void Update()
    {
        if (!isFollowingPath || currentPath == null || currentWaypointIndex < 0)
        {
            return; // Tidak sedang mengikuti jalur
        }

        // --- Cek Jarak ke Waypoint ---
        Vector3 currentPosition = transform.position;
        // Proyeksikan posisi target dan robot ke bidang XZ untuk perbandingan jarak 2D
        Vector3 currentPosXZ = new Vector3(currentPosition.x, 0, currentPosition.z);
        Vector3 targetPosXZ = new Vector3(targetWorldPosition.x, 0, targetWorldPosition.z);
        float distanceToTarget = Vector3.Distance(currentPosXZ, targetPosXZ);

        // --- Mencapai Waypoint ---
        if (distanceToTarget < waypointReachedThreshold)
        {
            currentWaypointIndex++;
            // Cek apakah sudah sampai waypoint terakhir
            if (currentWaypointIndex >= currentPath.Count)
            {
                Debug.Log("PathFollower: Telah mencapai akhir jalur.");
                isFollowingPath = false;
                OnPathCompleted?.Invoke(); // Panggil event selesai
                // Jangan langsung berhenti, biarkan ExplorationController memutuskan
                return;
            }
            else
            {
                // Update target ke waypoint berikutnya
                targetWorldPosition = gridMappingSystem.GridToWorld(currentPath[currentWaypointIndex]);
                // Debug.Log($"PathFollower: Mencapai waypoint, lanjut ke indeks {currentWaypointIndex}");
            }
        }

        // --- Perhitungan Gerakan (jika tidak sedang menghindar aktif) ---
        // PENTING: Interaksi dengan ObstacleAvoider
        // Jika ObstacleAvoider aktif (isAvoiding=true), ia akan 'mengambil alih' kontrol DDMRController
        // Kita tetap hitung target dari PathFollower, tapi ObstacleAvoider akan memprioritaskan manuvernya.
        // ObstacleAvoider versi Repulsion akan secara natural mengkombinasikan dorongan & arah target.

        // Hitung arah ke target waypoint
        Vector3 directionToTarget = (targetWorldPosition - currentPosition);
        directionToTarget.y = 0; // Fokus pada gerakan horizontal
        directionToTarget.Normalize();

        // Hitung sudut yang perlu dibelokkan robot
        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        // Tentukan kecepatan linear target (bisa dibuat melambat saat mendekati target)
        // float speedScale = Mathf.Clamp01(distanceToTarget / (waypointReachedThreshold * 2)); // Contoh melambat
        float targetLinearSpeed = maxPathSpeed; // * speedScale;

        // Tentukan kecepatan angular target (lebih cepat jika sudut besar)
        // Konversi sudut ke radian dan kalikan faktor kecepatan belok DDMRController
        float targetAngularSpeedRad = (angleToTarget * Mathf.Deg2Rad) * turnSpeedFactor;

        // Kirim perintah ke DDMRController
        // ObstacleAvoider akan memodifikasi ini jika perlu (terutama angular speed)
         if (!obstacleAvoider.IsAvoiding()) // Hanya kirim jika tidak sedang explicit avoidance state (jika pakai state tsb)
         {
            // Jika menggunakan Avoider berbasis Repulsion, avoider akan selalu aktif & menggabungkan gaya,
            // jadi kita bisa selalu kirim perintah ini. Avoider akan 'menang' jika ada repulsion kuat.
             ddmrController.SetTargetVelocities(targetLinearSpeed, targetAngularSpeedRad);
         }
         // Jika Avoider punya state IsAvoiding() dan sedang aktif, kita biarkan Avoider yang mengontrol
         // else { // Biarkan avoider bekerja }


        // --- Deteksi Path Obstruction (Sederhana) ---
        // Jika ObstacleAvoider aktif saat kita sedang menuju waypoint, mungkin path terblokir
        if (obstacleAvoider.IsAvoiding())
        {
             // Bisa tambahkan logika di sini: jika Avoider aktif terlalu lama saat Path Following,
             // anggap path terblokir dan panggil OnPathInterrupted?.Invoke();
             // Lalu ExplorationController bisa trigger replanning.
        }

        // Visualisasi (opsional)
        if (isFollowingPath) Debug.DrawLine(transform.position, targetWorldPosition, Color.blue);

    } // Akhir Update
}