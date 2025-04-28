using UnityEngine;

[RequireComponent(typeof(DDMRController))]
public class OdometryEstimator : MonoBehaviour
{
    [Header("Status Odometri (Read Only)")]
    [Tooltip("Estimasi posisi robot berdasarkan odometri.")]
    public Vector3 estimatedPosition;
    [Tooltip("Estimasi rotasi robot (derajat) di sekitar sumbu Y.")]
    public float estimatedRotationYDeg;

    [Header("Debugging")]
    [Tooltip("Tampilkan Gizmo di Scene view untuk posisi estimasi.")]
    public bool showGizmo = true;
    public Color gizmoColor = Color.yellow;
    public float gizmoSize = 0.1f;

    // Referensi
    private DDMRController ddmrController;

    // State Odometri Internal
    private float odomX;
    private float odomZ;
    private float odomThetaRad; // Simpan dalam radian untuk perhitungan

    void Start()
    {
        ddmrController = GetComponent<DDMRController>();
        if (ddmrController == null) {
            Debug.LogError("OdometryEstimator: DDMRController tidak ditemukan!", this);
            enabled = false;
            return;
        }

        // Inisialisasi pose odometri dengan pose awal robot
        odomX = transform.position.x;
        odomZ = transform.position.z;
        odomThetaRad = transform.eulerAngles.y * Mathf.Deg2Rad; // Konversi awal ke radian

        // Update nilai publik awal
        UpdatePublicStatus();

        Debug.Log($"Odometry Initialized: Pos({odomX:F2}, {odomZ:F2}), ThetaRad({odomThetaRad:F3})");
    }

    void FixedUpdate()
    {
        // Dapatkan parameter dari DDMRController
        float r = ddmrController.wheelRadius;
        float L = ddmrController.axleLength;

        // Dapatkan kecepatan roda dari DDMRController (gunakan getter yang dibuat)
        float omegaL = ddmrController.CurrentLeftWheelSpeedRad;
        float omegaR = ddmrController.CurrentRightWheelSpeedRad;

        // Dapatkan delta time
        float dt = Time.fixedDeltaTime;

        // --- Hitung Perubahan Pose (Odometry Calculation) ---

        // Jarak tempuh roda
        float distL = omegaL * r * dt;
        float distR = omegaR * r * dt;

        // Jarak tempuh pusat robot
        float deltaDist = (distR + distL) / 2.0f;

        // Perubahan orientasi (radian)
        float deltaTheta = (distR - distL) / L;

        // --- Update Pose Odometri ---

        // Orientasi rata-rata selama interval dt
        float avgTheta = odomThetaRad + (deltaTheta / 2.0f);

        // Update posisi X dan Z
        odomX += deltaDist * Mathf.Sin(avgTheta); // Gerakan X = f(sin(theta))
        odomZ += deltaDist * Mathf.Cos(avgTheta); // Gerakan Z = f(cos(theta))

        // Update orientasi
        odomThetaRad += deltaTheta;

        // Normalisasi sudut theta agar tetap dalam rentang (-PI, PI] (opsional tapi bagus)
        odomThetaRad = Mathf.Atan2(Mathf.Sin(odomThetaRad), Mathf.Cos(odomThetaRad));

        // --- Update Status Publik ---
        UpdatePublicStatus();
    }

    /// <summary>
    /// Mengupdate variabel publik untuk ditampilkan di Inspector.
    /// </summary>
    void UpdatePublicStatus()
    {
        // Y diestimasi sama dengan Y transform asli (asumsi gerakan datar)
        estimatedPosition = new Vector3(odomX, transform.position.y, odomZ);
        estimatedRotationYDeg = odomThetaRad * Mathf.Rad2Deg; // Konversi ke derajat untuk Inspector
    }


    // --- Visualisasi Gizmo ---
    void OnDrawGizmos()
    {
        if (showGizmo && Application.isPlaying) // Hanya gambar saat playing
        {
            Gizmos.color = gizmoColor;
            // Gambar bola kecil di posisi estimasi odometri
            Gizmos.DrawSphere(estimatedPosition, gizmoSize);

            // Gambar garis arah orientasi estimasi
            Quaternion estimatedRotation = Quaternion.Euler(0, estimatedRotationYDeg, 0);
            Vector3 forwardDirection = estimatedRotation * Vector3.forward;
            Gizmos.DrawLine(estimatedPosition, estimatedPosition + forwardDirection * gizmoSize * 5f);
        }
    }

    // --- Fungsi Akses Publik (jika script lain perlu pose odometri) ---
    public Vector3 GetEstimatedPosition()
    {
        // Pastikan Y selalu update dengan transform asli jika perlu
        return new Vector3(odomX, transform.position.y, odomZ);
        // atau return estimatedPosition;
    }

    public float GetEstimatedRotationYRad()
    {
        return odomThetaRad;
    }

     public float GetEstimatedRotationYDeg()
    {
        return estimatedRotationYDeg;
    }

     public Quaternion GetEstimatedRotation()
     {
         return Quaternion.Euler(0, estimatedRotationYDeg, 0);
     }

} // Akhir Class