using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(DDMRController))]
public class ObstacleAvoider : MonoBehaviour
{
    // Enum untuk state kontrol robot: Menambah FollowingLeftWall
    public enum ControlState { StandardAvoidance, FollowingRightWall, FollowingLeftWall, SearchingForWall }

    [Header("Raycasting Setup")]
    [Tooltip("Maximum distance rays will check for obstacles.")]
    public float raycastDistance = 2.5f; // Tingkatkan sedikit dari sebelumnya?
    [Tooltip("Layers that contain obstacles the robot should avoid.")]
    public LayerMask obstacleLayer;
    [Tooltip("Number of rays to cast in a fan shape for general detection.")]
    [Range(5, 31)] // Pastikan ganjil jika ingin ada tengah
    public int numberOfRays = 11;
    [Tooltip("Total angle (degrees) covered by the main ray fan.")]
    public float detectionAngle = 140f;
    [Tooltip("Vertical offset for ray origins from the robot's pivot point.")]
    public float rayOriginHeightOffset = 0.15f;
    [Tooltip("Forward offset for ray origins from the robot's pivot point.")]
    public float rayOriginForwardOffset = 0.1f;
    [Tooltip("Show raycasts in the Scene view.")]
    public bool showDebugRays = true;


    [Header("Forward Bias Behavior")]
    [Tooltip("Angle (degrees) for the critical forward detection cone (+/- this value from forward).")]
    public float forwardDetectionAngle = 25f;
    [Tooltip("Distance threshold (meters). Obstacles closer than this in the forward cone trigger avoidance.")]
    public float forwardObstacleThreshold = 1.2f;


    [Header("Standard Avoidance (Fallback)")]
     [Tooltip("Kecepatan belok (derajat/detik) saat standard avoidance (tabrakan depan).")]
    public float avoidanceTurnSpeedDeg = 100f;
    [Tooltip("Kecepatan maju saat standard avoidance (0 untuk belok di tempat).")]
    public float speedDuringAvoidance = 0.0f;
    [Tooltip("Jarak minimum dari rintangan di semua arah agar dianggap terjebak.")]
    public float trapDistanceThreshold = 0.4f;


    // --- Parameter Wall Following untuk KEDUA SISI ---
    [Header("Wall Following (Both Sides)")]
    [Tooltip("Jarak ideal dari dinding samping (meter).")]
    public float targetWallDistance = 0.8f;
    [Tooltip("Seberapa kuat robot mengoreksi jaraknya ke dinding (Kp). Tuning diperlukan!")]
    public float wallCorrectionGain = 2.5f;
    [Tooltip("Batas kecepatan belok maksimum (rad/s) saat koreksi wall following.")]
    public float maxWallCorrectionTurnRad = 1.57f; // ~90 deg/s (Mathf.PI / 2f)
    [Tooltip("Jarak maksimum ray samping untuk mendeteksi dinding.")]
    public float wallDetectionMaxDistance = 2.0f;
    // --- Sudut spesifik per sisi ---
    [Tooltip("Sudut ray ke samping kanan (derajat) untuk deteksi dinding kanan.")]
    public float wallRightAngle = 90f;
    [Tooltip("Sudut ray ke depan-kanan (derajat) untuk antisipasi tikungan kanan.")]
    public float wallForwardRightAngle = 45f;
    [Tooltip("Sudut ray ke samping kiri (derajat, negatif) untuk deteksi dinding kiri.")]
    public float wallLeftAngle = -90f;
    [Tooltip("Sudut ray ke depan-kiri (derajat, negatif) untuk antisipasi tikungan kiri.")]
    public float wallForwardLeftAngle = -45f;


    [Header("Perilaku Default")]
    [Tooltip("Kecepatan maju default (m/s) saat tidak menghindar/mengikuti dinding.")]
    public float defaultForwardSpeed = 1.0f;


    [Header("Status (Read Only)")]
    [SerializeField] private ControlState currentState = ControlState.SearchingForWall;
    [SerializeField] private bool isAvoidingCritical = false;


    // Referensi & variabel private
    private DDMRController ddmrController;
    private GridMappingSystem gridMappingSystem;
    private float avoidanceTurnSpeedRad;


    // Public getter
    public ControlState CurrentControlState => currentState;
    public bool IsAvoiding() { return isAvoidingCritical; }


    void Awake()
    {
        ddmrController = GetComponent<DDMRController>();
        gridMappingSystem = FindObjectOfType<GridMappingSystem>();
        if (gridMappingSystem == null) { Debug.LogWarning("GridMappingSystem not found for ObstacleAvoider mapping.", this); }
        if (ddmrController == null) { Debug.LogError("DDMRController component not found!", this); enabled = false; return; }

        avoidanceTurnSpeedRad = avoidanceTurnSpeedDeg * Mathf.Deg2Rad;

        // Validasi parameter
        if(wallDetectionMaxDistance <= targetWallDistance) {
             Debug.LogWarning("wallDetectionMaxDistance should be greater than targetWallDistance.");
        }
         if (forwardObstacleThreshold <= 0) { Debug.LogWarning("forwardObstacleThreshold should be positive."); }
         if(wallRightAngle <= 0 || wallForwardRightAngle <=0) {Debug.LogWarning("Right wall following angles should be positive.");}
         if(wallLeftAngle >= 0 || wallForwardLeftAngle >=0) {Debug.LogWarning("Left wall following angles should be negative.");}

    }

    void FixedUpdate()
    {
        PerformObstacleCheckAndControl();
        // Panggil update mapping sekali saja
        UpdateMappingFromScan(CalculateRayOrigin());
    }

    Vector3 CalculateRayOrigin() {
         return transform.position + (Vector3.up * rayOriginHeightOffset) + (transform.forward * rayOriginForwardOffset);
    }


    void PerformObstacleCheckAndControl()
    {
        Vector3 rayOrigin = CalculateRayOrigin();
        ControlState previousState = currentState; // Simpan state sebelumnya untuk deteksi perubahan

        // --- 1. Cek Rintangan Kritis di Depan ---
        isAvoidingCritical = CheckForwardObstacles(rayOrigin);

        // --- 2. Tentukan Logika Kontrol ---
        if (isAvoidingCritical)
        {
            currentState = ControlState.StandardAvoidance;
            PerformStandardAvoidance(rayOrigin);
        }
        else // Depan aman, cek dinding samping
        {
            float sideDistR, frontDistR, sideDistL, frontDistL;
            bool wallOnRight = CheckWallOnRight(rayOrigin, out sideDistR, out frontDistR);
            bool wallOnLeft = CheckWallOnLeft(rayOrigin, out sideDistL, out frontDistL);

            if (wallOnRight && wallOnLeft) // Keduanya terdeteksi (di koridor?)
            {
                // Prioritas: Pertahankan state sebelumnya jika memungkinkan, atau default ke kanan
                if (previousState == ControlState.FollowingLeftWall) {
                     currentState = ControlState.FollowingLeftWall;
                     PerformLeftWallFollowing(sideDistL, frontDistL);
                } else {
                     // Default ke kanan jika baru masuk koridor atau sebelumnya kanan/search
                     currentState = ControlState.FollowingRightWall;
                     PerformRightWallFollowing(sideDistR, frontDistR); // Nama fungsi diubah jadi PerformRightWallFollowing
                }
            }
            else if (wallOnRight) // Hanya kanan terdeteksi
            {
                currentState = ControlState.FollowingRightWall;
                PerformRightWallFollowing(sideDistR, frontDistR); // Nama fungsi diubah jadi PerformRightWallFollowing
            }
            else if (wallOnLeft) // Hanya kiri terdeteksi
            {
                currentState = ControlState.FollowingLeftWall;
                PerformLeftWallFollowing(sideDistL, frontDistL);
            }
            else // Tidak ada dinding terdeteksi
            {
                currentState = ControlState.SearchingForWall;
                PerformSearchOrDefaultForward();
            }
        }

        // Log jika state berubah
        if (currentState != previousState)
        {
            Debug.Log($"ObstacleAvoider State Changed: {previousState} -> {currentState}");
        }

    } // End PerformObstacleCheckAndControl


    // --- Fungsi Helper: Cek Rintangan Depan ---
    bool CheckForwardObstacles(Vector3 rayOrigin)
    {
        // (Implementasi sama seperti sebelumnya, pastikan berfungsi baik)
         int numForwardRays = 1;
         if (numberOfRays > 1 && detectionAngle > 0) {
              numForwardRays = Mathf.Max(1, Mathf.RoundToInt((forwardDetectionAngle / detectionAngle) * (numberOfRays-1) / 2.0f) * 2 + 1);
              numForwardRays = Mathf.Clamp(numForwardRays, 3, numberOfRays);
         }

         float angleStep = (numForwardRays > 1) ? forwardDetectionAngle / (numForwardRays - 1) : 0f;
         float startAngle = -forwardDetectionAngle / 2f;

         for (int i = 0; i < numForwardRays; i++) {
              float currentAngleDeg = (numForwardRays == 1) ? 0f : startAngle + i * angleStep;
              Quaternion rotation = Quaternion.AngleAxis(currentAngleDeg, transform.up);
              Vector3 direction = rotation * transform.forward;
              RaycastHit hit;
              if (Physics.Raycast(rayOrigin, direction, out hit, forwardObstacleThreshold, obstacleLayer)) {
                   if (showDebugRays) Debug.DrawRay(rayOrigin, direction * hit.distance, Color.magenta);
                   return true;
              } else { if (showDebugRays) Debug.DrawRay(rayOrigin, direction * forwardObstacleThreshold, Color.cyan); }
         }
         return false;
    }


    // --- Fungsi Helper: Cek Dinding Kanan ---
    // (Nama parameter sudut diubah agar lebih jelas)
    bool CheckWallOnRight(Vector3 rayOrigin, out float sideDistance, out float frontDistance)
    {
        sideDistance = float.MaxValue;
        frontDistance = float.MaxValue;
        bool sideWallDetected = false;

        // Ray Samping Kanan
        Quaternion sideRot = Quaternion.AngleAxis(wallRightAngle, transform.up);
        Vector3 sideDir = sideRot * transform.forward;
        RaycastHit sideHitInfo;
        if (Physics.Raycast(rayOrigin, sideDir, out sideHitInfo, wallDetectionMaxDistance, obstacleLayer)) {
            sideDistance = sideHitInfo.distance;
            sideWallDetected = true;
            if (showDebugRays) Debug.DrawRay(rayOrigin, sideDir * sideDistance, Color.blue);
        } else { if (showDebugRays) Debug.DrawRay(rayOrigin, sideDir * wallDetectionMaxDistance, Color.gray); }

        // Ray Depan-Kanan
        Quaternion frontRot = Quaternion.AngleAxis(wallForwardRightAngle, transform.up);
        Vector3 frontDir = frontRot * transform.forward;
        RaycastHit frontHitInfo;
        if (Physics.Raycast(rayOrigin, frontDir, out frontHitInfo, wallDetectionMaxDistance, obstacleLayer)) {
            frontDistance = frontHitInfo.distance;
            if (showDebugRays) Debug.DrawRay(rayOrigin, frontDir * frontDistance, Color.cyan);
        } else { if (showDebugRays) Debug.DrawRay(rayOrigin, frontDir * wallDetectionMaxDistance, Color.gray); }

        return sideWallDetected;
    }

     // --- Fungsi Helper: Cek Dinding Kiri ---
    bool CheckWallOnLeft(Vector3 rayOrigin, out float sideDistance, out float frontDistance)
    {
        sideDistance = float.MaxValue;
        frontDistance = float.MaxValue;
        bool sideWallDetected = false;

        // Ray Samping Kiri (gunakan sudut negatif)
        Quaternion sideRot = Quaternion.AngleAxis(wallLeftAngle, transform.up);
        Vector3 sideDir = sideRot * transform.forward;
        RaycastHit sideHitInfo;
        if (Physics.Raycast(rayOrigin, sideDir, out sideHitInfo, wallDetectionMaxDistance, obstacleLayer)) {
            sideDistance = sideHitInfo.distance;
            sideWallDetected = true;
            if (showDebugRays) Debug.DrawRay(rayOrigin, sideDir * sideDistance, Color.magenta); // Warna beda
        } else { if (showDebugRays) Debug.DrawRay(rayOrigin, sideDir * wallDetectionMaxDistance, Color.gray); }

        // Ray Depan-Kiri (gunakan sudut negatif)
        Quaternion frontRot = Quaternion.AngleAxis(wallForwardLeftAngle, transform.up);
        Vector3 frontDir = frontRot * transform.forward;
         RaycastHit frontHitInfo;
        if (Physics.Raycast(rayOrigin, frontDir, out frontHitInfo, wallDetectionMaxDistance, obstacleLayer)) {
            frontDistance = frontHitInfo.distance;
            if (showDebugRays) Debug.DrawRay(rayOrigin, frontDir * frontDistance, Color.yellow); // Warna beda
        } else { if (showDebugRays) Debug.DrawRay(rayOrigin, frontDir * wallDetectionMaxDistance, Color.gray); }

        return sideWallDetected;
    }


    // --- Logika Kontrol: Standard Avoidance (Hindaran Depan) ---
    void PerformStandardAvoidance(Vector3 rayOrigin)
    {
        // (Implementasi sama seperti sebelumnya: cari arah bebas atau pakai repulsion)
        float bestScore = -1f;
        Vector3 chosenEscapeDirection = transform.forward;
        float minHitDistanceOverall = raycastDistance;

        float angleStep = (numberOfRays > 1) ? detectionAngle / (numberOfRays - 1) : 0f;
        float startAngle = -detectionAngle / 2f;
        for (int i = 0; i < numberOfRays; i++) {
            float currentAngleDeg = (numberOfRays == 1) ? 0f : startAngle + i * angleStep;
            Quaternion rotation = Quaternion.AngleAxis(currentAngleDeg, transform.up);
            Vector3 dir = rotation * transform.forward;
            RaycastHit hit;
            bool hitOccurred = Physics.Raycast(rayOrigin, dir, out hit, raycastDistance, obstacleLayer);
            float score = hitOccurred ? hit.distance / raycastDistance : 1.0f;
            if(hitOccurred) minHitDistanceOverall = Mathf.Min(minHitDistanceOverall, hit.distance);
            if (score > bestScore) { bestScore = score; chosenEscapeDirection = dir; }
        }

        if (minHitDistanceOverall < trapDistanceThreshold) {
            Debug.LogWarning("ObstacleAvoider (Standard): Trapped! Forcing sharp turn.");
            ddmrController.SetTargetVelocities(0f, avoidanceTurnSpeedRad);
        } else {
            float turnAngle = Vector3.SignedAngle(transform.forward, chosenEscapeDirection, transform.up);
            float targetAngularVelocity = Mathf.Sign(turnAngle) * avoidanceTurnSpeedRad;
            ddmrController.SetTargetVelocities(speedDuringAvoidance, targetAngularVelocity);
        }
    }


    // --- Logika Kontrol: Wall Following Kanan ---
    // (Nama fungsi diubah agar lebih jelas)
    void PerformRightWallFollowing(float currentSideDist, float currentFrontDist)
    {
        float error = currentSideDist - targetWallDistance;
        // Error positif (terlalu jauh) -> belok kanan (negatif)
        float targetAngularVelocityRad = -error * wallCorrectionGain;
        targetAngularVelocityRad = Mathf.Clamp(targetAngularVelocityRad, -maxWallCorrectionTurnRad, maxWallCorrectionTurnRad);

        float forwardSpeed = defaultForwardSpeed;

        // Antisipasi tikungan dalam kanan (depan-kanan dekat) -> belok kiri lebih kuat
        if (currentFrontDist < targetWallDistance * 1.1f) {
            forwardSpeed *= 0.4f;
            float turnBoost = Mathf.Clamp01(1.0f - (currentFrontDist / (targetWallDistance * 1.1f)));
            targetAngularVelocityRad += maxWallCorrectionTurnRad * 0.6f * turnBoost; // Belok kiri (positif)
            targetAngularVelocityRad = Mathf.Clamp(targetAngularVelocityRad, -maxWallCorrectionTurnRad, maxWallCorrectionTurnRad);
        }
        // Antisipasi tikungan luar kanan / celah (samping kanan & depan-kanan jauh)
        else if (currentSideDist > targetWallDistance * 1.8f && currentFrontDist > wallDetectionMaxDistance * 0.8f) {
            targetAngularVelocityRad = -maxWallCorrectionTurnRad; // Belok kanan maks
            forwardSpeed *= 0.8f;
        }

        ddmrController.SetTargetVelocities(forwardSpeed, targetAngularVelocityRad);
    }

     // --- Logika Kontrol: Wall Following Kiri ---
    void PerformLeftWallFollowing(float currentSideDist, float currentFrontDist)
    {
        float error = currentSideDist - targetWallDistance;
         // Error positif (terlalu jauh) -> belok kiri (positif)
        float targetAngularVelocityRad = +error * wallCorrectionGain; // TANDA + UNTUK KIRI
        targetAngularVelocityRad = Mathf.Clamp(targetAngularVelocityRad, -maxWallCorrectionTurnRad, maxWallCorrectionTurnRad);

        float forwardSpeed = defaultForwardSpeed;

        // Antisipasi tikungan dalam kiri (depan-kiri dekat) -> belok kanan lebih kuat
        if (currentFrontDist < targetWallDistance * 1.1f) {
            forwardSpeed *= 0.4f;
             float turnBoost = Mathf.Clamp01(1.0f - (currentFrontDist / (targetWallDistance * 1.1f)));
            targetAngularVelocityRad -= maxWallCorrectionTurnRad * 0.6f * turnBoost; // Belok kanan (negatif)
             targetAngularVelocityRad = Mathf.Clamp(targetAngularVelocityRad, -maxWallCorrectionTurnRad, maxWallCorrectionTurnRad);
        }
         // Antisipasi tikungan luar kiri / celah (samping kiri & depan-kiri jauh)
        else if (currentSideDist > targetWallDistance * 1.8f && currentFrontDist > wallDetectionMaxDistance * 0.8f) {
            targetAngularVelocityRad = +maxWallCorrectionTurnRad; // Belok kiri maks
            forwardSpeed *= 0.8f;
        }

        ddmrController.SetTargetVelocities(forwardSpeed, targetAngularVelocityRad);
    }


    // --- Logika Kontrol: Default / Mencari Dinding ---
    void PerformSearchOrDefaultForward()
    {
        // Opsi 1: Selalu maju lurus jika tidak ada dinding/halangan
        ddmrController.SetTargetVelocities(defaultForwardSpeed, 0f);

        // Opsi 2: Aktif mencari dinding terdekat (misal belok pelan ke arah terakhir dinding terlihat?)
        // (Implementasi lebih kompleks, bisa ditambahkan nanti jika perlu)
    }

     // --- Fungsi Helper: Update Mapping ---
     void UpdateMappingFromScan(Vector3 rayOrigin) {
          if (gridMappingSystem == null) return;
          // (Implementasi sama seperti sebelumnya)
         float angleStep = (numberOfRays > 1) ? detectionAngle / (numberOfRays - 1) : 0f;
         float startAngle = -detectionAngle / 2f;
         for (int i = 0; i < numberOfRays; i++) {
              float currentAngleDeg = (numberOfRays == 1) ? 0f : startAngle + i * angleStep;
              Quaternion rotation = Quaternion.AngleAxis(currentAngleDeg, transform.up);
              Vector3 direction = rotation * transform.forward;
              RaycastHit hit;
              bool hitOccurred = Physics.Raycast(rayOrigin, direction, out hit, raycastDistance, obstacleLayer);
              if (hitOccurred) {
                   gridMappingSystem.MarkCellAsObstacle(hit.point);
                   Vector3 pointBeforeHit = rayOrigin + direction * (hit.distance - gridMappingSystem.CellSize * 0.5f);
                   gridMappingSystem.MarkLineAs(rayOrigin, pointBeforeHit, GridMappingSystem.CellStatus.Free);
              } else {
                   gridMappingSystem.MarkLineAs(rayOrigin, rayOrigin + direction * raycastDistance, GridMappingSystem.CellStatus.Free);
              }
         }
     }

} // Akhir Class