using UnityEngine;

// Pastikan GameObject memiliki komponen Rigidbody
[RequireComponent(typeof(Rigidbody))]
public class DDMRController : MonoBehaviour
{
    [Header("Parameter Robot")]
    [Tooltip("Jari-jari roda dalam meter")]
    public float wheelRadius = 0.1f; // meter (r)
    [Tooltip("Jarak antara pusat kedua roda dalam meter")]
    public float axleLength = 0.5f; // meter (L)

    [Header("Batas Kecepatan (Opsional)")]
    [Tooltip("Kecepatan linear maksimum (m/s)")]
    public float maxLinearSpeed = 1.5f; // m/s
    [Tooltip("Kecepatan angular maksimum (rad/s)")]
    public float maxAngularSpeed = Mathf.PI; // rad/s (sekitar 180 derajat/s)

    [Header("Status Debugging (Read Only)")]
    [SerializeField] private float targetLinearVelocity = 0f;
    [SerializeField] private float targetAngularVelocity = 0f; // rad/s
    [SerializeField] private float leftWheelSpeedRad = 0f;      // rad/s (ωL)
    [SerializeField] private float rightWheelSpeedRad = 0f;     // rad/s (ωR)

    public float CurrentLeftWheelSpeedRad => leftWheelSpeedRad;
    public float CurrentRightWheelSpeedRad => rightWheelSpeedRad;

    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Komponen Rigidbody tidak ditemukan pada GameObject ini.", this);
            enabled = false; // Nonaktifkan script jika tidak ada Rigidbody
            return;
        }

        // Validasi parameter dasar
        if (wheelRadius <= 0f)
        {
            Debug.LogError("Wheel Radius harus bernilai positif.", this);
            enabled = false;
            return;
        }
        if (axleLength <= 0f)
        {
            Debug.LogError("Axle Length harus bernilai positif.", this);
            enabled = false;
            return;
        }

        // (Opsional) Konfigurasi Rigidbody untuk pergerakan di bidang datar (XZ plane)
        // rb.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
    }

    void FixedUpdate()
    {
        // 1. Hitung kecepatan roda target berdasarkan kecepatan robot yang diinginkan (Inverse Kinematics)
        CalculateWheelSpeeds(targetLinearVelocity, targetAngularVelocity);

        // 2. Hitung kecepatan robot aktual berdasarkan kecepatan roda (Forward Kinematics)
        //    Kita bisa langsung gunakan targetLinearVelocity dan targetAngularVelocity jika mengasumsikan
        //    kontrol ideal, atau hitung dari leftWheelSpeedRad dan rightWheelSpeedRad untuk simulasi.
        float actualLinearVelocity = (wheelRadius * (rightWheelSpeedRad + leftWheelSpeedRad)) / 2.0f;
        float actualAngularVelocity = (wheelRadius * (rightWheelSpeedRad - leftWheelSpeedRad)) / axleLength; // rad/s

        // 3. Terapkan pergerakan ke Rigidbody
        ApplyMotion(actualLinearVelocity, actualAngularVelocity);
    }

    // --- Inverse Kinematics ---
    // Menghitung kecepatan sudut roda (rad/s) yang diperlukan
    void CalculateWheelSpeeds(float targetLinear, float targetAngular)
    {
        // Batasi kecepatan target jika melebihi batas maksimum
        targetLinear = Mathf.Clamp(targetLinear, -maxLinearSpeed, maxLinearSpeed);
        targetAngular = Mathf.Clamp(targetAngular, -maxAngularSpeed, maxAngularSpeed);

        // Rumus Inverse Kinematics:
        // v_R = v + (ω * L / 2)  (Kecepatan linear roda kanan)
        // v_L = v - (ω * L / 2)  (Kecepatan linear roda kiri)
        float rightWheelLinearVelocity = targetLinear + (targetAngular * axleLength / 2.0f);
        float leftWheelLinearVelocity = targetLinear - (targetAngular * axleLength / 2.0f);

        // Konversi kecepatan linear roda ke kecepatan sudut roda (ω = v / r)
        rightWheelSpeedRad = rightWheelLinearVelocity / wheelRadius;
        leftWheelSpeedRad = leftWheelLinearVelocity / wheelRadius;
    }

    // --- Menerapkan Gerakan ke Rigidbody ---
    void ApplyMotion(float linearVel, float angularVelRad)
    {
        // Kecepatan linear: Terapkan ke arah depan robot
        Vector3 worldLinearVelocity = transform.forward * linearVel;
        rb.velocity = new Vector3(worldLinearVelocity.x, rb.velocity.y, worldLinearVelocity.z); // Pertahankan kecepatan Y dari gravitasi/fisika lain

        // Kecepatan angular: Terapkan sebagai rotasi di sekitar sumbu Y (sumbu atas robot)
        Vector3 worldAngularVelocity = transform.up * angularVelRad; // angularVelRad sudah dalam rad/s
        rb.angularVelocity = new Vector3(rb.angularVelocity.x, worldAngularVelocity.y, rb.angularVelocity.z); // Pertahankan rotasi X dan Z
    }

    // --- Metode Publik untuk Kontrol Otomatis ---

    /// <summary>
    /// Mengatur target kecepatan linear dan angular robot.
    /// </summary>
    /// <param name="linear">Target kecepatan linear (m/s). Positif = maju, negatif = mundur.</param>
    /// <param name="angularRad">Target kecepatan angular (rad/s). Positif = belok kiri, negatif = belok kanan (mengikuti aturan tangan kanan Unity di sumbu Y).</param>
    public void SetTargetVelocities(float linear, float angularRad)
    {
        targetLinearVelocity = Mathf.Clamp(linear, -maxLinearSpeed, maxLinearSpeed);
        targetAngularVelocity = Mathf.Clamp(angularRad, -maxAngularSpeed, maxAngularSpeed);
    }

    /// <summary>
    /// Perintahkan robot bergerak maju dengan kecepatan tertentu.
    /// </summary>
    /// <param name="speed">Kecepatan dalam m/s (harus positif).</param>
    public void MoveForward(float speed)
    {
        SetTargetVelocities(Mathf.Abs(speed), 0f);
    }

    /// <summary>
    /// Perintahkan robot bergerak mundur dengan kecepatan tertentu.
    /// </summary>
    /// <param name="speed">Kecepatan dalam m/s (harus positif).</param>
    public void MoveBackward(float speed)
    {
        SetTargetVelocities(-Mathf.Abs(speed), 0f);
    }

    /// <summary>
    /// Perintahkan robot berbelok (berputar di tempat).
    /// </summary>
    /// <param name="angularSpeedRad">Kecepatan angular dalam rad/s. Positif = belok kiri, negatif = belok kanan.</param>
    public void Turn(float angularSpeedRad)
    {
        SetTargetVelocities(0f, angularSpeedRad);
    }

    /// <summary>
    /// Perintahkan robot berbelok (berputar di tempat) menggunakan input derajat per detik.
    /// </summary>
    /// <param name="angularSpeedDeg">Kecepatan angular dalam derajat/s. Positif = belok kiri, negatif = belok kanan.</param>
    public void TurnDegrees(float angularSpeedDeg)
    {
        Turn(angularSpeedDeg * Mathf.Deg2Rad); // Konversi derajat ke radian
    }

    /// <summary>
    /// Menghentikan semua gerakan robot.
    /// </summary>
    public void Stop()
    {
        SetTargetVelocities(0f, 0f);
        // Reset langsung kecepatan Rigidbody untuk berhenti seketika
        rb.velocity = new Vector3(0, rb.velocity.y, 0); // Pertahankan kecepatan Y
        rb.angularVelocity = Vector3.zero;
        leftWheelSpeedRad = 0f;
        rightWheelSpeedRad = 0f;
    }

    // --- Contoh Penggunaan Sederhana (Bisa dihapus atau dimodifikasi) ---
    // void Update()
    // {
    //     // Contoh kontrol keyboard sederhana untuk testing
    //     float moveInput = Input.GetAxis("Vertical"); // W/S atau Panah Atas/Bawah
    //     float turnInput = Input.GetAxis("Horizontal"); // A/D atau Panah Kiri/Kanan

    //     // Ubah input menjadi target kecepatan
    //     float targetLin = moveInput * maxLinearSpeed;
    //     // Input horizontal positif (D/Kanan) biasanya berarti belok kanan (angular negatif)
    //     float targetAngRad = -turnInput * maxAngularSpeed;

    //     SetTargetVelocities(targetLin, targetAngRad);

    //     // --- Contoh Panggilan Fungsi Otomatis ---
    //     // Tekan angka untuk menguji fungsi gerakan:
    //     // if (Input.GetKeyDown(KeyCode.Alpha1)) MoveForward(0.8f);
    //     // if (Input.GetKeyDown(KeyCode.Alpha2)) MoveBackward(0.5f);
    //     // if (Input.GetKeyDown(KeyCode.Alpha3)) TurnDegrees(90f);  // Belok kiri 90 deg/s
    //     // if (Input.GetKeyDown(KeyCode.Alpha4)) TurnDegrees(-90f); // Belok kanan 90 deg/s
    //     // if (Input.GetKeyDown(KeyCode.Alpha0)) Stop();
    // }
}