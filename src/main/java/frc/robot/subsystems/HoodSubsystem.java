package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * HOOD SUBSYSTEM - WCP Big Dumper 2026 (WCP Birebir Uyumlu)
 * ============================================================================
 *
 * WCP Hood.java'dan eklenen kritik fark:
 *   - currentPosition takibi: Servo fiziksel olarak hareket eder, anında ulaşamaz.
 *     WCP, kServoLength + kMaxServoSpeed ile gerçek pozisyonu simüle eder.
 *   - isPositionWithinTolerance() buna göre çalışır → PrepareShotCommand doğru bekler.
 *
 * Donanım:
 *   Sol Servo:  PWM 3
 *   Sağ Servo:  PWM 4
 *
 * Pozisyon aralığı: 0.01 (en düşük açı) - 0.77 (en yüksek açı)
 * ============================================================================
 */
public class HoodSubsystem extends SubsystemBase {

    // ========================================================================
    // PWM PORTLARI
    // ========================================================================
    public static final int LEFT_SERVO_PWM  = 3;
    public static final int RIGHT_SERVO_PWM = 4;

    // ========================================================================
    // SABİTLER - WCP Hood.java ile birebir
    // ========================================================================

    /** Servo fiziksel uzunluğu mm cinsinden (WCP: 100mm) */
    private static final double SERVO_LENGTH_MM = 100.0;

    /**
     * Servoların maksimum hareket hızı (mm/s).
     * WCP: 20mm/s → 1 saniyelik harekette 20/100 = 0.20 pozisyon birimi
     */
    private static final double MAX_SERVO_SPEED_MM_PER_S = 20.0;

    /** Minimum servo pozisyonu (WCP: 0.01) */
    public static final double MIN_POSITION = 0.01;

    /** Maksimum servo pozisyonu (WCP: 0.77) */
    public static final double MAX_POSITION = 0.77;

    /** Varsayılan başlangıç pozisyonu */
    public static final double DEFAULT_POSITION = 0.10;

    /**
     * Pozisyon toleransı - WCP: 0.01
     * (eski kodunuzda 0.02 idi, WCP ile uyumlu olsun diye 0.01 yapıldı)
     */
    private static final double POSITION_TOLERANCE = 0.01;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final Servo leftServo;
    private final Servo rightServo;

    // ========================================================================
    // STATE - WCP Hood.java ile aynı mantık
    // ========================================================================
    /** Gerçek (simüle edilmiş) pozisyon - hareket hızına göre güncellenir */
    private double currentPosition = DEFAULT_POSITION;

    /** Hedef pozisyon */
    private double targetPosition = DEFAULT_POSITION;

    /** Son güncelleme zamanı (saniye) */
    private double lastUpdateTime = Timer.getFPGATimestamp();

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public HoodSubsystem() {
        leftServo  = new Servo(LEFT_SERVO_PWM);
        rightServo = new Servo(RIGHT_SERVO_PWM);

        // WCP ile aynı PWM sınırları
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        setPosition(DEFAULT_POSITION);

        System.out.println("[Hood] Initialized - PWM " + LEFT_SERVO_PWM + " / " + RIGHT_SERVO_PWM);
    }

    // ========================================================================
    // KONTROL
    // ========================================================================

    /**
     * Hood'u belirtilen pozisyona ayarlar.
     * @param position 0.0 - MAX_POSITION (0.77)
     */
    public void setPosition(double position) {
        targetPosition = MathUtil.clamp(position, MIN_POSITION, MAX_POSITION);
        leftServo.set(targetPosition);
        rightServo.set(targetPosition);
    }

    /** Varsayılan pozisyona döner. */
    public void setDefault() {
        setPosition(DEFAULT_POSITION);
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    public double getTargetPosition()  { return targetPosition; }
    public double getCurrentPosition() { return currentPosition; }

    /**
     * Hood hedef pozisyona ulaştı mı?
     *
     * WCP mantığı: currentPosition gerçek zamanla güncellenir,
     * tolerans kontrolü fiziksel hareketi dikkate alır.
     * Böylece PrepareShotCommand doğru anda ateş eder.
     */
    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, POSITION_TOLERANCE);
    }

    // ========================================================================
    // WCP: updateCurrentPosition() - FPGA timestamp tabanlı hareket simülasyonu
    // ========================================================================

    /**
     * Her döngüde gerçek servo hızına göre currentPosition'ı günceller.
     * WCP Hood.java'dan birebir alınan mantık.
     *
     * Servo 20mm/s hızda hareket eder, 100mm servo uzunluğu var.
     * Yani max hız = 20/100 = 0.20 pozisyon birimi/saniye.
     */
    private void updateCurrentPosition() {
        final double now = Timer.getFPGATimestamp();
        final double dt = now - lastUpdateTime;
        lastUpdateTime = now;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        // Geçen sürede servo ne kadar hareket edebilir?
        final double maxDistanceTraveled = (MAX_SERVO_SPEED_MM_PER_S / SERVO_LENGTH_MM) * dt;

        if (targetPosition > currentPosition) {
            currentPosition = Math.min(targetPosition, currentPosition + maxDistanceTraveled);
        } else {
            currentPosition = Math.max(targetPosition, currentPosition - maxDistanceTraveled);
        }
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        updateCurrentPosition();

        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Hood/TargetPos",   Math.round(targetPosition   * 1000.0) / 1000.0);
        SmartDashboard.putNumber("Hood/CurrentPos",  Math.round(currentPosition  * 1000.0) / 1000.0);
        SmartDashboard.putBoolean("Hood/AtTarget",   isPositionWithinTolerance());
    }
}