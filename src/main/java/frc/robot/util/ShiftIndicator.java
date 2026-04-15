package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ============================================================================
 * SHIFT INDICATOR - 2026 REBUILT Sira Sistemi
 * ============================================================================
 * 7536 HubShiftUtil.java'dan uyarlanmistir.
 *
 * 2026 REBUILT kuralinda teleop 140 saniyedir ve 6 shift'e bolunmustur:
 *   Transition  (0-10s)   : Her iki ittifak da aktif
 *   Shift 1     (10-35s)  : Ilk baslayan aktif
 *   Shift 2     (35-60s)  : Diger aktif
 *   Shift 3     (60-85s)  : Ilk baslayan aktif
 *   Shift 4     (85-110s) : Diger aktif
 *   Endgame     (110-140s): Her iki ittifak da aktif
 *
 * Hangi ittifakin ilk basladigini FMS belirler:
 *   DriverStation.getGameSpecificMessage() -> 'R' = Blue ilk baslar
 *                                          -> 'B' = Red ilk baslar
 *   (Az skor yapan once baslar - telafi mantigi)
 *
 * Dashboard ciktilari ("Shift/" prefix):
 *   - Shift/OurTurn     : boolean (Elastic'te yesil/kirmizi gosterge)
 *   - Shift/Status      : string  ("BIZIM SIRA" / "KARSI TARAF" / "IKISI DE")
 *   - Shift/TimeLeft    : double  (mevcut shift'te kalan sure)
 *   - Shift/CurrentShift: string  (hangi shift'te oldugumuz)
 * ============================================================================
 */
public class ShiftIndicator {

    public enum Shift {
        TRANSITION,  // 0-10s   ikisi de aktif
        SHIFT_1,     // 10-35s  ilk baslayan
        SHIFT_2,     // 35-60s  diger
        SHIFT_3,     // 60-85s  ilk baslayan
        SHIFT_4,     // 85-110s diger
        ENDGAME,     // 110-140s ikisi de aktif
        AUTO,
        DISABLED
    }

    // Shift zamanlari (saniye, teleop basindan itibaren)
    private static final double[] SHIFT_START = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] SHIFT_END   = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    // Ilk baslayan ittifak icin takvim
    // Transition=aktif, Shift1=aktif, Shift2=pasif, Shift3=aktif, Shift4=pasif, Endgame=aktif
    private static final boolean[] FIRST_SCHEDULE  = {true, true, false, true, false, true};
    // Ikinci ittifak icin takvim (ters)
    private static final boolean[] SECOND_SCHEDULE = {true, false, true, false, true, true};

    private static final Shift[] SHIFT_VALUES = Shift.values();

    // Timer — 7536 gibi kendi timer'imizi tutuyoruz
    private static final Timer shiftTimer = new Timer();
    private static double timerOffset = 0.0;

    // FMS timer ile senkronizasyon esigi
    private static final double SYNC_THRESHOLD = 3.0;

    // ========================================================================
    // PUBLIC API
    // ========================================================================

    /** Teleop baslarken cagir — timer'i sifirlar */
    public static void initialize() {
        timerOffset = 0.0;
        shiftTimer.restart();
    }

    /**
     * robotPeriodic'te cagir — shift durumunu hesaplar ve dashboard'a yazar.
     * Sadece teleop sirasinda anlamli deger verir.
     */
    public static void update() {
        boolean ourTurn;
        String status;
        double timeLeft;
        String shiftName;

        if (DriverStation.isAutonomousEnabled()) {
            ourTurn = true;
            status = "OTONOM";
            timeLeft = DriverStation.getMatchTime();
            shiftName = "AUTO";
        } else if (DriverStation.isTeleopEnabled()) {
            // Mevcut zamani hesapla
            double currentTime = getCurrentTeleopTime();

            // Hangi shift'teyiz?
            int idx = getShiftIndex(currentTime);
            Shift currentShift = SHIFT_VALUES[idx];
            shiftName = currentShift.name();

            // Bizim takvimimizi al
            boolean[] schedule = getOurSchedule();
            boolean active = schedule[idx];

            // Kalan sure
            timeLeft = SHIFT_END[idx] - currentTime;
            if (timeLeft < 0) timeLeft = 0;

            // Eger sonraki shift de ayni durumdaysa, sure'yi birlestir (7536 mantigi)
            if (idx < SHIFT_END.length - 1 && schedule[idx] == schedule[idx + 1]) {
                timeLeft = SHIFT_END[idx + 1] - currentTime;
            }

            ourTurn = active;

            if (idx == 0 || idx == 5) {
                // Transition veya Endgame — ikisi de aktif
                status = "IKISI DE AKTIF";
            } else if (active) {
                status = "BIZIM SIRA";
            } else {
                status = "KARSI TARAF";
            }
        } else {
            ourTurn = false;
            status = "DISABLED";
            timeLeft = 0;
            shiftName = "DISABLED";
        }

        // Shift timer formatlama (0:25 formatinda)
        int mins = (int) (timeLeft / 60);
        int secs = (int) (timeLeft % 60);
        String timerFormatted = String.format("%d:%02d", mins, secs);

        // Dashboard'a yaz
        SmartDashboard.putBoolean("Shift/OurTurn", ourTurn);
        SmartDashboard.putString("Shift/Status", status);
        SmartDashboard.putNumber("Shift/TimeLeft", timeLeft);
        SmartDashboard.putString("Shift/Timer", timerFormatted);
        SmartDashboard.putString("Shift/CurrentShift", shiftName);
    }

    /** Bizim siramiz mi? (ShootCommand veya baska yerden hizli erisim) */
    public static boolean isOurTurn() {
        if (!DriverStation.isTeleopEnabled()) return true; // auto/disabled'da engel yok
        double currentTime = getCurrentTeleopTime();
        int idx = getShiftIndex(currentTime);
        return getOurSchedule()[idx];
    }

    // ========================================================================
    // INTERNAL
    // ========================================================================

    /**
     * FMS'den hangi ittifakin ilk basladigini oku.
     * 7536 mantigi: 'R' = Blue ilk baslar, 'B' = Red ilk baslar
     * (Az skor yapan ittifak ilk aktif shift'i alir)
     */
    private static Alliance getFirstActiveAlliance() {
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char ch = message.charAt(0);
            if (ch == 'R') return Alliance.Blue;  // Red fazla yaptı → Blue once
            if (ch == 'B') return Alliance.Red;   // Blue fazla yaptı → Red once
        }
        // FMS mesaji yoksa (practice match): kendi ittifakimizin KARSI tarafini ver
        // Boylece practice'te de shift gorunur (biz 2. oluruz)
        Alliance us = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (us == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
    }

    /** Bizim ittifakimiz icin dogru takvimi dondur */
    private static boolean[] getOurSchedule() {
        Alliance firstActive = getFirstActiveAlliance();
        Alliance us = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (firstActive == us) ? FIRST_SCHEDULE : SECOND_SCHEDULE;
    }

    /** Teleop basindan itibaren gecen sureyi hesapla (FMS senkronizasyonlu) */
    private static double getCurrentTeleopTime() {
        double timerValue = shiftTimer.get();
        double currentTime = timerValue - timerOffset;

        // FMS ile senkronize et (7536 mantigi)
        // DriverStation.getMatchTime() geri sayar, biz ileri sayiyoruz
        double fmsTime = 140.0 - DriverStation.getMatchTime();
        if (Math.abs(fmsTime - currentTime) >= SYNC_THRESHOLD
                && fmsTime <= 135.0
                && DriverStation.isFMSAttached()) {
            timerOffset += currentTime - fmsTime;
            currentTime = fmsTime;
        }

        return currentTime;
    }

    /** Verilen zamana gore hangi shift index'inde oldugunu bul */
    private static int getShiftIndex(double time) {
        for (int i = 0; i < SHIFT_START.length; i++) {
            if (time >= SHIFT_START[i] && time < SHIFT_END[i]) {
                return i;
            }
        }
        // Zaman disindaysa son shift (endgame)
        return SHIFT_START.length - 1;
    }
}
