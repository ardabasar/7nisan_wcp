package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * LED SUBSYSTEM - WS2812B Adreslenebilir LED
 * ============================================================================
 * RoboRIO PWM 0 portuna baglanir.
 *
 * Modlar:
 *   - IDLE:      Alliance rengi (mavi/kirmizi) yavas nefes efekti
 *   - INTAKE:    Yesil yanip sonme
 *   - SHOOTING:  Turuncu hizli flash
 *   - ALIGNED:   Yesil sabit
 *   - ERROR:     Kirmizi hizli flash
 *   - RAINBOW:   Gokkusagi (disable/beklemede)
 *   - AUTO:      Mor kosu efekti
 * ============================================================================
 */
public class LEDSubsystem extends SubsystemBase {

    // ========================================================================
    // DONANIM
    // ========================================================================
    private static final int PWM_PORT = 0;       // RoboRIO PWM port
    private static final int LED_COUNT = 60;      // LED sayisi - kendi seridine gore degistir
    private static final int MAX_BRIGHTNESS = 80; // Max parlaklik (0-255) - RoboRIO brownout onleme

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    // ========================================================================
    // LED MODLARI
    // ========================================================================
    public enum LEDMode {
        IDLE,       // Alliance rengi nefes
        INTAKE,     // Yesil yanip sonme
        SHOOTING,   // Turuncu flash
        ALIGNED,    // Yesil sabit
        ERROR,      // Kirmizi flash
        RAINBOW,    // Gokkusagi
        AUTO,       // Mor kosu
        OFF         // Kapali
    }

    private LEDMode currentMode = LEDMode.RAINBOW;
    private int rainbowHue = 0;
    private int chaseOffset = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public LEDSubsystem() {
        led = new AddressableLED(PWM_PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(LED_COUNT);
        led.setData(buffer);
        led.start();

        System.out.println("[LED] Initialized - PWM " + PWM_PORT + ", " + LED_COUNT + " LEDs");
    }

    // ========================================================================
    // MOD AYARLA
    // ========================================================================
    public void setMode(LEDMode mode) {
        this.currentMode = mode;
    }

    public LEDMode getMode() {
        return currentMode;
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        switch (currentMode) {
            case IDLE:
                breatheAlliance();
                break;
            case INTAKE:
                flash(0, 255, 0, 4.0);  // Yesil flash
                break;
            case SHOOTING:
                flash(255, 100, 0, 8.0);  // Turuncu hizli flash
                break;
            case ALIGNED:
                solid(0, 255, 0);  // Yesil sabit
                break;
            case ERROR:
                flash(255, 0, 0, 8.0);  // Kirmizi hizli flash
                break;
            case RAINBOW:
                rainbow();
                break;
            case AUTO:
                chase(128, 0, 255);  // Mor kosu
                break;
            case OFF:
                solid(0, 0, 0);
                break;
        }
        led.setData(buffer);
    }

    // ========================================================================
    // LED EFEKTLERI
    // ========================================================================

    /** Tum LED'leri tek renk yap (parlaklik sinirli) */
    private void solid(int r, int g, int b) {
        r = r * MAX_BRIGHTNESS / 255;
        g = g * MAX_BRIGHTNESS / 255;
        b = b * MAX_BRIGHTNESS / 255;
        for (int i = 0; i < LED_COUNT; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    /** Yanip sonme efekti */
    private void flash(int r, int g, int b, double hz) {
        boolean on = (Timer.getFPGATimestamp() * hz) % 1.0 < 0.5;
        if (on) {
            solid(r, g, b);
        } else {
            solid(0, 0, 0);
        }
    }

    /** Alliance renginde nefes efekti (yavas parlayip sonme) */
    private void breatheAlliance() {
        boolean isRed = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed = (alliance.get() == DriverStation.Alliance.Red);
        }

        // Sinusoidal parlaklik (0-255 arasi yavas degisim)
        double t = Timer.getFPGATimestamp();
        int brightness = (int) (127.5 + 127.5 * Math.sin(t * 2.0));

        if (isRed) {
            solid(brightness, 0, 0);
        } else {
            solid(0, 0, brightness);
        }
    }

    /** Gokkusagi efekti */
    private void rainbow() {
        for (int i = 0; i < LED_COUNT; i++) {
            int hue = (rainbowHue + (i * 180 / LED_COUNT)) % 180;
            buffer.setHSV(i, hue, 255, MAX_BRIGHTNESS);
        }
        rainbowHue = (rainbowHue + 3) % 180;
    }

    /** Kosu (chase) efekti */
    private void chase(int r, int g, int b) {
        r = r * MAX_BRIGHTNESS / 255;
        g = g * MAX_BRIGHTNESS / 255;
        b = b * MAX_BRIGHTNESS / 255;
        for (int i = 0; i < LED_COUNT; i++) {
            if ((i + chaseOffset) % 6 < 3) {
                buffer.setRGB(i, r, g, b);
            } else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        chaseOffset++;
    }
}
