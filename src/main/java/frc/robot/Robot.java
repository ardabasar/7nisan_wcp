// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * ============================================================================
 * ROBOT.java — 2. Otonom Crash Fix (Kok Neden Cozumu)
 * ============================================================================
 * 
 * PROBLEM:
 *   2. kez otonom enable yapildiginda:
 *   - Gercek robotta: "No robot code" hatasi
 *   - Simulasyonda: Simulasyon coker ve kapanir
 *
 * KOK NEDEN:
 *   AutoBuilder.buildAutoChooser() tarafindan dondurilen SendableChooser,
 *   her getSelected() cagrisinda AYNI Command nesnesini dondurur.
 *   Commands.defer() lambda icerisinde autoChooser.getSelected() cagirilsa bile,
 *   PathPlanner icsel olarak ayni "composed" Command'i cache'ler.
 *   
 *   WPILib kurali: Bir Command "composed" olarak isaretlendikten sonra
 *   tekrar compose veya schedule edilemez.
 *   
 *   IllegalStateException -> CommandScheduler unhandled exception ->
 *   Tum scheduler durur -> "No robot code" / simulasyon crash
 *
 * COZUM:
 *   1) autonomousInit'te CommandScheduler.getInstance().cancelAll() ile
 *      TUM aktif command'ler temizlenir (sadece m_autonomousCommand degil)
 *   2) getAutonomousCommand() icinde .asProxy() kullanilarak
 *      PathPlanner command'i "ProxyCommand" ile wrap edilir.
 *      ProxyCommand her schedule edildiginde taze bir command olusturur
 *      ve "composed" flag'ini bypass eder.
 *   3) autonomousExit + teleopInit'te de cancelAll() yapilir.
 * ============================================================================
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    /**
     * Vision lokalizasyonu VisionSubsystem tarafindan yonetilir.
     * Robot acilir acilmaz (disabled dahil) konum aramaya baslar.
     * Ayrica teleopInit ve autonomousInit'te seed yapilir.
     */

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        // Vision fuzyonu artik VisionSubsystem.periodic() icinde yapiliyor.
        // VisionSubsystem varsayilan olarak ACIK basliyor, robot nerede
        // oldugunu hemen bilir.
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // ================================================================
        // [FIX] KRITIK: Tum aktif command'leri iptal et
        // Sadece m_autonomousCommand degil, TUM command'ler temizlenmeli.
        // Cunku PathPlanner composed command'leri baska subsystem'lere de
        // bagli olabilir ve stale referans birakabilir.
        // ================================================================
        CommandScheduler.getInstance().cancelAll();
        m_autonomousCommand = null;

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            // WPILib 2026: Command.schedule() deprecated -> CommandScheduler kullan
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        // Otonom bitince HER SEYI temizle
        CommandScheduler.getInstance().cancelAll();
        m_autonomousCommand = null;
    }

    @Override
    public void teleopInit() {
        // Otonom'dan kalan her seyi temizle
        CommandScheduler.getInstance().cancelAll();
        m_autonomousCommand = null;

        // Teleop baslarken de vision ile konum dogrula
        // VisionSubsystem zaten acik, ama bir kez daha seed komutu calistir
        // (eger otonom oncesi seed yapilmadiysa veya robot yeni acildiysa)
        if (!m_robotContainer.drivetrain.isVisionSeeded()) {
            CommandScheduler.getInstance().schedule(
                new frc.robot.commands.auto.VisionAutoSeedCommand(
                    m_robotContainer.drivetrain,
                    m_robotContainer.getVision(),
                    "limelight"
                )
            );
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
