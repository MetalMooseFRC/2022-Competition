// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;

import static frc.robot.Constants.Gate.*;
import static frc.robot.Constants.Auto.*;
import frc.robot.Constants;

// Runs a five ball autonomous. Robot must be positioned in specific place on tarmac wall
public class AutoFiveBallBlue extends SequentialCommandGroup {
  /** Creates a new AutoThreeBallAlongSide. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  private Turret m_turret;

  public AutoFiveBallBlue(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, Gate gate, Collector collector, Turret turret) {
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    m_lifter = lifter;
    m_loader = loader;
    m_gate = gate;
    m_collector = collector;
    m_turret = turret;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_turret.turretMotor.set(Constants.Turret.DEFAULT_SPEED)).until(() -> m_turret.limelightHasValidTarget() == true),  
          new TrackTargetWithLimelight(m_turret)),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_collector.collect(), m_collector),
          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
          new DriveStraight(m_drivetrain, 1.1, AUTO_DRIVE_SPEED+0.1),
          new DriveStraight(m_drivetrain, 1, AUTO_DRIVE_SPEED-0.15).until(() -> (m_lifter.getColorLower() != "None")),
          new WaitCommand(0.2),
          new AutonomousShootingAtSpeed(3000, m_shooter, m_gate, m_lifter, m_loader),
          //Shoot 2 balls

          new TurnToAngle(99, m_drivetrain),
          // new TurnToAngleWithPID(99, m_drivetrain, 0.015, 0.005, 0.00),
          new DriveAtAngle(m_drivetrain, 5, AUTO_DRIVE_SPEED+0.1, 99),
          // new DriveStraight(m_drivetrain, 5.25, AUTO_DRIVE_SPEED+0.1),

          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
          new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED), m_lifter),
          new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9), m_loader),
          
          new DriveAtAngle(m_drivetrain, 1.5, AUTO_DRIVE_SPEED-0.15, 99),
          // new DriveArcade(() -> AUTO_DRIVE_SPEED-0.15, () -> 0, m_drivetrain).until(() -> (m_lifter.getColorLower() != "None")).withTimeout(1),
          new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
          new InstantCommand(() -> m_loader.setMotorPower(0), m_loader),
          new AutonomousShootingAtSpeed(3200, m_shooter, m_gate, m_lifter, m_loader),
          //Shoot 1 Ball

          new DriveAtAngle(m_drivetrain, 9.7, AUTO_DRIVE_SPEED+0.1, 95),
          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),

          new ParallelCommandGroup(
            new SequentialCommandGroup(
              new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED), m_lifter),
              new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9), m_loader),
              new WaitUntilCommand(() -> m_lifter.getColorUpper() != "None").withTimeout(2),
              new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
              new InstantCommand(() -> m_loader.setMotorPower(0), m_loader)
            ),
            new SequentialCommandGroup(
              new TurnToAngle(45, m_drivetrain).withTimeout(0.5),
              new DriveAtAngle(m_drivetrain, 4.2, AUTO_DRIVE_SPEED-0.2, 45),
              new DriveAtAngle(m_drivetrain, -0.75, -0.4, 45),
              new WaitCommand(0.5)
              )
            )
          )),
        new TurnToAngle(75, m_drivetrain),
        new DriveAtAngle(m_drivetrain, -7, -AUTO_DRIVE_SPEED-0.1, 85),
        new ParallelRaceGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> m_turret.turretMotor.set(0.3)),
            new TrackTargetWithLimelight(m_turret)
          ),
          new DriveAtAngle(m_drivetrain, -4, -AUTO_DRIVE_SPEED+0.1, 85)
        ),
        new WaitCommand(0.3),
        new AutonomousShootingAtSpeed(3300, m_shooter, m_gate, m_lifter, m_loader)
    );
  }
}
