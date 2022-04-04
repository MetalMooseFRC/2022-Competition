// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class AutoFiveBallWall extends ParallelCommandGroup {
  /** Creates a new AutoThreeBallAlongSide. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  private Turret m_turret;

  public AutoFiveBallWall(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, Gate gate, Collector collector, Turret turret) {
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
      
      new InstantCommand(() -> m_turret.turretMotor.set(Constants.Turret.DEFAULT_SPEED)).until(() -> m_turret.limelightHasValidTarget() == true)  
        .andThen(new TrackTargetWithLimelight(m_turret)),
      new SequentialCommandGroup(
        new InstantCommand(() -> m_collector.collect(), m_collector),
        new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
        new DriveStraight(m_drivetrain, 1.1, AUTO_DRIVE_SPEED+0.1),
        new DriveStraight(m_drivetrain, 1, AUTO_DRIVE_SPEED-0.15).until(() -> (m_lifter.getColorLower() != "None")),
        new WaitCommand(0.2),
        new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader),
        //Shoot 2 balls

        new InstantCommand(() -> m_drivetrain.resetYaw(), m_drivetrain),
        new TurnToAngle(120, m_drivetrain),
        new DriveStraight(m_drivetrain, 5.75, AUTO_DRIVE_SPEED+0.1),

        new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
        new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED), m_lifter),
        new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9), m_loader),
        
        new TurnToBall(() -> 0, m_drivetrain).withTimeout(1.5),

        new DriveArcade(() -> AUTO_DRIVE_SPEED-0.15, () -> 0, m_drivetrain).until(() -> (m_lifter.getColorLower() != "None")).withTimeout(1),
        new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
        new InstantCommand(() -> m_loader.setMotorPower(0), m_loader),
        new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader),
        //Shoot 1 Ball
        
        new InstantCommand(() -> m_drivetrain.resetYaw(), m_drivetrain),
        new TurnToAngle(-17.5, m_drivetrain),
        new TurnToBall(() -> 0, m_drivetrain).withTimeout(0.5),
        new DriveStraight(m_drivetrain, 1.5, AUTO_DRIVE_SPEED),
        new DriveStraight(m_drivetrain, 5.5, AUTO_DRIVE_SPEED+0.65),
        new DriveStraight(m_drivetrain, 3, AUTO_DRIVE_SPEED),
        new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
        new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED), m_lifter),
        new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9), m_loader),

        new TurnToBall(() -> 0, m_drivetrain).withTimeout(0.5),
        new DriveArcade(() -> AUTO_DRIVE_SPEED-0.3, () -> 0, m_drivetrain).until(() -> (m_lifter.getColorLower() != "None")).withTimeout(1.3),
        new DriveArcade(() -> 0, () -> 0, m_drivetrain).withTimeout(1.2),

        new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
        new InstantCommand(() -> m_loader.setMotorPower(0), m_loader),
        //Picks up 2 balls

        new InstantCommand(() -> m_drivetrain.resetYaw(), m_drivetrain),
        new DriveStraight(m_drivetrain, -7, -AUTO_DRIVE_SPEED-0.6).withTimeout(2),
        new InstantCommand(() -> m_gate.setGate(0), m_gate),
        new DriveStraight(m_drivetrain, -2, -AUTO_DRIVE_SPEED-0.2).withTimeout(2),
        new DriveStraight(m_drivetrain, -1.5, -AUTO_DRIVE_SPEED+0.2),
        new WaitCommand(0.2).withTimeout(0.2),
        new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader),
        new InstantCommand(() -> m_gate.setGate(0), m_gate),
        new InstantCommand(() -> m_collector.stopCollecting()))
    );
  }
}
