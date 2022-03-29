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

// Runs a four ball autonomous. Robot must be positioned in specific place on tarmac wall
public class AutoFourBallWall extends SequentialCommandGroup {
  /** Creates a new AutoThreeBallAlongSide. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  private Turret m_turret;

  public AutoFourBallWall(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, Gate gate, Collector collector, Turret turret) {
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
      new ParallelCommandGroup(
        new InstantCommand(() -> m_turret.turretMotor.set(Constants.Turret.DEFAULT_SPEED)).andThen(new TrackTargetWithLimelight(m_turret)),
      new SequentialCommandGroup(
        new InstantCommand(() -> m_collector.collect(), m_collector),
        new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
        new DriveStraight(m_drivetrain, 1.3, AUTO_DRIVE_SPEED+0.1),
        new DriveStraight(m_drivetrain, 1, AUTO_DRIVE_SPEED-0.1).until(() -> (m_lifter.getColorLower() == DriverStation.getAlliance().toString())),
        new WaitCommand(0.2),
        new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader),
        //Shoot 2 balls

        new InstantCommand(() -> m_drivetrain.resetYaw()),
        new TurnToAngle(100, m_drivetrain),
        new DriveStraight(m_drivetrain, 11, AUTO_DRIVE_SPEED),
        new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
        new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED), m_lifter),
        new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9), m_loader),
        new TurnToBall(() -> 0, m_drivetrain),
        new DriveArcade(() -> AUTO_DRIVE_SPEED-0.15, () -> 0, m_drivetrain).until(() -> (m_lifter.getColorLower() == DriverStation.getAlliance().toString())).withTimeout(1.2),
        new InstantCommand(() -> m_drivetrain.resetYaw()),
        new TurnToAngle(-90, m_drivetrain),
        new WaitCommand(0.6),
        new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
        new InstantCommand(() -> m_loader.setMotorPower(0), m_loader),
        new DriveStraight(m_drivetrain, -6, -AUTO_DRIVE_SPEED-0.6).withTimeout(2),
        new InstantCommand(() -> m_gate.setGate(0), m_gate),
        new DriveStraight(m_drivetrain, -2, -AUTO_DRIVE_SPEED-0.2).withTimeout(3),
        new DriveStraight(m_drivetrain, -1.5, -AUTO_DRIVE_SPEED+0.2),
        new WaitCommand(0.3),
        new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader),
        new InstantCommand(() -> m_gate.setGate(0), m_gate),
        new InstantCommand(() -> m_collector.stopCollecting())
          
        )//end SequentialCommandGroup
      )//end ParallellCommandGroup
    );//end addCommands
  }
}
