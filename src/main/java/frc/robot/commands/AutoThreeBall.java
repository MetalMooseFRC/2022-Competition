// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Runs a three ball auto from the wall side
public class AutoThreeBall extends ParallelCommandGroup {
  /** Creates a new TwoBallAutonomous. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  private Turret m_turret;

  public AutoThreeBall(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, Gate gate, Collector collector, Turret turret) {
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
        new AutonomousShootingAtSpeed(3000, m_shooter, m_gate, m_lifter, m_loader),
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
        new AutonomousShootingAtSpeed(3140, m_shooter, m_gate, m_lifter, m_loader)
    ));
  }
}
