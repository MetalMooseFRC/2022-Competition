// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;
import static frc.robot.Constants.Gate.*;
import static frc.robot.Constants.Auto.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFourBallCenterFixed extends SequentialCommandGroup {
  /** Creates a new AutoFourBallCenterFixed. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  private Turret m_turret;
  /** Creates a new AutoFourBallCenter. */
  public AutoFourBallCenterFixed(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, Gate gate, Collector collector, Turret turret) {
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
          new DriveAtAngle(m_drivetrain, 4, AUTO_DRIVE_SPEED-0.15, 0),
        // new DriveStraight(m_drivetrain, 4, AUTO_DRIVE_SPEED-0.15).until(() -> (m_lifter.getColorLower() != "None")),
          new WaitCommand(0.2),
          new AutonomousShootingAtSpeed(3000, m_shooter, m_gate, m_lifter, m_loader),
        //Shoot 2 balls

          new TurnToAngle(30, m_drivetrain),

          new DriveAtAngle(m_drivetrain, 10.3, AUTO_DRIVE_SPEED+0.1, 30),

          new TurnToAngle(-20, m_drivetrain).withTimeout(1.3),

          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
          new ParallelRaceGroup(
            new SequentialCommandGroup(
              new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED), m_lifter),
              new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9), m_loader),
              new WaitUntilCommand(() -> m_lifter.getColorUpper() != "None").withTimeout(2),
              new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
              new RunCommand(() -> m_loader.setMotorPower(0), m_loader)),
            new SequentialCommandGroup(
              new DriveAtAngle(m_drivetrain, 3, 0.4, -20),
              new DriveAtAngle(m_drivetrain, -2, -0.4, -20))),
        
          new TurnToAngle(20, m_drivetrain),
          new DriveAtAngle(m_drivetrain, -3, -AUTO_DRIVE_SPEED, 20),
          new InstantCommand(() -> m_gate.setGate(0), m_gate))
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> m_turret.turretMotor.set(Constants.Turret.DEFAULT_SPEED)).until(() -> m_turret.limelightHasValidTarget() == true).andThen(new TrackTargetWithLimelight(m_turret)),
          new SequentialCommandGroup(
            new DriveAtAngle(m_drivetrain, -7, -AUTO_DRIVE_SPEED, 20),
            new WaitCommand(1),
            new AutonomousShootingAtSpeed(3200, m_shooter, m_gate, m_lifter, m_loader),
        //Shoot 2 more balls
        
            new InstantCommand(() -> m_gate.setGate(0), m_gate),
            new InstantCommand(() -> m_collector.stopCollecting())
          )
        )
      );
  }
}
