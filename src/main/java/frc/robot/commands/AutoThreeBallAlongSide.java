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

public class AutoThreeBallAlongSide extends SequentialCommandGroup {
  /** Creates a new AutoThreeBallAlongSide. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  private Turret m_turret;

  public AutoThreeBallAlongSide(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, Gate gate, Collector collector, Turret turret) {
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
        
        new InstantCommand(() -> m_turret.turretMotor.set(Constants.Turret.DEFAULT_SPEED))
          
          .andThen(new TrackTargetWithLimelight(m_turret)),
        
          new SequentialCommandGroup(
                    //Start Collecting
          new InstantCommand(() -> m_collector.collect(), m_collector),
          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),

          new DriveStraight(m_drivetrain, TWO_BALL_AUTO_DRIVE_DISTANCE, AUTO_DRIVE_SPEED),
                    //Stop Collecting
          new InstantCommand(() -> m_collector.stopCollecting(), m_collector),
          new InstantCommand(() -> m_gate.setGate(0), m_gate),
                    //Start Shooting Sequence
          new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader),        //end shooting sequence and 2 ball
                    //Turn to face 3rd ball
          new TurnToAngle(THREE_BALL_AUTO_FIRST_TURN + m_drivetrain.getAngle(), m_drivetrain),
                    //Start Collecting
          new InstantCommand(() -> m_collector.collect(), m_collector),
          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
          new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED)),
          new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*-10/9)),
                    //Drive to 3rd ball
          new DriveStraight(m_drivetrain, 16, AUTO_DRIVE_SPEED),
          new TurnToBall(() -> AUTO_DRIVE_SPEED, m_drivetrain).until(()-> m_lifter.getColorUpper() == DriverStation.getAlliance().toString()),

          //Turn and Drive towards hub
          new TurnToAngle(THREE_BALL_AUTO_SECOND_TURN + m_drivetrain.getAngle(), m_drivetrain),

          new InstantCommand(() -> m_lifter.setMotorPower(0)),
          new InstantCommand(() -> m_loader.setMotorPower(0)),

          new WaitCommand(1.2),

          new DriveStraight(m_drivetrain, THREE_BALL_AUTO_SECOND_DISTANCE, AUTO_DRIVE_SPEED),
                    //Stop collecting
          new InstantCommand(() -> m_collector.stopCollecting(), m_collector),
          new InstantCommand(() -> m_gate.setGate(0), m_gate),
                    //Shooting sequence
          new AutonomousShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader)
          
        )//end SequentialCommandGroup
      )//end ParallellCommandGroup
    );//end addCommands
  }
}
