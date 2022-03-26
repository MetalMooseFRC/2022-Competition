// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;
import static frc.robot.Constants.Lifter.*;

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

          new DriveStraight(m_drivetrain, TWO_BALL_AUTO_DRIVE_DISTANCE, TWO_BALL_AUTO_DRIVE_SPEED),
                    //Stop Collecting
          new InstantCommand(() -> m_collector.stopCollecting(), m_collector),
          new InstantCommand(() -> m_gate.setGate(0), m_gate),
                    //Start Shooting Sequence
          new ParallelRaceGroup(
                    //Run Shooter Indefinitely while
            new RunShooterWithTurret(m_shooter, m_turret).withTimeout(LIFTLOAD_AUTO_TIMEOUT+0.1),
                    //Running a sequence
            new SequentialCommandGroup(
                    //Wait until shooter is at speed
              new WaitUntilCommand(
                () -> ((m_shooter.getLeftWheelSpeed()) >= (m_turret.getRequiredVelocity()*Constants.Shooter.SHOOTING_SPEED_THRESHOLD))),
                    //Once shooter is at speed shoot
              new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
              new RunLifterLoader(m_lifter, LIFTER_DEFAULT_SPEED, m_loader, LIFTER_DEFAULT_SPEED*10/9)
                .withTimeout(LIFTLOAD_AUTO_TIMEOUT)
                .andThen(new InstantCommand(() -> m_gate.setGate(0), m_gate)))
          ),        //end shooting sequence and 2 ball
                    //Turn to face 3rd ball
          new TurnToAngle(THREE_BALL_AUTO_FIRST_TURN + m_drivetrain.getAngle(), m_drivetrain),
                    //Start Collecting
          new InstantCommand(() -> m_collector.collect(), m_collector),
          new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
                    //Drive to 3rd ball
          new DriveStraight(m_drivetrain, THREE_BALL_AUTO_FIRST_DISTANCE, TWO_BALL_AUTO_DRIVE_SPEED),
                    //Turn and Drive towards hub
          new TurnToAngle(THREE_BALL_AUTO_SECOND_TURN + m_drivetrain.getAngle(), m_drivetrain),
          new DriveStraight(m_drivetrain, THREE_BALL_AUTO_SECOND_DISTANCE, TWO_BALL_AUTO_DRIVE_SPEED),
                    //Stop collecting
          new InstantCommand(() -> m_collector.stopCollecting(), m_collector),
          new InstantCommand(() -> m_gate.setGate(0), m_gate),
                    //Shooting sequence
          new ParallelRaceGroup(
                    //Run shooter indefinitely
            new RunShooterWithTurret(m_shooter, m_turret).withTimeout(LIFTLOAD_AUTO_TIMEOUT+0.1),
                    //wait until shooter is at speed
            new SequentialCommandGroup(
              new WaitUntilCommand(
                () -> ((m_shooter.getLeftWheelSpeed()) >= (m_turret.getRequiredVelocity()*Constants.Shooter.SHOOTING_SPEED_THRESHOLD))),
                    //shoot ball when shooter is at speed
              new InstantCommand(() -> m_gate.setGate(GATE_DEFAULT_SPEED), m_gate),
              new RunLifterLoader(m_lifter, LIFTER_DEFAULT_SPEED, m_loader, LIFTER_DEFAULT_SPEED*10/9)
                .withTimeout(LIFTLOAD_AUTO_TIMEOUT)
                .andThen(new InstantCommand(() -> m_gate.setGate(0)))
            )
          )
        )//end SequentialCommandGroup
      )//end ParallellCommandGroup
    );//end addCommands
  }
}
