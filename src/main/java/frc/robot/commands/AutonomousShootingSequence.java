// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// The process to shoot 1-2 loaded balls properly during autonomous (handles all timings and speeds)
public class AutonomousShootingSequence extends SequentialCommandGroup {
  /** Creates a new AutonomousShootingSequence. */
  private Shooter m_shooter;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Turret m_turret;
  /** Creates a new ShootingSequence. */
  public AutonomousShootingSequence(Shooter shooter, Turret turret, Gate gate, Lifter lifter, Loader loader) {
    m_shooter = shooter;
    m_lifter = lifter;
    m_loader = loader;
    m_gate = gate;
    m_turret = turret;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //FIXME: BEFORE COMPETITION MAKE SURE TO SWAP THE COMMENTING ON THIS SO THAT OUR BALLS ACTUALLY MAKE IT IN INSTEAD OF BEING BURPED

      // new SequentialCommandGroup(
      //   new InstantCommand(() -> m_shooter.setShooterSpeed(m_turret.getRequiredVelocity()), m_shooter),
      //   new WaitUntilCommand(() -> ((m_shooter.getLeftWheelSpeed()) >= (m_turret.getRequiredVelocity()*Constants.Shooter.SHOOTING_SPEED_THRESHOLD)))),

      new SequentialCommandGroup(
        new InstantCommand(() -> m_shooter.setShooterSpeed(2500), m_shooter),
        new WaitUntilCommand(() -> ((m_shooter.getLeftWheelSpeed()) >= (2000)))),

      new InstantCommand(() -> m_gate.setGate(Constants.Gate.GATE_DEFAULT_SPEED), m_gate),
      new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED)),
      new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*10/9)),
      new WaitCommand(Constants.Auto.LIFTLOAD_AUTO_TIMEOUT),
      new InstantCommand(() -> m_shooter.setShooterSpeed(0), m_shooter),
      new InstantCommand(() -> m_gate.setGate(0), m_gate),
      new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
      new InstantCommand(() -> m_loader.setMotorPower(0), m_loader)
    );
  }
}
