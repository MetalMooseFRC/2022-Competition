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
public class ManualShootingSequence extends SequentialCommandGroup {
  /** Creates a new ManualShootingSequence. */
  private Shooter m_shooter;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  // private Turret m_turret;
  /** Creates a new ShootingSequence. */
  public ManualShootingSequence(Shooter shooter, Turret turret, Gate gate, Lifter lifter, Loader loader) {
    m_shooter = shooter;
    m_lifter = lifter;
    m_loader = loader;
    m_gate = gate;
    // m_turret = turret;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
    new InstantCommand(() -> m_shooter.setShooterSpeed(Constants.Shooter.SHOOTER_DEFAULT_SPEED), m_shooter),
    new WaitUntilCommand(() -> ((m_shooter.getLeftWheelSpeed()) >= Constants.Shooter.SHOOTER_DEFAULT_SPEED*0.85)),

      //Code Block 4/4/22 added
    new ConditionalCommand(
      new InstantCommand(() -> m_gate.setGate(Constants.Gate.GATE_DEFAULT_SPEED), m_gate),
      new InstantCommand(() -> m_gate.setGate(-Constants.Gate.GATE_DEFAULT_SPEED), m_gate),
      () -> (DriverStation.getAlliance().toString() == m_lifter.getColorLower())),
    //Block end


    // new InstantCommand(() -> m_gate.setGate(Constants.Gate.GATE_DEFAULT_SPEED), m_gate),
    new InstantCommand(() -> m_lifter.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED)),
    new InstantCommand(() -> m_loader.setMotorPower(Constants.Lifter.LIFTER_DEFAULT_SPEED*10/9)),

    new WaitCommand(Constants.Auto.LIFTLOAD_AUTO_TIMEOUT-0.15),

    // new InstantCommand(() -> m_shooter.setShooterSpeed(0), m_shooter),
    new InstantCommand(() -> m_shooter.setShooterSpeed(2500), m_shooter),
    new InstantCommand(() -> m_gate.setGate(0), m_gate),
    new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter),
    new InstantCommand(() -> m_loader.setMotorPower(0), m_loader));
  }
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0);
    // m_shooter.setShooterSpeed(2500);
    m_gate.setGate(0);
    m_lifter.setMotorPower(0);
    m_loader.setMotorPower(0);
  }
}
