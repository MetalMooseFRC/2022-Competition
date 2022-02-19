// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterControl extends CommandBase {
  private final Shooter m_shooter;

  private final DoubleSupplier m_speedSupplier;

  private final PIDController m_pidController = new PIDController(Constants.Shooter.KP, Constants.Shooter.KI, Constants.Shooter.KD);
  /** Creates a new ShooterControl. */
  public ShooterControl(DoubleSupplier speedSupplier, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speedSupplier = speedSupplier;

    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setSetpoint(m_speedSupplier.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tells the PID loop how fast the wheels are
    double m_leftPID = m_pidController.calculate(m_shooter.getLeftWheelSpeed());
    double m_rightPID = m_pidController.calculate(m_shooter.getRightWheelSpeed());

    m_shooter.setLeftWheelSpeed(m_leftPID);
    m_shooter.setRightWheelSpeed(m_rightPID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
