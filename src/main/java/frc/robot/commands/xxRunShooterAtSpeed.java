// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class xxRunShooterAtSpeed extends CommandBase {

  private final Shooter m_shooter;
  private final Double m_power;

  
  /** Creates a new ShooterControl. */
  public xxRunShooterAtSpeed(double power, Shooter shooter) {
    m_power = power;
    
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    m_shooter.m_motorLeft.set(m_power);
    m_shooter.m_motorRight.set(m_power);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.m_motorLeft.set(0.0);
    m_shooter.m_motorRight.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
