// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterControl extends CommandBase {

  private final Shooter m_shooter;
  private final DoubleSupplier m_speedSupplier;
  
  /** Creates a new ShooterControl. */
  public ShooterControl(DoubleSupplier speedSupplier, Shooter shooter) {
    m_speedSupplier = speedSupplier;
    
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set shooter wheels to calculated speeds
    m_shooter.m_motorLeft.set(m_speedSupplier.getAsDouble());
    m_shooter.m_motorRight.set(m_speedSupplier.getAsDouble());
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
