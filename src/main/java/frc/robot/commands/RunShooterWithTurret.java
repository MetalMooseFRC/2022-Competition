// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RunShooterWithTurret extends CommandBase {

  private final Shooter m_shooter;
  private final Turret m_turret;
  
  /** Creates a new ShooterControl. */
  public RunShooterWithTurret(Shooter shooter, Turret turret) {
    m_shooter = shooter;

    m_turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = m_turret.getRequiredVelocity();
    // Set shooter wheels to calculated speeds
    if (velocity < 0) {
      m_shooter.setShooterSpeed(0);
    } 
    else {
      m_shooter.setShooterSpeed(velocity);
    }
    SmartDashboard.putNumber("Set Shooter Velocity", velocity);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
