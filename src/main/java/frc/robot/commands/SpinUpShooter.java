// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class SpinUpShooter extends CommandBase {

  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Lifter m_lifter;


  /** Creates a new SpinUpShooter. */
  public SpinUpShooter(Shooter shooter, Turret turret, Lifter lifter) {

    m_shooter = shooter;
    m_turret = turret;
    m_lifter = lifter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_turret.limelightHasValidTarget() 
        && (m_turret.getTurretDistance()<525) 
        && (m_turret.getTurretDistance()>310) 
        && m_lifter.getColorUpper() == DriverStation.getAlliance().toString())
      {
        m_shooter.setShooterSpeed(m_turret.getRequiredVelocity());
      } else {
        m_shooter.setShooterSpeed(1000);
      }
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
