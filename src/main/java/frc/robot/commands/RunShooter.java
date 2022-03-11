// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RunShooter extends CommandBase {

  private final Shooter m_shooter;
  private final DoubleSupplier m_speedSupplier;
  private final Turret m_turret;
  
  /** Creates a new ShooterControl. */
  public RunShooter(DoubleSupplier speedSupplier, Shooter shooter, Turret turret) {
    m_speedSupplier = speedSupplier;
    
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
    double dis = m_turret.limelightGetDistance();

    double power = 2.26 + -8.55E-03*dis + 1.11E-05*Math.pow(dis,2);
    //double power = 0.0173 + 2.29E-03*dis + 2.31E-06*Math.pow(dis,2) + -1.44E-08*Math.pow(dis,3) + 1.3E-11*Math.pow(dis,4);
    // Set shooter wheels to calculated speeds
    if (m_speedSupplier.getAsDouble() == 0.0) {
      power = 0.0;
    }
    m_shooter.m_motorLeft.set(power);
    m_shooter.m_motorRight.set(power);
    // m_shooter.m_motorLeft.set(m_shooter.getSliderValue());
    // m_shooter.m_motorRight.set(m_shooter.getSliderValue());
    SmartDashboard.putNumber("Shooter Power", power);
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
