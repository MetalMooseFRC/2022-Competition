// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants;



public class VisionFollow extends CommandBase {

  private final Turret m_turret;
  private final Shooter m_shooter;

  private final double m_turretkP = Constants.Limelight.PID.kP;
  private final double m_turretkI = Constants.Limelight.PID.kI;
  private final double m_turretkD = Constants.Limelight.PID.kD;

  private final PIDController m_turretPID = new PIDController(m_turretkP, m_turretkI, m_turretkD);

  /** Creates a new VisionFollow. */
  public VisionFollow(Turret turret, Shooter shooter) {
    
    m_turret = turret;
    m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret, m_shooter);

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turretPID.setSetpoint(Constants.Limelight.PID.TURRET_SETPOINT);
    m_turretPID.setTolerance(Constants.Limelight.PID.VISION_ERROR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double m_turretPIDOut = m_turretPID.calculate(m_turret.limelightGetTx());

    m_turret.setTurretSpeed(m_turretPIDOut);
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
