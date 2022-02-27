// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.MathUtil;

public class TurnTurretToAngle extends CommandBase {
  /** Creates a new TurnTurretToAngle. */
  private final Turret m_turret;
  private final double m_angle;

  private final PIDController turretController = new PIDController(Constants.Turret.PID.kP, Constants.Turret.PID.kI, Constants.Turret.PID.kD);
  
  public TurnTurretToAngle(double angle, Turret turret) {
    m_angle = angle;
    m_turret = turret;
    addRequirements(m_turret);

    turretController.enableContinuousInput(-180,180);
    turretController.setTolerance(Constants.Turret.PID.TOLERANCE, Constants.Turret.PID.TOLERANCE_BUFFER);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretController.setSetpoint(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = turretController.calculate(m_turret.getTurretAngle());
    power = MathUtil.clamp(power, -Constants.Turret.CLAMP, Constants.Turret.CLAMP);
    m_turret.turretMotor.set(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.turretMotor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretController.atSetpoint();
  }
}
