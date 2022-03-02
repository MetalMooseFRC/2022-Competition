// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.MathUtil;

public class TrackTargetWithLimelight extends CommandBase {

  private final Turret m_turret;
  // private final Shooter m_shooter;
  private final PIDController turretController = new PIDController(Constants.Turret.PID.kP, Constants.Turret.PID.kI, Constants.Turret.PID.kD);

  /** Creates a new TrackTargetWithLimelight. */
  public TrackTargetWithLimelight(Turret turret) {

    // m_shooter = shooter;
    m_turret = turret;

    addRequirements(m_turret);
    turretController.setTolerance(Constants.Turret.PID.TOLERANCE, Constants.Turret.PID.TOLERANCE_BUFFER);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Running TrackTargetWithLimelight");
    // double shooterSpeed = m_turret.getShooterSpeed();
    // m_shooter.m_motorLeft.set(shooterSpeed);
    // m_shooter.m_motorRight.set(shooterSpeed);
    if(m_turret.limelightHasValidTarget()) {
      // System.out.println("Not at setpoint");
      double offBy = m_turret.limelightGetTx();
      double power = turretController.calculate(offBy);
      power =  MathUtil.clamp(power, -Constants.Turret.CLAMP, Constants.Turret.CLAMP);
      m_turret.turretMotor.set(-power);
    } else {
      // System.out.println("Searching");
      m_turret.turretMotor.set(Constants.Limelight.SEARCH_SPEED);
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
