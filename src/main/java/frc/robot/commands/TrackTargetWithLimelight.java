// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;

// PID Loop to track (spin turret to) the hub with limelight
public class TrackTargetWithLimelight extends CommandBase {

  private final Turret m_turret;
  // private final Shooter m_shooter;
  private final PIDController turretController = new PIDController(Constants.Turret.PID.kP, Constants.Turret.PID.kI, Constants.Turret.PID.kD);

  //final MedianFilter turretFilter = new MedianFilter(5);
  double offBy, power;

  //final SlewRateLimiter turretLimiter = new SlewRateLimiter(Constants.Turret.TURRET_LIMITER);
  
  
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
    m_turret.setLimelightLights(3);
    //offBy = 1;  //arbitrary starting point, used to establish direction of search

    System.out.println("initializing TrackTargetWithLimelight");
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
      offBy = m_turret.limelightGetTx();
      //offBy = turretFilter.calculate(offBy); // take median of last 5 readings

      power = turretController.calculate(offBy);
      power =  MathUtil.clamp(power, -Constants.Turret.CLAMP, Constants.Turret.CLAMP);
      //power = turretLimiter.calculate(power);
      m_turret.turretMotor.set(-power);
    } else {
      // System.out.println("Searching");
      //m_turret.turretMotor.set(Constants.Limelight.SEARCH_SPEED);
      //m_turret.turretMotor.set(-Constants.Limelight.SEARCH_SPEED*Math.abs(offBy)/offBy); //move in direction of last known target
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("endingTrackTargetWithLimelight");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
