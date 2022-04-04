// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// Allows for driving of robot
public class DriveArcade extends CommandBase {

  private final Drivetrain m_drivetrain;
  
  private DoubleSupplier m_speedSupplier;
  private DoubleSupplier m_turnSupplier;

  double drive_fb, drive_lr;
  // fb for forward/backward   lr for left/right
 

  final SlewRateLimiter driveLimiter = new SlewRateLimiter(Constants.Drivetrain.DRIVE_LIMITER);
  final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.Drivetrain.TURN_LIMITER);


  /** Creates a new DriveArcade. */
  public DriveArcade(DoubleSupplier speedSupplier, DoubleSupplier turnSupplier, Drivetrain drivetrain) {
    m_speedSupplier = speedSupplier;
    m_turnSupplier = turnSupplier;
    
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_driveTrain.resetHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    drive_fb = Constants.Preferences.JOYSTICK_SPEED_FACTOR * m_speedSupplier.getAsDouble();
    if (Math.abs(m_turnSupplier.getAsDouble()) < Constants.Drivetrain.TURN_DEADBAND){
      drive_lr = 0;
    } else {
      drive_lr = Constants.Preferences.JOYSTICK_TURN_FACTOR * m_turnSupplier.getAsDouble();
    }

    
    drive_fb = driveLimiter.calculate(drive_fb);
    drive_lr = turnLimiter.calculate(drive_lr);

    m_drivetrain.drive(drive_fb, drive_lr);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
