// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterControl extends CommandBase {

  private final Shooter m_shooter;
  private final DoubleSupplier m_speedSupplier;
  private final PIDController m_pidController = new PIDController(Constants.Shooter.KP, Constants.Shooter.KI, Constants.Shooter.KD);
  
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
    // Set the PID loop to aim to converge on the speedSupplier value
    m_pidController.setSetpoint(m_speedSupplier.getAsDouble());
    // PID loop calculates target speed for wheels.
    System.out.println("Speedsupplier: " + m_speedSupplier.getAsDouble());
    double m_leftPID = m_pidController.calculate(m_shooter.getLeftWheelSpeed() / 5676);
    if(m_leftPID < 0) {m_leftPID = 0;}
    double m_rightPID = m_pidController.calculate(m_shooter.getRightWheelSpeed() / 5676);
    if (m_rightPID < 0) {m_rightPID = 0;}


    System.out.println("LeftPID: " + m_leftPID);
    System.out.println("RightPID: " + m_rightPID);


    // Set shooter wheels to calculated speeds
    m_shooter.setLeftWheelSpeed(m_speedSupplier.getAsDouble());

    m_shooter.setRightWheelSpeed(m_speedSupplier.getAsDouble());
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
