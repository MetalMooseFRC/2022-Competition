// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifter;

public class LifterControl extends CommandBase {

  private final Lifter m_lifter;
  private final DoubleSupplier m_speedSupplier;
  private final PIDController m_pidController = new PIDController(Constants.Lifter.KP, Constants.Lifter.KI, Constants.Lifter.KD);
  
  /** Creates a new LifterControl. */
  public LifterControl(DoubleSupplier speedSupplier, Lifter lifter) {
    m_speedSupplier = speedSupplier;
    
    m_lifter = lifter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lifter);

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
    // double m_motorPID = m_pidController.calculate(m_lifter.getWheelSpeed());
  
    // Set lifter wheels to calculated speeds
    m_lifter.setWheelSpeed(m_speedSupplier.getAsDouble());
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
