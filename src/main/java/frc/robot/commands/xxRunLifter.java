// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;

public class xxRunLifter extends CommandBase {
  
  private Lifter m_lifter;

  private final DoubleSupplier m_lifterSpeedSupplier;

  /** Creates a new MoveLifterUp. */
  public xxRunLifter(Lifter lifter, DoubleSupplier lifterSpeedSupplier) {

    m_lifter = lifter;
    m_lifterSpeedSupplier = lifterSpeedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(lifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lifter.m_motor.set(m_lifterSpeedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lifter.m_motor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
