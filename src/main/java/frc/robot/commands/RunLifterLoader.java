// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Lifter;

public class RunLifterLoader extends CommandBase {
  /** Creates a new StopLifterLoader. */
  private final Lifter m_lifter;
  private final Loader m_loader;
  private final double m_lifterSpeed;
  private final double m_loaderSpeed;

  public RunLifterLoader(Lifter lifter, double lifterSpeed, Loader loader, double loaderSpeed) {
    m_lifter = lifter;
    m_lifterSpeed = lifterSpeed;
    m_loader = loader;
    m_loaderSpeed = loaderSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_lifter, m_loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lifter.m_motor.set(m_lifterSpeed);
    m_loader.m_motor.set(m_loaderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_loader.m_motor.set(0.0);
    m_lifter.m_motor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
