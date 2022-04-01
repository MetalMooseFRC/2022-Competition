// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;

// Load lifter until upper ball is detected by color sensor
public class IdleLifterLoader extends CommandBase {
  
  private Lifter m_lifter;
  private Loader m_loader;
  private final DoubleSupplier m_lifterSpeedSupplier;
  private String upperColor;
  private final DoubleSupplier m_loaderSpeedSupplier;
  /** Creates a new MoveLifterUp. */
  public IdleLifterLoader(Lifter lifter, DoubleSupplier lifterSpeedSupplier, Loader loader, DoubleSupplier loaderSpeedSupplier) {

    m_lifter = lifter;
    m_lifterSpeedSupplier = lifterSpeedSupplier;
    m_loader = loader;
    m_loaderSpeedSupplier = loaderSpeedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_lifter, m_loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    upperColor = m_lifter.getColorUpper();
    if (upperColor != "None"){
      m_lifter.m_motor.set(0.0);
      m_loader.m_motor.set(0.0);
    }
    else {
    m_lifter.m_motor.set(m_lifterSpeedSupplier.getAsDouble());
    m_loader.setMotorSpeed(m_loaderSpeedSupplier.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lifter.m_motor.set(0.0);
    m_loader.m_motor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
