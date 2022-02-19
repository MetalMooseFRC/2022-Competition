// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectorControl extends CommandBase {
  
  private final DoubleSupplier m_armSpeedSupplier;
  private final DoubleSupplier m_gateSpeedSupplier;
  private final Collector m_collector;

  /** Creates a new CollectorControl. */
  public CollectorControl(DoubleSupplier armSpeedSupplier, DoubleSupplier gateSpeedSupplier, Collector collector) {
    m_armSpeedSupplier = armSpeedSupplier;
    m_gateSpeedSupplier = gateSpeedSupplier;
    
    m_collector = collector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.setArm(m_armSpeedSupplier.getAsDouble());
    m_collector.setGate(m_gateSpeedSupplier.getAsDouble());
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
