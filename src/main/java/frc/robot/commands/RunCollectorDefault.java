// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.Constants;


public class RunCollectorDefault extends CommandBase {
  private final Collector m_collector;
  /** Creates a new RunCollectorDefault. */
  public RunCollectorDefault(Collector collector) {
    m_collector = collector;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_collector.setArm(Constants.Collector.ARM_SPEED);
    m_collector.setGate(Constants.Collector.GATE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}