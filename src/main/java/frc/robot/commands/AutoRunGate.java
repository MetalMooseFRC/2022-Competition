// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Lifter;

// Run gate for autonomous
public class AutoRunGate extends CommandBase {
  /** Creates a new AutoRunGate. */
  private final Lifter m_lifter;
  private final Gate m_gate;
  private final Collector m_collector;
  public AutoRunGate(Lifter lifter, Gate gate, Collector collector) {
    m_lifter = lifter;
    m_collector = collector;
    m_gate = gate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_gate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_lifter.getColorUpper() != "None" && m_lifter.getColorLower() != "None") || (m_collector.m_armSolenoid.get().toString() == "kReverse")) {

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
