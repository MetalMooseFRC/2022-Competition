// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Gate.*;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Lifter;

// Put down collector and turn on gate. If wrong color ball is detected in system, reverse collector
public class HuntForBalls extends CommandBase {

  private Collector m_collector;
  private Gate m_gate;
  private Lifter m_lifter;

  /** Creates a new HuntForBalls. */
  public HuntForBalls(Collector collector, Gate gate, Lifter lifter) {

    m_collector = collector;
    m_gate = gate;
    m_lifter = lifter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_collector, m_gate);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.collect();//arm forward and spinning
    if ((DriverStation.getAlliance().toString() == m_lifter.getColorLower()) || (m_lifter.getColorLower() == "None")) { // inverts gate direction if it sees enemy ball
      m_gate.setGate(GATE_DEFAULT_SPEED);
    } else {
      m_gate.setGate(-GATE_DEFAULT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setArm(0);
    m_collector.pullCollectorIn();
    m_gate.setGate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
