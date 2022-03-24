// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Gate.*;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;
import static frc.robot.Constants.Lifter.*;
import static frc.robot.Constants.Loader.*;
import static frc.robot.Constants.Gate.*;
import static frc.robot.Constants.Collector.*;

public class HuntForBalls extends CommandBase {

  private Collector m_collector;
  private Gate m_gate;
  private Lifter m_lifter;
  private Loader m_loader;

  /** Creates a new HuntForBalls. */
  public HuntForBalls(Collector collector, Gate gate, Lifter lifter, Loader loader) {

    m_collector = collector;
    m_gate = gate;
    m_lifter = lifter;
    m_loader = loader;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_collector, m_gate, m_lifter, m_loader);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.collect();                //arm forward and spinning
    m_gate.setGate(GATE_DEFAULT_SPEED);
    m_lifter.setMotorPower(LIFTER_DEFAULT_SPEED);
    m_loader.setMotorPower(LOADER_IDLE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setArm(0);
    m_collector.pullCollectorIn();
    m_gate.setGate(0);
    m_lifter.setMotorPower(0);
    m_loader.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
