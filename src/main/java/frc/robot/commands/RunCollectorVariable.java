// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import java.util.function.DoubleSupplier;


public class RunCollectorVariable extends CommandBase {

  private final Collector m_collector;
  private final DoubleSupplier m_collectorArmSupplier,m_collectorGateSupplier ;

  /** Creates a new RunCollectorVariable. */
  public RunCollectorVariable(DoubleSupplier armSupplier,DoubleSupplier gateSupplier, Collector collector) {
    m_collector = collector;
    m_collectorArmSupplier = armSupplier;
    m_collectorGateSupplier = gateSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.setArm(m_collectorArmSupplier.getAsDouble());
    m_collector.setGate(m_collectorGateSupplier.getAsDouble());

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
