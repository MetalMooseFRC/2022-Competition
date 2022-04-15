// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanger;
import static frc.robot.Constants.Hanger.*;

public class PullUpHalfway extends CommandBase {

  private Hanger m_hanger;
  private double m_currentPosition, m_startPosition, m_endPosition;
  
  /** Creates a new PullUpHalfway. */
  public PullUpHalfway(Hanger hanger) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hanger = hanger;
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPosition = m_hanger.getHangerPosition();
    m_endPosition = m_startPosition/2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPosition = m_hanger.getHangerPosition();
    if(m_currentPosition >= m_startPosition* 0.8) {                                    // hanger in first segment of climb
      m_hanger.set(5*MAX_PULL_POWER*((m_currentPosition/m_startPosition)-1) - 0.1);  
    } else if (m_currentPosition <= m_startPosition*0.7) {                             // hanger in last segment of climb
      m_hanger.set(5*MAX_PULL_POWER*(0.5-(m_currentPosition/m_startPosition)) - 0.1);
    } else {                                                                           // hanger in middle segment of climb
      m_hanger.set(MAX_PULL_POWER - 0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hanger.set(-0.07);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentPosition < m_endPosition;
  }
}
