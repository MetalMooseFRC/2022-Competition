// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Hanger.*;
import frc.robot.subsystems.Hanger;


/**
 * Moves the elevator to a certain position.
 */


public class xxMoveElevatorToHeight extends CommandBase {

  // The starting and ending position of the elevator (if it needs to go to a certain point)
  private double m_endPosition, m_currentPosition;
  private Hanger m_hanger;

  /** Creates a new MoveElevatorToHeight. */
  public xxMoveElevatorToHeight(double endPosition, Hanger hanger) {

    m_endPosition = endPosition;
    m_hanger = hanger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
      m_currentPosition = m_hanger.getHangerPosition();

      // SmartDashboard.putNumber("current",m_currentPosition );
      // Where do we want to go (if positive, go up; if negative, go down)
      // Signum returns either +1 or -1 (that is what we want the speed to be
      double speed = Math.signum(m_endPosition - m_currentPosition);

      m_hanger.set(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_hanger.set(0);
    

    // SmartDashboard.putNumber("lastCurrent", m_currentPosition);
    // SmartDashboard.putNumber("endPos", m_endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_currentPosition - m_endPosition) < HANGER_POSITION_TOLERANCE;
  }
}

