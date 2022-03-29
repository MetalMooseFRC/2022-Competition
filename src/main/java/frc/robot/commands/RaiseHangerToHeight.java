// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Hanger;
import static frc.robot.Constants.Hanger.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Raise hanger to specific height (RPM)
public class RaiseHangerToHeight extends PIDCommand {

  private Hanger m_hanger;
  private double m_setpoint;
  private double m_currentPosition;

  /** Creates a new RaiseHangerToHeight. */
  public RaiseHangerToHeight(double setpoint, Hanger hanger) {
    super(
        // The controller that the command will use
        new PIDController(HANGER_kP, HANGER_kI, HANGER_kD),   // PID was tuned at 12.2 V
        // This should return the measurement
        hanger::getHangerPosition,
        // This should return the setpoint (can also be a constant)
        setpoint-HANGER_POSITION_TOLERANCE,    // a little buffer for overshoot
        // This uses the output
        output -> {
          //if (output > 0) {
            hanger.set(output + 0.02);
            // SmartDashboard.putNumber("out", output);
          //}


        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hanger);

    // Configure additional PID options by calling `getController` here.
    m_hanger = hanger;
    m_setpoint = setpoint;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_currentPosition = m_hanger.getHangerPosition();
    if (m_currentPosition > m_setpoint) {
      m_controller.close();
      m_hanger.set(0);
      return true;
    }
    return false ;    
  }

  
  
}
