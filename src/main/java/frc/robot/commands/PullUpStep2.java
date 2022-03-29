// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Hanger;
import static frc.robot.Constants.Hanger.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PullUpStep2 extends PIDCommand {

  private Hanger m_hanger;
  private double m_currentPosition;
  

  /** Creates a new RaiseHangerToHeight. */
  public PullUpStep2(Hanger hanger) {
    super(
        // The controller that the command will use
        new PIDController(HANGER_PULL_kP, HANGER_PULL_kI, HANGER_PULL_kD),
        // This should return the measurement
        hanger::getHangerPosition,
        // This should return the setpoint (can also be a constant)
        STEP_2 ,
        // This uses the output
        output -> {
      
          hanger.set(output + 0.04);
        
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hanger);

    // Configure additional PID options by calling `getController` here.
    m_hanger = hanger;
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_currentPosition = m_hanger.getHangerPosition();
    
    if(m_currentPosition > STEP_2){
      m_controller.close();
      m_hanger.set(0.06);   //hold
      return true;
    }
      
          
    return false;//m_currentPosition > MAX_HEIGHT - HANGER_POSITION_TOLERANCE;
  }

  
  
}
