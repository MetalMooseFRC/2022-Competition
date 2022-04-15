// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Turn entire robot to exact angle
public class TurnToAngleWithPID extends PIDCommand {

  //private final DriveTrain m_driveTrain;
  //private final double m_targetAngle;
  //private final Joystick m_joystick;
  
  /** Creates a new TurnToAngle. */
  public TurnToAngleWithPID(double targetAngle, Drivetrain drivetrain, double p, double i, double d) {
    
    super(
      // The controller that the command will use
      new PIDController(p, i, d),
      // This should return the measurement
      drivetrain::getAngle,
      // This should return the setpoint (can also be a constant)
      targetAngle,
      // This uses the output
      output -> {
        // Use the output here
        output = MathUtil.clamp(output, -0.43, 0.43);
        if(output > 0) {
          drivetrain.drive(0, output + Constants.Drivetrain.FEED_TURN);
        } else if(output < 0) {
          drivetrain.drive(0, output - Constants.Drivetrain.FEED_TURN);
        } else {
          drivetrain.drive(0, 0);
        }
      } , drivetrain);  //require the driveTrain
      
      
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
      
      
      // Configure additional PID options by calling `getController` here.
      getController().enableContinuousInput(-180, 180);
      getController().setTolerance(Constants.Drivetrain.TOLERANCE_TURN/*, Constants.Drivetrain.TOLERANCE_TURN_RATE*/);
      
      
    }
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint();
    }
    
    
  }
  