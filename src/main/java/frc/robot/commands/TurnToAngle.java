// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class TurnToAngle extends PIDCommand {

  //private final DriveTrain m_driveTrain;
  //private final double m_targetAngle;
  //private final Joystick m_joystick;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(double targetAngle, Drivetrain drivetrain, Joystick joystick) {

    super(
        // The controller that the command will use
        new PIDController(Constants.Drivetrain.P_TURN, Constants.Drivetrain.I_TURN, Constants.Drivetrain.D_TURN),
        // This should return the measurement
        drivetrain::getAngle,
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // This uses the output
        output -> {
          System.out.println(output);
          // Use the output here
          if(output > 0) {
            drivetrain.drive(-joystick.getY(), output + Constants.Drivetrain.FEED_TURN);
          } else if(output < 0) {
              drivetrain.drive(-joystick.getY(), output - Constants.Drivetrain.FEED_TURN);
          } else {
              drivetrain.drive(-joystick.getY(), 0);
          }
        } , drivetrain);  //require the driveTrain

    
    //m_driveTrain = driveTrain;
    //m_targetAngle = targetAngle; //?
    //m_joystick = joystick;  //?

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);


    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(Constants.Drivetrain.TOLERANCE_TURN, Constants.Drivetrain.TOLERANCE_TURN_RATE);
    
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }


}
