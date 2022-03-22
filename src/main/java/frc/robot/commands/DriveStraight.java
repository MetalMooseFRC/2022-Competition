// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends PIDCommand {

  private final Drivetrain m_drivetrain;
  private final double m_distance;
  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain drivetrain, double distance, double speed) {  //distance in feet
    super(

        // The controller that the command will use
        new PIDController(Constants.Drivetrain.P_TURN, Constants.Drivetrain.I_TURN, Constants.Drivetrain.D_TURN),
        // This should return the measurement
        drivetrain::getAngle,
        // This should return the setpoint (can also be a constant)
        0.0,
        // This uses the output
        output -> {

         
          
          // Use the output here
          if(output > 0) {
              drivetrain.drive(speed, output + Constants.Drivetrain.FEED_TURN);
          } else if(output < 0) {
              drivetrain.drive(speed, output - Constants.Drivetrain.FEED_TURN);
          } else {
              drivetrain.drive(speed, 0);
          }
        } , drivetrain);  //require the driveTrain

        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(Constants.Drivetrain.TOLERANCE_TURN, Constants.Drivetrain.TOLERANCE_TURN_RATE);

    m_distance = distance;
    m_drivetrain = drivetrain;
    m_drivetrain.resetEncoders();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getEncoderPosition() > m_distance* 7.44 /(Math.PI*4.1875/12) ;
  }
}
