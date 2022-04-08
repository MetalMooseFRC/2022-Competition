// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Turn to closest ball of our color and turns to it
public class TurnToHub extends PIDCommand {
  /** Creates a new TurnToBall. */
  public TurnToHub(DoubleSupplier driveSpeed, Drivetrain drivetrain, Turret turret) {
    super(
        // The controller that the command will use
        new PIDController(Constants.Drivetrain.P_TURN, Constants.Drivetrain.I_TURN, Constants.Drivetrain.D_TURN),
        // This should return the measurement
        turret::limelightGetTx,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          if(output > 0) {
            // SmartDashboard.putNumber("TurnToBall Output", output);
            drivetrain.drive(driveSpeed.getAsDouble(), -output - Constants.Drivetrain.FEED_TURN);
          } else if(output < 0) {
              drivetrain.drive(driveSpeed.getAsDouble(), -output + Constants.Drivetrain.FEED_TURN);
          } else {
              drivetrain.drive(driveSpeed.getAsDouble(), 0);
          }
        },
         drivetrain);  //require the driveTrain

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);


    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(Constants.Drivetrain.TOLERANCE_TURN, Constants.Drivetrain.TOLERANCE_TURN_RATE);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
