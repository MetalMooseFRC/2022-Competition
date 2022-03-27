// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hanger;
import static frc.robot.Constants.Hanger.*;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PullRobotUpWithPitch extends PIDCommand {

  private Hanger m_hanger;

  /** Creates a new PullRobotUpWithPitch. */
  public PullRobotUpWithPitch(Hanger hanger, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(HANGER_PULL_kP, HANGER_PULL_kI, HANGER_PULL_kD),
        // This should return the measurement
        hanger::getHangerPosition,
        // This should return the setpoint (can also be a constant)
        () -> BAR_POSITION,
        // This uses the output
        output -> {
          SmartDashboard.putNumber("output", output);
          // if(Math.abs(drivetrain.getPitchRate()) < MAX_PITCHRATE) {
          //   hanger.set(output);
          // } else {
          //   hanger.set(-0.07);
          // }
          hanger.set(-0.07+(output*(1-(Math.min(Math.abs(drivetrain.getPitchRate()), MAX_PITCHRATE)/(MAX_PITCHRATE)))));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hanger);

    // Configure additional PID options by calling `getController` here.

    m_hanger = hanger;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hanger.getHangerPosition() <= BAR_POSITION;
  }
}
