// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Hanger.*;
import static frc.robot.Constants.CANIDs.*;
import static frc.robot.Constants.PneumaticsIDs.*;


import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class xxHanger extends SubsystemBase {

  DoubleSolenoid m_hangerSolenoid = new DoubleSolenoid(PNEUMATICS_HUB, PneumaticsModuleType.REVPH, HANGER_B, HANGER_A);
  public CANSparkMax m_motorClimberLeft = new CANSparkMax(HA_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public CANSparkMax m_motorClimberRight = new CANSparkMax(HA_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public double hangerPosition;
  public RelativeEncoder leftEncoder = m_motorClimberLeft.getEncoder();
  public RelativeEncoder rightEncoder = m_motorClimberRight.getEncoder();


  /** Creates a new Hanger. */
  public xxHanger() {

    m_hangerSolenoid.set(kReverse);  //start with elevator pulled back
    m_motorClimberLeft.setInverted(true);
    
    rightEncoder.setPosition(0.0); // starting position of motor, in revs.
    leftEncoder.setPosition(0.0); // starting position of motor, in revs.


  }


@Override
  public void periodic() {

    //puts elevator encoders on smartdashboard    
    SmartDashboard.putNumber("encoder average", (rightEncoder.getPosition() + leftEncoder.getPosition())/2);
    

    // This method will be called once per scheduler run
  }

    /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value Value to clip.
   * @param deadband Range around zero.
   * @return The value after the deadband is applied.
   */
  private static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }


  public double getHangerPosition(){
    return rightEncoder.getPosition();
  }


  public void toggleSolenoid() {
    m_hangerSolenoid.toggle();
  }

  //lets your control the hanger with Joystick input
  public void controlHanger(double speed) {

    double adjustedSpeed = applyDeadband(speed, .02);
    m_motorClimberLeft.set(adjustedSpeed);
    m_motorClimberRight.set(adjustedSpeed);
    SmartDashboard.putNumber("Hanger Encoder",rightEncoder.getPosition() );

  }

  /**
     * Set throttled speed of the elevator motors
     *
     * @param speed Speed to which to set the motor to (this will be throttled).
     */
    public void setThrottledSpeed(double speed) {
      // The x of the function
      double x = leftEncoder.getPosition();

      // The actual speed (multiplying the y of the function with the input speed)
      double throttledSpeed = speed * getThrottledSpeed(x, Math.signum(speed));

      controlHanger(throttledSpeed);
  }

  /**
   * Get the y value at point x of the graph of the throttle function.
   *
   * @param x         The x value of the graph.
   * @param direction The direction that the elevator is going (the functions are different when going up and down)
   * @return The y value of the graph.
   */
  private double getThrottledSpeed(double x, double direction) {
      // Restrict the x values (from 0 to maximum), to remove any unpredictable behavior
      if (x < 0) x = 0;
      else if (x > MAX_HEIGHT) x = MAX_HEIGHT;

      // Two sets of coefficients for the elevator going up and down
      final double[] upCoefficients = new double[]{-0.0000001233779715, 0.000024675594291, -0.0017622246431853, 0.052844492636667, 0.45};
      final double[] downCoefficients = new double[]{-0.0000000957264957, 0.0000191452991454, -0.0016365811965889, 0.0679316239315355, -0.1};

      // This variable will just take one of the previous two arrays as a reference
      double[] coefficients;

      // Pick the right equation from the direction that the elevator wants to go
      // Also, if going up, we go full speed (similar to going down) - that is why we set x to half of the polynomial max
      if (direction == 1) {
          coefficients = upCoefficients;
          if (x < MAX_HEIGHT / 2) x = MAX_HEIGHT / 2;
      } else {
          coefficients = downCoefficients;
          if (x > MAX_HEIGHT / 2) x = MAX_HEIGHT / 2;
      }

      // Calculate the y value of the graph
      double value = 0;
      for (double coefficient : coefficients) value = value * x + coefficient;

      // The maximum motor speed is 1 (or -1, for that matter)
      // Since one of the functions goes slightly above 1
      if (value > 1) return 1;
      else if (value < -1) return -1;
      else return value;
  }

}
