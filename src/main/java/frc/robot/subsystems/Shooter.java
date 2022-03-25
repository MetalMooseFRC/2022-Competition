// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class Shooter extends SubsystemBase {

  public final CANSparkMax m_motorLeft = new CANSparkMax(Constants.CANIDs.SH_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final CANSparkMax m_motorRight = new CANSparkMax(Constants.CANIDs.SH_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

  // BangBangController leftController = new BangBangController();
  // BangBangController rightController = new BangBangController();
  PIDController leftController = new PIDController(0.5, 0.0, 0.0);


  /** Creates a new Shooter. */
  public Shooter() {
    m_motorLeft.setInverted(true);
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  } 
  
  // public double getSliderValue() {
  // }

  //sets shooter speed
  public void setShooterSpeed(double velocity) {  //speed in RPM

    m_motorLeft.set(velocity/5676);
    m_motorRight.set(velocity/5676);
    // m_motorLeft.set(leftController.calculate(m_motorLeft.getEncoder().getVelocity(), velocity));
    // m_motorRight.set(rightController.calculate(m_motorLeft.getEncoder().getVelocity(), velocity));

  }

  public void setShooterPower(double power) {
    m_motorLeft.set(power);
    m_motorRight.set(power);
  }
  
  // return velocity, in RPM, of left wheel
  public double getLeftWheelSpeed() {
    return m_motorLeft.getEncoder().getVelocity();
  }
  // return velocity, in RPM, of right wheel
  public double getRightWheelSpeed() {
    return m_motorRight.getEncoder().getVelocity();
  }
  
  //sets left wheel speed
  //public void setLeftWheelSpeed(double speed) {
    //m_motorLeft.set(speed);
  //}
  //sets right wheel speed
  //public void setRightWheelSpeed(double speed) {
    //m_motorRight.set(speed);
  //}
  
  
  // sets both right and left wheel speeds to slider value
  // public void runShooter() {
  //   setRightWheelSpeed(getSliderValue());
  //   setLeftWheelSpeed(getSliderValue());
  // }

  //gets slider value for shooter speed
  //public double getSliderValue() {
    //return shooterSpeedEntry.getDouble(0);
  //}


}
