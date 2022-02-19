// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
  private ShuffleboardTab TestingTab = Shuffleboard.getTab("Testing");
  NetworkTableEntry m_LifterSpeed = TestingTab.add("Lifter Speed", 0).getEntry();


  public final CANSparkMax m_motor = new CANSparkMax(Constants.CANIDs.LF_MAIN, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new Lifter. */
  public Lifter() {  
    m_motor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
  // return velocity, in RPM, of left wheel
  public double getWheelSpeed() {
    return m_motor.getEncoder().getVelocity();
  }

  public void setWheelSpeed(double speed) {
    m_motor.set(speed);
  }

  public double getSliderValue() {
    return m_LifterSpeed.getDouble(0);
  }


}
