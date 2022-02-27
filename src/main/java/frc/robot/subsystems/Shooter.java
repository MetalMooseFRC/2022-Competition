// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private ShuffleboardTab TestingTab = Shuffleboard.getTab("Testing");
  NetworkTableEntry m_ShooterSpeed = TestingTab.add("Shooter Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

  public final CANSparkMax m_motorLeft = new CANSparkMax(Constants.CANIDs.SH_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final CANSparkMax m_motorRight = new CANSparkMax(Constants.CANIDs.SH_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    m_motorLeft.setInverted(true);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  } 
  // return velocity, in RPM, of left wheel
  public double getLeftWheelSpeed() {
    return m_motorLeft.getEncoder().getVelocity();
  }

  public void setLeftWheelSpeed(double speed) {
    m_motorLeft.set(speed);
  }

  // return velocity, in RPM, of right wheel
  public double getRightWheelSpeed() {
    return m_motorRight.getEncoder().getVelocity();
  }

  public void setRightWheelSpeed(double speed) {
    m_motorRight.set(speed);
  }

  public double getSliderValue() {
    return m_ShooterSpeed.getDouble(0);
  }


}
