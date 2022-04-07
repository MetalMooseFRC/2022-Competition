// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class Lifter extends SubsystemBase {

  public final CANSparkMax m_motor = new CANSparkMax(Constants.CANIDs.LF_MAIN, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final I2C.Port roboRioI2CPort = I2C.Port.kOnboard;
  private final I2C.Port navxI2CPort = I2C.Port.kMXP;
  
  private final ColorSensorV3 m_upperColorSensor = new ColorSensorV3(navxI2CPort);
  private final ColorSensorV3 m_lowerColorSensor = new ColorSensorV3(roboRioI2CPort);
  
  private final ColorMatch m_colorMatcher = new ColorMatch();

  //Default Blue and Red
  // private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  // private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

  private final Color kRedTarget = new Color(0.612, 0.196, 0.187);
  private final Color kBlueTarget = new Color(0.141, 0.188, 0.427);


  /** Creates a new Lifter. */
  public Lifter() {  
    m_motor.setInverted(true);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Lifter Motor Current", m_motor.getOutputCurrent());
    // This method will be called once per scheduler run
  } 
  // return velocity, in RPM, of left wheel
  
  //gets the wheel speed
  public double getMotorSpeed() {
    return m_motor.getEncoder().getVelocity();
  }

  //sets the wheel speed
  public void setMotorPower(double speed) {
    m_motor.set(speed);
  }

  public String getColorUpper() {
    m_colorMatcher.setConfidenceThreshold(Constants.Lifter.COLOR_CONFIDENCE);
    Color detectedColor = m_upperColorSensor.getColor();
    String upperCargoColor;
    // System.out.println("Detected Upper Color = "+ m_upperColorSensor.getColor().toString());
    try {
    if (m_colorMatcher.matchColor(detectedColor).color == kBlueTarget) {
      upperCargoColor = "Blue";
    } else if (m_colorMatcher.matchColor(detectedColor).color == kRedTarget) {
      upperCargoColor = "Red";
    } else {
      upperCargoColor = "Unknown";
    }
    return upperCargoColor;
  } 
  catch (Exception e) {
    upperCargoColor = "None";
    return upperCargoColor;
  }
  }

  public String getColorLower() {
    m_colorMatcher.setConfidenceThreshold(Constants.Lifter.COLOR_CONFIDENCE);
    Color detectedColor = m_lowerColorSensor.getColor();
    String lowerCargoColor;
    // System.out.println("Detected Lower Color = "+ m_lowerColorSensor.getColor().toString());
    try{
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
    if (match.color == kBlueTarget) {
      lowerCargoColor = "Blue";
    } else if (match.color == kRedTarget) {
      lowerCargoColor = "Red";
    } else {
      lowerCargoColor = "Unknown";
    }
    return lowerCargoColor;
    }
    catch (Exception e){
      lowerCargoColor = "None";
      return lowerCargoColor;
    }
  }
  //gets a slider value for lifter
  //public double getSliderValue() {
    //return m_LifterSpeed.getDouble(0);
  //}

}