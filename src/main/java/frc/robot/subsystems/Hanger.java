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


public class Hanger extends SubsystemBase {

  DoubleSolenoid m_hangerSolenoid = new DoubleSolenoid(PNEUMATICS_HUB, PneumaticsModuleType.REVPH, HANGER_B, HANGER_A);
  private CANSparkMax m_motorClimberLeft = new CANSparkMax(HA_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_motorClimberRight = new CANSparkMax(HA_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder leftEncoder = m_motorClimberLeft.getEncoder();
  private RelativeEncoder rightEncoder = m_motorClimberRight.getEncoder();


  double encoderAverage;
 

  /** Creates a new Hanger. */
  public Hanger() {

    m_hangerSolenoid.set(kReverse);  //start with elevator pulled back
    m_motorClimberLeft.setInverted(false);
    m_motorClimberRight.setInverted(true);

    
    rightEncoder.setPosition(0.0); // starting position of motor, in revs.
    leftEncoder.setPosition(0.0); // starting position of motor, in revs.



  }


@Override
  public void periodic() {

    encoderAverage = (rightEncoder.getPosition() + leftEncoder.getPosition())/2;
    //puts elevator encoders on smartdashboard    
    SmartDashboard.putNumber("encoder average -1", encoderAverage);


    // This method will be called once per scheduler run
  }


  public double getHangerPosition(){
    
    SmartDashboard.putNumber("encoder average", encoderAverage);

    return encoderAverage;
  }


  public void toggleSolenoid() {
    System.out.print(m_hangerSolenoid.get());
    m_hangerSolenoid.toggle();
  }

  public void pushHangerOut() {
    System.out.print(m_hangerSolenoid.get());
    m_hangerSolenoid.set(kForward);;
  }

  public void pullHangerIn() {
    System.out.print(m_hangerSolenoid.get());
    m_hangerSolenoid.set(kReverse);
  }
  
  public void set(double speed) {
    m_motorClimberLeft.set(speed);
    m_motorClimberRight.set(speed);
  }


}
