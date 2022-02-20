// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hanger extends SubsystemBase {
  DoubleSolenoid m_lifterSolenoid = new DoubleSolenoid(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, Constants.PneumaticsIDs.HANGER_1, Constants.PneumaticsIDs.HANGER_2);
  CANSparkMax m_motorClimberLeft = new CANSparkMax(Constants.CANIDs.HA_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax m_motorClimberRight = new CANSparkMax(Constants.CANIDs.HA_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  /** Creates a new Hanger. */
  public Hanger() {
    m_motorClimberLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Motor", m_motorClimberLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Motor", m_motorClimberRight.getOutputCurrent());
    // This method will be called once per scheduler run
  }

  public void toggleSolenoid() {
    m_lifterSolenoid.toggle();
  }

  public void controlHanger(double speed) {
    m_motorClimberLeft.set(speed);
    m_motorClimberRight.set(speed);
  }

}
