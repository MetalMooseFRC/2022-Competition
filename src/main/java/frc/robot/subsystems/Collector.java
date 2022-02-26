// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  private ShuffleboardTab TestingTab = Shuffleboard.getTab("Testing");

  NetworkTableEntry m_ArmSpeed = TestingTab.add("Arm Speed", 0).getEntry();
  NetworkTableEntry m_gateSpeed = TestingTab.add("Gate Speed", 0).getEntry();

  private final CANSparkMax m_motorArm = new CANSparkMax(Constants.CANIDs.CL_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final TalonSRX m_motorGateLeft = new TalonSRX(Constants.CANIDs.CL_GATE_LEFT);
  private final TalonSRX m_motorGateRight= new TalonSRX(Constants.CANIDs.CL_GATE_RIGHT);
  DoubleSolenoid m_armSolenoid = new DoubleSolenoid(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, Constants.PneumaticsIDs.COLLECTOR_A, Constants.PneumaticsIDs.COLLECTOR_B);
  
  /** Creates a new Collector. */
  public Collector() {
    m_motorGateRight.setInverted(true);
    m_motorArm.setInverted(true); 
    m_armSolenoid.set(kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setArm(double speed) {
    m_motorArm.set(speed);
  }

  public double getArmSliderValue() {
    return m_ArmSpeed.getDouble(0);
  }

  public void setGate(double speed) {
    m_motorGateLeft.set(TalonSRXControlMode.PercentOutput, speed);
    m_motorGateRight.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public double getGateSliderValue() {
    return m_gateSpeed.getDouble(0);
  }

  public void toggleCollector() {
    m_armSolenoid.toggle();
    if(m_armSolenoid.get()==kReverse) {
      setArm(getArmSliderValue());
      setGate(getGateSliderValue());
    }
    else {
      m_motorArm.set(0);
    }
  }

}
