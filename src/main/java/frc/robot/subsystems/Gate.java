// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gate extends SubsystemBase {

  private final TalonSRX m_motorGateLeft = new TalonSRX(Constants.CANIDs.CL_GATE_LEFT);
  private final TalonSRX m_motorGateRight= new TalonSRX(Constants.CANIDs.CL_GATE_RIGHT);

  
  /** Creates a new Gate. */
  public Gate() {

    m_motorGateRight.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setGate(double power) {
    m_motorGateLeft.set(TalonSRXControlMode.PercentOutput, power);
    m_motorGateRight.set(TalonSRXControlMode.PercentOutput, power);
  }

  //public double getGateSliderValue() {
   // return m_gateSpeed.getDouble(0);
  //}
  
}
