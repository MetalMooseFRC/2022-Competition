// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {

  public final CANSparkMax m_motor = new CANSparkMax(Constants.CANIDs.LOADER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  BangBangController controller = new BangBangController();

  /** Creates a new Loader. */
  public Loader() {
    m_motor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   //sets the wheel speed
   public void setMotorSpeed(double rpm) {
    // m_motor.set(controller.calculate(m_motor.getEncoder().getVelocity(), rpm));
    m_motor.set(0.3);
  }


}
