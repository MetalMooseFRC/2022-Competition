// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax m_motorLeftFront = new CANSparkMax(Constants.CANIDs.DT_LEFT_FRONT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorLeftMiddle = new CANSparkMax(Constants.CANIDs.DT_LEFT_MIDDLE, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorLeftBack = new CANSparkMax(Constants.CANIDs.DT_LEFT_BACK, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup m_motorsLeft = new MotorControllerGroup(m_motorLeftFront, m_motorLeftMiddle, m_motorLeftBack);

  private final CANSparkMax m_motorRightFront = new CANSparkMax(Constants.CANIDs.DT_RIGHT_FRONT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorRightMiddle = new CANSparkMax(Constants.CANIDs.DT_RIGHT_MIDDLE, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorRightBack = new CANSparkMax(Constants.CANIDs.DT_RIGHT_BACK, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup m_motorsRight = new MotorControllerGroup(m_motorRightFront, m_motorRightMiddle, m_motorRightBack);
  
  public final DifferentialDrive diffDrive = new DifferentialDrive(m_motorsLeft, m_motorsRight);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // create deadband
    diffDrive.setDeadband(Constants.Preferences.DEADBAND);
    // invert right motors
    m_motorRightMiddle.setInverted(true);
    m_motorRightBack.setInverted(true);
    m_motorRightFront.setInverted(true);
  }

  @Override
  public void periodic() {

  }
}
