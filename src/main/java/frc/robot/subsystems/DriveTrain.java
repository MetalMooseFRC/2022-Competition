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

// NavX
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.I2C.Port;

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

  private AHRS m_navx = new AHRS(Port.kMXP);

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_navx.getRotation2d(), new Pose2d(0.0, 0.0, new Rotation2d()));

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
    
    m_odometry.update(m_navx.getRotation2d(),
    m_motorLeftMiddle.getEncoder().getPosition(),
    m_motorRightMiddle.getEncoder().getPosition());
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    
    // System.out.println(m_odometry.getPoseMeters());
    
  }

  public Pose2d getPosition(){
    return m_odometry.getPoseMeters();
  }
}
