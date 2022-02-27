// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  public final CANSparkMax turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  /** Creates a new Turret. */
  public Turret() {
    turretMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    System.out.println(getTurretAngle());
    // This method will be called once per scheduler run
  }

  public double getDeadbandSpeed(double speed) {
    double adjustedSpeed = MathUtil.applyDeadband(speed, Constants.Preferences.DEADBAND);
    return(adjustedSpeed);
  }

  public double getTurretAngle() {
    double pos = (turretMotor.getEncoder().getPosition() % Constants.Turret.GEAR_RATIO * 9);
    if(pos >= 180) {
      pos -= 360;
    } else if(pos < -180) {
      pos += 360;
    }
    return pos;
  }

}
