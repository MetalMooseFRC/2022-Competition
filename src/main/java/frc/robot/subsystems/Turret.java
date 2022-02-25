// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final CANSparkMax m_motor = new CANSparkMax(Constants.CANIDs.TURRET_AIM, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new Turret. */
  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setTurretSpeed(double speed) {
    m_motor.set(speed);
  }
  
  public double getTurretSpeed(){
    return m_motor.getEncoder().getVelocity();
  }

    //get the x error between the crosshair and target
  public double limelightGetTx() {
      return m_limelightTable.getEntry("tx").getDouble(0.0);
  }

  //get the y error between the crosshair and target
  public double limelightGetTy() {
    return m_limelightTable.getEntry("ty").getDouble(0.0); 
  }

  //get the area of the target
  public double limelightGetTa() {
    return m_limelightTable.getEntry("ta").getDouble(0.0);
}


  //does the limelight see a viable target
  public boolean limelightHasValidTarget() {
    return m_limelightTable.getEntry("tv").getDouble(0.0) == 1;
  }

  //calculate the distance based on trig
  public double limelightGetDistance() {
      return (Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180);
  }
}
