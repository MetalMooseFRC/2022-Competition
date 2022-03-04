// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public final CANSparkMax turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  private ShuffleboardTab TestingTab = Shuffleboard.getTab("Testing");
  /** Creates a new Turret. */
  public Turret() {
    turretMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelight has target", limelightHasValidTarget());
    SmartDashboard.putNumber("Distance", getTurretDistance());

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

  public double limelightGetTx() {
      // System.out.println("Giving Tx");
      // System.out.println(m_limelightTable.getEntry("tx").getDouble(0.0));
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
    return m_limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

  //calculate the distance based on trig
  public double limelightGetDistance() {
    return (Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180);
  }

  public double getTurretDistance() {
    // double turretDistance;
    // //LIMELIGHT CENTERED BEHIND AT 60* AND CENTERED IN FRONT AT -120*
    // if (105.0 >= getTurretAngle() && getTurretAngle() >= 15.0){
    //   turretDistance = limelightGetDistance()-27;
    // } else if (-150<= getTurretAngle() && -60 >= getTurretAngle()){
    //   turretDistance = limelightGetDistance()+27;
    // } else {
    //   turretDistance = limelightGetDistance();
    // };
    // return (turretDistance);
    double offset = 27 * -Math.cos((getTurretAngle() + 31.989)*Math.PI/180);
    return limelightGetDistance() + offset;
  }
  // public double getShooterSpeed() {
  //   double shooterSpeed = getTurretDistance()*0.0005+0.4;
  //   return shooterSpeed;
  // }
  public void setLimelightLights(int setting) {
    m_limelightTable.getEntry("ledMode").setNumber(setting);
  }
}
