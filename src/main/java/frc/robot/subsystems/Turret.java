// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final MedianFilter m_filter = new MedianFilter(10);
  public final CANSparkMax turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  
  /** Creates a new Turret. */
  public Turret() {
    turretMotor.getEncoder().setPosition(0);
    //turretMotor.setOpenLoopRampRate(Constants.Turret.RAMP_RATE);
    turretMotor.setOpenLoopRampRate(0.0);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Distance(Imperial)", limelightGetDistance()/2.54/12);
    SmartDashboard.putNumber("Distance(Metric)", limelightGetDistance());
    SmartDashboard.putNumber("Turret Angel", getTurretAngle());
    SmartDashboard.putNumber("Ty", limelightGetTy());
    SmartDashboard.putNumber("distance Without Correction", (Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180));
    SmartDashboard.putNumber("Added Distance", -15.8 + -0.289*getTurretAngle() + 1.53E-03*Math.pow(getTurretAngle(),2) + 9.05E-06*Math.pow(getTurretAngle(),3) + -2.46E-08*Math.pow(getTurretAngle(),4));

    // This method will be called once per scheduler run
  }

  public double getDeadbandSpeed(double speed) {
    double adjustedSpeed = MathUtil.applyDeadband(speed, Constants.Preferences.DEADBAND);
    return(adjustedSpeed);
    //this returns a speed with a deadband applied from an inputted speed
  }

  public double getTurretAngle() {
    //gets the angle of the turret from 180 to -180
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
    double ty = m_limelightTable.getEntry("ty").getDouble(0.0); 
    double tyAdjusted = ty + ((m_limelightTable.getEntry("tshort").getDouble(0.0)*(50/320))/2);
    return m_filter.calculate(tyAdjusted);
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
    return (((Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180))
    //+ (-16.1 -.65*getTurretAngle() + 6E-03*Math.pow(getTurretAngle(),2) + 2.38E-05*Math.pow(getTurretAngle(),3))
    + (-15.8 + -0.289*getTurretAngle() + 1.53E-03*Math.pow(getTurretAngle(),2) + 9.05E-06*Math.pow(getTurretAngle(),3) + -2.46E-08*Math.pow(getTurretAngle(),4))
    );


  }

  //gets the distance of the turret based on the limelight
  public double getTurretDistance() {
    double offset = 27 * -Math.cos((getTurretAngle() + 31.989)*Math.PI/180);
    return limelightGetDistance() + offset;
  }

  public double getRequiredVelocity() {
    double dis = getTurretDistance();
    double velocity = 4033 + -10*dis + 0.0214*Math.pow(dis,2);
    return velocity;
  }

  // public double getShooterSpeed() {
  //   double shooterSpeed = getTurretDistance()*0.0005+0.4;
  //   return shooterSpeed;
  // }

  //sets the lights on the limelight to a setting(on/off)
  public void setLimelightLights(int setting) {
    m_limelightTable.getEntry("ledMode").setNumber(setting);
  }
}
