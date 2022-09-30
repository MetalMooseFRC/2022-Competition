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
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-twelve");
  private final MedianFilter m_filter = new MedianFilter(10);
  private String turretAimMode;
  public final CANSparkMax turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  // private double m_oldDistance, m_newDistance, m_distanceRate;
  
  /** Creates a new Turret. */
  public Turret() {
    turretMotor.getEncoder().setPosition(0);
    //turretMotor.setOpenLoopRampRate(Constants.Turret.RAMP_RATE);
    turretMotor.setOpenLoopRampRate(0.0);
  }

  @Override
  public void periodic() {
    // m_oldDistance = m_newDistance;
    // m_newDistance = getTurretDistance();

    // m_distanceRate = (m_oldDistance - m_newDistance);
    
    // SmartDashboard.putNumber("Distance(Imperial)", limelightGetDistance()/2.54/12);
    // SmartDashboard.putNumber("Distance(Metric)", limelightGetDistance());
    // SmartDashboard.putNumber("Turret Angel", getTurretAngle());
    // SmartDashboard.putNumber("Ty", limelightGetTy());
    // SmartDashboard.putNumber("distance Without Correction", (Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180));
    // SmartDashboard.putNumber("Added Distance", -15.8 + -0.289*getTurretAngle() + 1.53E-03*Math.pow(getTurretAngle(),2) + 9.05E-06*Math.pow(getTurretAngle(),3) + -2.46E-08*Math.pow(getTurretAngle(),4));

    // This method will be called once per scheduler run
  }

  public void setTurretMode(String mode) {
    turretAimMode = mode;
  }

  public String getTurretMode() {
    return turretAimMode;
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

  public double getLimelightOffset() {
    return 10*Math.sin(getTurretAngle()*Math.PI/180)+1.75;
  }

  public double getShotMultiplier() {
    return 0.09*Math.cos(getTurretAngle()*Math.PI/180);
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
    double turretAngle = getTurretAngle();
    return (((Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180))
    + (-15.8 + -0.289*turretAngle + 1.53E-03*Math.pow(turretAngle,2) + 9.05E-06*Math.pow(turretAngle,3) + -2.46E-08*Math.pow(turretAngle,4))
    );


  }

  //gets the distance of the turret based on the limelight
  // public double getTurretDistance() {
  //   double offset = 27 * -Math.cos((getTurretAngle() + 31.989)*Math.PI/180);
  //   return limelightGetDistance() + offset;
  // }
  public double getTurretDistance() {
    double turretAngle = getTurretAngle();
    // double offset = 27 * -Math.cos((turretAngle + 31.989)*Math.PI/180); // This is off
    double offset = (-24 * Math.cos((turretAngle-60)/56))-5;
    double limelightDistance = ((Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LIMELIGHT_HEIGHT)/Math.tan((limelightGetTy() + Constants.Limelight.LIMELIGHT_ANGLE)*Math.PI/180))
    + (-15.8 + -0.289*turretAngle + 1.53E-03*Math.pow(turretAngle,2) + 9.05E-06*Math.pow(turretAngle,3) + -2.46E-08*Math.pow(turretAngle,4));
    return limelightDistance + offset;

  }

  public double getRequiredVelocity() {
    double dis = getTurretDistance();
    double velocity;
    // velocity = 4194 + -11.1*dis + 0.0229*Math.pow(dis, 2);  
    //  velocity = 4528 + -12.7*dis + 0.0248*Math.pow(dis, 2);  
    //  velocity = 4554 + -12.6*dis + 0.0242*Math.pow(dis, 2);  //Bensalem Power (Consistently short at Lehigh)
    //  velocity = 4952 + -14.3*dis + 0.026*Math.pow(dis, 2);   
    //  velocity = 4828 + -12.9*dis + 0.0238*Math.pow(dis, 2);  
    //  velocity = 2917 + -2.14*dis + 0.00654*Math.pow(dis, 2); //Second to Last match day 1 Lehigh
    //  velocity = 4256 + -9.08*dis + 0.0145*Math.pow(dis, 2);     //Previous without first data point 4/8/22 8:25 (good until past 400cm)
    //  velocity = 4646 + -11.6*dis + 0.0185*Math.pow(dis, 2); // Adjusted previous for long shots 4/8/22 morning after practice field 1
    //  velocity = 5351 + -15.9*dis +0.025*Math.pow(dis, 2); //Adjusted previous long shots after practice field 2 4/8/22 10:01
    //  velocity = 4760 + -12.8*dis + 0.0212*Math.pow(dis, 2); // Functioning in sweet spot 4/8/22 after last match 5:20
    //  velocity = 4659 + -12.5*dis + 0.0214*Math.pow(dis, 2); //4/8/22 5:26

    //piecewise function for velocity if polynomial doesn't work ↓
    if (dis<300){
      velocity = 3150;
    } else if (dis<350) {
      velocity = dis + 2850;
    } else if ( dis<450){
      velocity = 2*dis + 2500;
    } else if (dis<500){
      velocity = 5*dis + 1150;
    } else {
      velocity = dis/2 + 3400;
    }
    
    //  velocity = 4252 + -10.6*dis + 0.0193*Math.pow(dis, 2);
    // if (m_distanceRate == 0) {
    // }
    // else if (m_distanceRate < 0) {
    //   velocity = velocity-100;
    // }
    // else {
    //   velocity = velocity+150;
    // }
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
