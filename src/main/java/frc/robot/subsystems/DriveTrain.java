// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.CANIDs.*;


// NavX
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax m_motorLeftFront = new CANSparkMax(DT_LEFT_FRONT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorLeftMiddle = new CANSparkMax(DT_LEFT_MIDDLE, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorLeftBack = new CANSparkMax(DT_LEFT_BACK, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup m_motorsLeft = new MotorControllerGroup(m_motorLeftFront, m_motorLeftMiddle, m_motorLeftBack);

  private final CANSparkMax m_motorRightFront = new CANSparkMax(DT_RIGHT_FRONT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorRightMiddle = new CANSparkMax(DT_RIGHT_MIDDLE, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorRightBack = new CANSparkMax(DT_RIGHT_BACK, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup m_motorsRight = new MotorControllerGroup(m_motorRightFront, m_motorRightMiddle, m_motorRightBack);
  
  private final DifferentialDrive diffDrive = new DifferentialDrive(m_motorsLeft, m_motorsRight);


  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // private final String allianceColor = DriverStation.getAlliance().toString();

  //DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    // m_navx.getRotation2d(), new Pose2d(0.0, 0.0, new Rotation2d()));
    
    // navX gyro
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final RelativeEncoder leftEncoder = m_motorLeftMiddle.getEncoder();
    public final RelativeEncoder rightEncoder = m_motorRightMiddle.getEncoder();
    private double m_pitchRate
    // , m_oldPitch, m_newPitch, m_oldCount, m_newCount
     ;
    
    
    
    
    
    //Set up shuffleboard
    private ShuffleboardTab m_tab = Shuffleboard.getTab("Development");
    // private ComplexWidget z_tab = Shuffleboard.getTab("Development")
    // .add("NavX", navx);
    
    
    
    // put the navx on the shuffleboard
    private NetworkTableEntry m_navxAngleEntry = m_tab
    .add("NavX Angle", 0)
    //.withWidget(BuiltInWidgets.kGyro)
    .getEntry();
    private NetworkTableEntry m_navxYawEntry = m_tab
    .add("NavX Yaw", 0)
    //.withWidget(BuiltInWidgets.kGyro)
    .getEntry();
    
    
    /** Creates a new Drivetrain. */
    public Drivetrain() {
      if (DriverStation.getAlliance().toString() == "Blue"){
      m_limelightTable.getEntry("pipeline").forceSetValue(1);
    } else {
      m_limelightTable.getEntry("pipeline").forceSetValue(0);
    }
      this.resetYaw();  //reset navx when m_driveTrain is constructed @ start
      
    // create deadband
    diffDrive.setDeadband(DRIVE_DEADBAND);
    // invert right motors
    m_motorRightMiddle.setInverted(true);
    m_motorRightBack.setInverted(true);
    m_motorRightFront.setInverted(true);

    
  }

  @Override
  public void periodic() {

    m_navxAngleEntry.setDouble(navx.getAngle());
    m_navxYawEntry.setDouble(navx.getYaw());

    SmartDashboard.putNumber("Navx Yaw", navx.getYaw());

    // m_oldPitch = m_newPitch;
    // m_newPitch = navx.getPitch();

    // m_oldCount = m_newCount;
    // m_newCount = navx.getUpdateCount();

    // m_pitchRate = (m_oldPitch - m_newPitch)/(m_oldCount - m_newCount) *  navx.getActualUpdateRate();

    // SmartDashboard.putNumber("Vel X", navx.getVelocityX());
    // SmartDashboard.putNumber("Vel Y", navx.getVelocityY());
    // SmartDashboard.putNumber("count", navx.getUpdateCount());
    // SmartDashboard.putNumber("last timestamp", navx.getLastSensorTimestamp());
    // SmartDashboard.putNumber("Req Rate", navx.getRequestedUpdateRate());
    // SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
    // SmartDashboard.putNumber("pitchRate", m_pitchRate);

    

    // boolean motionDetected = navx.isMoving();
    // SmartDashboard.putBoolean("MotionDetected", motionDetected);
    // boolean isConnected = navx.isConnected();
    // SmartDashboard.putBoolean("Is Connected", isConnected);
    // SmartDashboard.putNumber("rotations",rightEncoder.getPosition() );
    

    //m_odometry.update(m_navx.getRotation2d(),
    //m_motorLeftMiddle.getEncoder().getPosition();
    //m_motorRightMiddle.getEncoder().getPosition();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    
    // System.out.println(m_odometry.getPoseMeters());
    
  }

  public void drive(double speed, double turn){
    diffDrive.arcadeDrive(speed, turn, true);
  }

  public void resetEncoders(){
      rightEncoder.setPosition(0.0); // starting position of motor, in revs.
      leftEncoder.setPosition(0.0); // starting position of motor, in revs.
  }

  public double getEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double limelightTwelveGetTx() {
    return -m_limelightTable.getEntry("tx").getDouble(0.0);
}

  // **********  Gyro Methods  ********** //

  // get angle ranging from -180 to 180
  public double getYaw() {
    return navx.getYaw();
  }
  
  // get accumulated angle 
  public double getAngle() {
    return navx.getAngle();
  }


  // zero heading
  public void resetYaw() {
    navx.reset();
    
  } 

  //get PitchRate
  public double getPitchRate() {
    return m_pitchRate;
  }

  //public Pose2d getPosition(){
  //  return m_odometry.getPoseMeters();
 // }
}
