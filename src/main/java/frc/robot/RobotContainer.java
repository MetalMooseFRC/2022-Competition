// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Lifter.*;
import static frc.robot.Constants.Gate.*;
import static frc.robot.Constants.Collector.*;
import static frc.robot.Constants.Hanger.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // ************  OI Controller  ***************
  public static final Joystick driverStick = new Joystick(Constants.DSPorts.DRIVER_STICK_PORT);
  JoystickButton huntForBallsButton;
  JoystickButton hangerPneumaticsReverseButton, hangerPneumaticsForwardButton; //armToggleButton, armCollectButton;

  public static final Joystick operatorStick = new Joystick(Constants.DSPorts.OPERATOR_STICK_PORT);
  JoystickButton shootCargoButton, turnTurretToZeroButton, shootingSpeedUpButton, shootingSpeedDownButton, turretAimToggleButton,
   invertCollectorButton, runShooterToggleButton, shootSliderButton, runLifterReverseButton, burpBallButton, runShooterAtSlider;
  POVButton incrementHangerUpButton, incrementHangerDownButton, hangerToMaxHeightButton, cancelHangerUpButton, turnTurretTo90Button, turnTurretToN90Button;

  // ************  Subsystems  **************
  private Drivetrain m_drivetrain = new Drivetrain();
  private Shooter m_shooter = new Shooter();
  private Lifter m_lifter = new Lifter();
  private Collector m_collector = new Collector();
  private Hanger m_hanger = new Hanger();
  private Turret m_turret = new Turret();
  private Loader m_loader = new Loader();
  private Gate m_gate = new Gate();

  // Set up Chooser for autonomous command
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  //********** Network Table Entries **********//

  private final ShuffleboardTab matchTab = Shuffleboard.getTab("Matches");    //for during matches
  private final ShuffleboardTab commandTab = Shuffleboard.getTab("Commands"); //for testing commands
  private final ShuffleboardTab devTab = Shuffleboard.getTab("Development");  //for all else
  
  SuppliedValueWidget<Double> turretDistance = 
  commandTab.addNumber("Turret Distance", () -> m_turret.getTurretDistance())
  .withPosition(0, 3)
  .withSize(1,1);
  SuppliedValueWidget<Double> shooterSpeed =
  commandTab.addNumber("Shooter Speed", () -> m_shooter.getLeftWheelSpeed())
  .withPosition(1,3)
  .withSize(1,1);

  SuppliedValueWidget<Boolean> hasTargetEntry = 
    matchTab.addBoolean("Target Acquired", () -> m_turret.limelightHasValidTarget())
    .withPosition(4,0)
    .withSize(4,4);
  
  SuppliedValueWidget<String> upperCargoColor = 
    matchTab.addString("Upper Cargo Color", () -> m_lifter.getColorUpper())
    .withPosition(2, 0)
    .withSize(2,1);

  SuppliedValueWidget<String> lowerCargoColor =
    matchTab.addString("Lower Cargo Color", () -> m_lifter.getColorLower())
    .withPosition(2,1)
    .withSize(2,1);

  NetworkTableEntry autoTimeEntry = 
    devTab.add("Auto Time", Constants.Auto.DEFAULT_AUTO_TIME)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 20))
          .getEntry();
  NetworkTableEntry autoDistanceEntry =
    devTab.add("Auto Distance", 1.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 20))
        .getEntry();
  NetworkTableEntry autoSpeedEntry =
    devTab.add("Auto Speed", 0.2)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
  NetworkTableEntry targetAngleEntry = 
    devTab.add("Target Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -180, "max", 180))
        .getEntry();
  
        

  //************ Pneumatics **********/
  Compressor phCompressor = new Compressor(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain.resetYaw();  //set the navx to 0
    
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous options in Shuffleboard
    matchTab.add("Autonomous Choice", autoChooser)
    .withPosition(0,0)
    .withSize(2,2);


    autoChooser.setDefaultOption("Score 3", new InstantCommand());
    autoChooser.addOption("Auto Drive for Time", new DriveArcade(() -> 0.5, () -> 0.0, m_drivetrain)
            .withTimeout(autoTimeEntry.getDouble(1.0)));
    autoChooser.addOption("Auto Drive for Distance", new DriveStraight(m_drivetrain, autoDistanceEntry.getDouble(1.0), autoSpeedEntry.getDouble(0.0)));
    autoChooser.addOption("Auto Drive 2 feet at 0.3", new DriveStraight(m_drivetrain, 2.0, 0.3));
          

    //************* Shuffleboard Commands for Testing ***************//
    //Shuffleboard Shooter Commands

    ShuffleboardLayout shooterCommands = Shuffleboard.getTab("Commands")
      .getLayout("Shooter", BuiltInLayouts.kList)
      .withSize(2,2)
      .withPosition(0,0)
      .withProperties(Map.of("Label Position", "LEFT"));
    NetworkTableEntry shooterSpeedEntry = 
      shooterCommands.add("Shooter Speed", SHOOTER_DEFAULT_SPEED)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 5676))
        .getEntry();
    shooterCommands.add("Shooter Slider", new FunctionalCommand(
      ()->{},
      () -> m_shooter.setShooterSpeed(shooterSpeedEntry.getDouble(0)), 
      interrupted -> m_shooter.setShooterSpeed(0), 
      () -> false , 
      m_shooter));
    shooterCommands.add("Shooter Default", new FunctionalCommand(
      ()->{},
      () -> m_shooter.setShooterSpeed(SHOOTER_DEFAULT_SPEED), 
      interrupted -> m_shooter.setShooterSpeed(0), 
      () -> false , 
      m_shooter));
    shooterCommands.addNumber("Default Shooter Speed",() -> SHOOTER_DEFAULT_SPEED);
    
    
    //Shuffleboard Lifter Commands
    ShuffleboardLayout lifterCommands = Shuffleboard.getTab("Commands")
      .getLayout("Lifter-Loader", BuiltInLayouts.kList)
      .withSize(2,3)
      .withPosition(2,0)
      .withProperties(Map.of("Label Position", "LEFT"));
    NetworkTableEntry lifterSpeedEntry = 
      lifterCommands.add("Lifter Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    lifterCommands.add("Lifter Slider", new FunctionalCommand(
      ()->{},
      () -> m_lifter.setMotorPower(lifterSpeedEntry.getDouble(0)), 
      interrupted -> {m_lifter.setMotorPower(0); m_loader.setMotorSpeed(0);}, 
      () -> false ,
       m_lifter));
    lifterCommands.add("Lifter Default", new FunctionalCommand(
      ()->{},
      () -> m_lifter.setMotorPower(LIFTER_DEFAULT_SPEED), 
      interrupted -> {
        m_lifter.setMotorPower(0); 
        m_loader.setMotorSpeed(0);
      }, 
      () -> false , 
      m_lifter));
    lifterCommands.addNumber("Default Lifter Speed",() -> LIFTER_DEFAULT_SPEED);
    lifterCommands.add("Spin Loader Wheels", new RunCommand(() -> m_loader.setMotorSpeed(m_lifter.getMotorSpeed()*10/9), m_loader)); // always follows lifter speed
    
    //Shuffleboard Gate Commands
    ShuffleboardLayout gateCommands = Shuffleboard.getTab("Commands")
      .getLayout("Gate", BuiltInLayouts.kList)
      .withSize(2,2)
      .withPosition(6,0)
      .withProperties(Map.of("Label Position", "LEFT"));
    NetworkTableEntry gateSpeedEntry = 
      gateCommands.add("Gate Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    gateCommands.add("Gate Slider", new FunctionalCommand(
      ()->{},
      () -> m_gate.setGate(gateSpeedEntry.getDouble(0)), 
      interrupted -> m_gate.setGate(0), 
      () -> false , 
      m_gate));
    gateCommands.add("Gate Default", new FunctionalCommand(
      ()->{},
      () -> m_gate.setGate(GATE_DEFAULT_SPEED), 
      interrupted -> m_gate.setGate(0), 
      () -> false , 
      m_gate));
    gateCommands.addNumber("Default Gate Speed",() -> GATE_DEFAULT_SPEED);

    //Shuffleboard Collector Commands to ShuffleBoard
    ShuffleboardLayout collectorCommands = Shuffleboard.getTab("Commands")
      .getLayout("Collector", BuiltInLayouts.kList)
      .withSize(2,3)
      .withPosition(4,0)
      .withProperties(Map.of("Label Position", "LEFT"));
    NetworkTableEntry armSpeedEntry = 
      collectorCommands.add("Arm Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    collectorCommands.add("Collector Slider", new FunctionalCommand(
      ()->{},
      () -> m_collector.setArm(armSpeedEntry.getDouble(0)), 
      interrupted -> m_collector.setArm(0), 
      () -> false , 
      m_collector));
    collectorCommands.add("Collector Default", new FunctionalCommand(
      ()->{},
      () -> m_collector.setArm(COLLECTOR_DEFAULT_SPEED), 
      interrupted -> m_collector.setArm(0), 
      () -> false , 
      m_collector));
    collectorCommands.addNumber("Default Arm Speed",() -> COLLECTOR_DEFAULT_SPEED);
    collectorCommands.add("forward", new InstantCommand(m_collector::setCollectorForward));
    collectorCommands.add("reverse", new InstantCommand(m_collector::setCollectorReverse));

    devTab.add("Drive for 5", new DriveArcade(() ->.5, () -> 0.0, m_drivetrain).withTimeout(5));
    devTab.add("Drive for 2", new DriveArcade(()->.5, () -> 0.0, m_drivetrain).withTimeout(2));
    devTab.add("Turn to Angle", new TurnToAngle(targetAngleEntry.getDouble(0.0), m_drivetrain, driverStick ));
   
    // ************  DEFAULT COMMANDS  ***************
    m_drivetrain.setDefaultCommand(new DriveArcade(
      () -> -driverStick.getY(),
      () -> driverStick.getZ(),
      m_drivetrain));
      
      //m_lifter.setDefaultCommand(new LifterControl(
       // () -> m_lifter.getSliderValue(), 
       // m_lifter));
        
        //m_collector.setDefaultCommand(new RunCollectorVariable(
          //() -> m_collector.getArmSliderValue(),
          //() -> m_collector.getGateSliderValue(),  
     //m_collector));
     
     
     m_turret.setDefaultCommand((new TrackTargetWithLimelight(m_turret)));
     
   // m_shooter.setDefaultCommand((new RunShooter(() -> 0, m_shooter, m_turret)));
    
   
    m_hanger.setDefaultCommand(new ControlHanger(
      () -> operatorStick.getY(),
      () -> operatorStick.getRawAxis(SLIDER_AXIS),  //slider up means it's OK to Hang
      m_hanger));
      
    }
    
    /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // ************  DRIVER STICK  ***************
    huntForBallsButton = new JoystickButton(driverStick, HUNT_FOR_BALLS);
    huntForBallsButton.whenHeld(new HuntForBalls(m_collector, m_gate));

    
    invertCollectorButton = new JoystickButton(driverStick, COLLECTOR_REVERSE);
    invertCollectorButton.whenPressed(new RaiseHangerToHeight(50, m_hanger));
    
    //Raises hanger to max height
    hangerToMaxHeightButton = new POVButton(driverStick, ELEVATOR_UP);
    hangerToMaxHeightButton.whenPressed(new ConditionalCommand(new RaiseHangerToHeight(MAX_HEIGHT, m_hanger), 
      new InstantCommand(() -> {}), () -> driverStick.getRawAxis(3) < -0.8));       
    /*
    armToggleButton = new JoystickButton(driverStick, Constants.Buttons.ARM_TOGGLE);
    armToggleButton.whenPressed(new ToggleCollector(m_collector));
    
    invertCollectorButton = new JoystickButton(driverStick, Constants.Buttons.COLLECTOR_REVERSE);
    invertCollectorButton.whenPressed(m_collector::invertDirection);
    //armToggleButton.whenPressed(m_collector::toggleCollector);
    
    */
    /* */
    
    // ************  OPERATOR STICK  ***************
    //Hanger pneumatics buttons 
    hangerPneumaticsReverseButton = new JoystickButton(operatorStick, Constants.Buttons.HANGER_PNEUMATICS_REVERSE);
    hangerPneumaticsReverseButton.whenPressed(m_hanger::pullHangerIn);
    
    hangerPneumaticsForwardButton = new JoystickButton(operatorStick, Constants.Buttons.HANGER_PNEUMATICS_FORWARD);
    hangerPneumaticsForwardButton.whenPressed(m_hanger::pushHangerOut);
    
    
    shootCargoButton = new JoystickButton(operatorStick, Constants.Buttons.SHOOT_ALLIANCE_BALL);
    shootCargoButton.whenPressed(new ConditionalCommand(new InstantCommand(() -> {}, m_hanger),  //requires hanger so operator can resume control in 'Hang Mode'
    new xxRunShooter(() -> 1, m_shooter, m_turret), () -> operatorStick.getRawAxis(3) < -0.8));          //2nd command will shoot
    
    incrementHangerUpButton = new POVButton(operatorStick, ELEVATOR_UP);
    incrementHangerUpButton.whenPressed(new IncrementHangerUp(m_hanger.getHangerPosition(), m_hanger));
    
    burpBallButton = new JoystickButton(operatorStick, 9);
    burpBallButton.whileHeld(new xxShootDefault(() -> 0.3, m_shooter,() -> 0.3, m_lifter, m_turret, m_loader));

    //operatorStick.getRawAxis(3) < -0.8
    //new InstantCommand(() -> {}, m_hanger)

    //shootCargoButton.whileHeld(new RunLifter(
      //m_lifter,
      //() -> Constants.Lifter.DEFAULT_SPEED
      //));
      /*
    turnTurretToZeroButton = new JoystickButton(operatorStick, Constants.Buttons.TURRET_TO_ZERO);
    turnTurretToZeroButton.whenPressed(new TurnTurretToAngle(
      Constants.Turret.ZERO,
      m_turret));
      
     turretAimToggleButton = new JoystickButton(operatorStick, Constants.Buttons.AIM_TOGGLE);
     turretAimToggleButton.toggleWhenPressed(new TurnTurretWithJoystick(() -> operatorStick.getZ(), m_turret));
     
     runShooterToggleButton = new JoystickButton(operatorStick, Constants.Buttons.RUN_SHOOTER_TOGGLE);
     runShooterToggleButton.toggleWhenPressed(new RunShooter(() -> m_shooter.getSliderValue(), m_shooter, m_turret));

     shootSliderButton = new JoystickButton(operatorStick, Constants.Buttons.SHOOT_WITH_SLIDER);
     shootSliderButton.whileHeld(new ShootDefault(() -> m_shooter.getSliderValue(), m_shooter, () -> Constants.Lifter.DEFAULT_SPEED, m_lifter, m_turret));
    
    runLifterReverseButton = new JoystickButton(operatorStick, Constants.Buttons.REVERSE_LIFTER);
    runLifterReverseButton.whileHeld(new RunLifter(m_lifter, () -> -0.2).withTimeout(1));


    runShooterAtSlider = new JoystickButton(operatorStick, 12);
    runShooterAtSlider.whileHeld(new RunShooterAtSpeed(() -> m_shooter.getSliderValue(), m_shooter));
    */
    
    //cancelHangerUpButton = new POVButton(operatorStick, ELEVATOR_CANCEL);
    //cancelHangerUpButton.whenPressed(new IncrementHangerUp(m_hanger.getHangerPosition(), m_hanger));
    

  
    
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    

    //m_drivetrain.resetYaw();          // zero the yaw on the navx at start of Autonomous period
    return autoChooser.getSelected();

    
  }

}
