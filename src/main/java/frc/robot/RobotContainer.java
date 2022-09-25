// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Lifter.*;
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

  // ************ Camera *****************
  public VideoCamera driverCam;
  
  // ************  OI Controller  ***************
  public static final Joystick driverStick = new Joystick(Constants.DSPorts.DRIVER_STICK_PORT);
  JoystickButton huntForBallsButton;
  JoystickButton hangerPneumaticsReverseButton, hangerPneumaticsForwardButton; //armToggleButton, armCollectButton;

  public static final Joystick driverTurnStick = new Joystick(2);
  
  public static final Joystick operatorStick = new Joystick(Constants.DSPorts.OPERATOR_STICK_PORT);
  JoystickButton shootCargoButton, turnTurretToZeroButton, shootingSpeedUpButton, shootingSpeedDownButton, turretAimToggleButton,
   invertCollectorButton, runShooterToggleButton, shootSliderButton, runLifterReverseButton, turnTestButton, runShooterAtSlider,
   huntBallAssistButton, holdHangerButton, restartLifterLoaderButton, searchForHubButton, autoShootingButton, manualShootingButton,
   lifterLoaderRunButton, turnRobotToHubButton, burpButton
   ;
  POVButton pullRobotUpButton, pullRobotHalfwayUpButton, incrementHangerUpButton, incrementHangerDownButton, hangerToMaxHeightButton,
   cancelHangerUpButton, turnTurretTo90Button, turnTurretToN90Button, stopLifterLoaderButton, stopShooterMotorsButton,
   turn180Button;

  // ************  Subsystems  **************
  private Drivetrain m_drivetrain = new Drivetrain(); //Drivetrain. Contains ball tracking limelight

  // Shooting subsystems: 
  private Collector m_collector = new Collector(); // Arm w/ wheels
  private Gate m_gate = new Gate(); // Vertical shaft
  private Lifter m_lifter = new Lifter(); // Top two black wheels
  private Loader m_loader = new Loader(); // Isolated section (originally part of the lifter) that helps gives more control over when to shoot
  private Shooter m_shooter = new Shooter(); // Flywheels
  private Turret m_turret = new Turret(); // Aims turntable. Contains top limelight
  
  // Endgame:
  private Hanger m_hanger = new Hanger(); // Elevator on the back

  // Set up Chooser for autonomous command
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  //********** Shuffleboard Tabs **********//
  private final ShuffleboardTab matchTab = Shuffleboard.getTab("Matches");    //for during matches
  // private final ShuffleboardTab commandTab = Shuffleboard.getTab("Commands"); //for testing commands
  // private final ShuffleboardTab devTab = Shuffleboard.getTab("Development");  //for all else
  
  //********** Shuffleboard Widgets **********//
  
  // Manually view shooter speed and turret's distance to target
  // Useful for recalculating power function for shooting
  // SuppliedValueWidget<Double> turretDistance = 
  // commandTab.addNumber("Turret Distance", () -> m_turret.getTurretDistance())
  // .withPosition(0, 3)
  // .withSize(1,1);
  // SuppliedValueWidget<Double> shooterSpeed =
  // commandTab.addNumber("Shooter Speed", () -> m_shooter.getLeftWheelSpeed())
  // .withPosition(1,3)
  // .withSize(1,1);

// Lets driver/operator know if limelight has target
  SuppliedValueWidget<Boolean> hasTargetEntry = 
  matchTab.addBoolean("Target Acquired", () -> m_turret.limelightHasValidTarget())
  .withPosition(4,0)
  .withSize(4,4);

  SuppliedValueWidget<Boolean> isInSweetSpot = 
  matchTab.addBoolean("In Sweet Spot", () -> (m_turret.getTurretDistance()<475) 
  && (m_turret.getTurretDistance()>310))
  .withPosition(3,2)
  .withSize(2,2);
  
// Tells driver/operator what ball is in the upper "slot"
  SuppliedValueWidget<String> upperCargoColor = 
  matchTab.addString("Upper Cargo Color", () -> m_lifter.getColorUpper())
  .withPosition(2, 0)
  .withSize(2,1);
  
// Tells driver/operator what ball is in the lower "slot" 
  SuppliedValueWidget<String> lowerCargoColor =
  matchTab.addString("Lower Cargo Color", () -> m_lifter.getColorLower())
  .withPosition(2,1)
  .withSize(2,1);
  
  // Hanger Joystick input for testing
  // Useful for knowing operatorStick input when configuring hanger changes
  // SuppliedValueWidget<Double> hangerJoystickInput =
  // commandTab.addNumber("Hanger Joystick", () -> operatorStick.getY())
  // .withPosition(4,4)
  // .withSize(1,1);

  //Operator stick z, for testing turret feed forward
  // SuppliedValueWidget<Double> operatorStickInput = 
  // matchTab.addNumber("Operator Stick", () -> operatorStick.getZ());
  
  // Team Alliance Color Widget
  // SuppliedValueWidget<String> driverStationAlliance = 
  // commandTab.addString("Alliance String", () -> DriverStation.getAlliance().toString())
  // .withSize(1,1);
  
  // If you want to test autonomous for a non-standard amount of time
  // NetworkTableEntry autoTimeEntry = 
  // devTab.add("Auto Time", Constants.Auto.DEFAULT_AUTO_TIME)
  // .withWidget(BuiltInWidgets.kNumberSlider)
  // .withProperties(Map.of("min", 0, "max", 20))
  // .getEntry();

  // For driving a total distance in auto
  // NetworkTableEntry autoDistanceEntry =
  // devTab.add("Auto Distance", 1.0)
  // .withWidget(BuiltInWidgets.kNumberSlider)
  // .withProperties(Map.of("min", 0, "max", 20))
  // .getEntry();

  // Speed of autonomous
  // NetworkTableEntry autoSpeedEntry =
  // devTab.add("Auto Speed", 0.2)
  // .withWidget(BuiltInWidgets.kNumberSlider)
  // .withProperties(Map.of("min", 0, "max", 1))
  // .getEntry();

  // Angle of target for testing
  // NetworkTableEntry targetAngleEntry = 
  // devTab.add("Target Angle", 0.0)
  // .withWidget(BuiltInWidgets.kNumberSlider)
  // .withProperties(Map.of("min", -180, "max", 180))
  // .getEntry();
  

  //************ Pneumatics **********/
  // RevRobotics Pneumatics Hub Compressor from CANID
  Compressor phCompressor = new Compressor(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    LiveWindow.disableAllTelemetry();
    //set the navx to 0 (at start of match)
    m_drivetrain.resetYaw();  
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous options in Shuffleboard
    matchTab.add("Autonomous Choice", autoChooser)
    .withPosition(0,0)
    .withSize(2,2);

    // Add individual options for autonomous to the chooser
    autoChooser.setDefaultOption("New Five Ball", new AutoFiveBallBlue(m_drivetrain, m_shooter, m_lifter, m_loader, m_gate, m_collector, m_turret));
    autoChooser.addOption("Center Four Ball", new AutoFourBallCenterFixed(m_drivetrain, m_shooter, m_lifter, m_loader, m_gate, m_collector, m_turret));
    autoChooser.addOption("Anywhere Two Ball", new AutoTwoBallNormal(m_drivetrain, m_shooter, m_lifter, m_loader, m_gate, m_collector, m_turret));
    autoChooser.addOption("Wall Two Ball", new AutoTwoBallWall(m_drivetrain, m_shooter, m_lifter, m_loader, m_gate, m_collector, m_turret));
    autoChooser.addOption("Wall Four Ball", new AutoFourBallWall(m_drivetrain, m_shooter, m_lifter, m_loader, m_gate, m_collector, m_turret));
    autoChooser.addOption("Wall Three Ball", new AutoThreeBall(m_drivetrain, m_shooter, m_lifter, m_loader, m_gate, m_collector, m_turret));
    autoChooser.addOption("Taxi and Shoot", new ParallelCommandGroup(
        new InstantCommand(() -> m_turret.turretMotor.set(Constants.Turret.DEFAULT_SPEED), m_turret)
          .andThen(new TrackTargetWithLimelight(m_turret)),
        new SequentialCommandGroup(
          new DriveStraight(m_drivetrain, 4, 0.5),
          new ShootingSequence(m_shooter, m_turret, m_gate, m_lifter, m_loader))));
    autoChooser.addOption("Taxi from Tarmac Edge", new DriveArcade(() -> 0.5, () -> 0.0, m_drivetrain)
            .withTimeout(3));
          

    //************* Shuffleboard Commands for Testing manually  ***************//
    //Shuffleboard Shooter Commands to manually run specific commands (for testing)
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
    // shooterCommands.add("Shooter Default", new FunctionalCommand(
    //   ()->{},
    //   () -> m_shooter.setShooterSpeed(SHOOTER_DEFAULT_SPEED), 
    //   interrupted -> m_shooter.setShooterSpeed(0), 
    //   () -> false , 
    //   m_shooter));
    // shooterCommands.addNumber("Default Shooter Speed",() -> SHOOTER_DEFAULT_SPEED);
    
    
    // //Shuffleboard Lifter Commands
    ShuffleboardLayout lifterCommands = Shuffleboard.getTab("Commands")
      .getLayout("Lifter-Loader", BuiltInLayouts.kList)
      .withSize(2,3)
      .withPosition(2,0)
      .withProperties(Map.of("Label Position", "LEFT"));
    // NetworkTableEntry lifterSpeedEntry = 
    //   lifterCommands.add("Lifter Speed", 0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();
    // lifterCommands.add("Lifter Slider", new FunctionalCommand(
    //   ()->{},
    //   () -> m_lifter.setMotorPower(lifterSpeedEntry.getDouble(0)), 
    //   interrupted -> {
    //     m_lifter.setMotorPower(0);
    //     //m_loader.setMotorPower(0);
    //   },
    //   () -> false ,
      //  m_lifter));
    lifterCommands.add("Lifter Default", new FunctionalCommand(
      ()->{},
      () -> m_lifter.setMotorPower(LIFTER_DEFAULT_SPEED), 
      interrupted -> {
        m_lifter.setMotorPower(0); 
        //m_loader.setMotorPower(0);
      }, 
      () -> false , 
      m_lifter));
    // lifterCommands.addNumber("Default Lifter Speed",() -> LIFTER_DEFAULT_SPEED);
    lifterCommands.add("Spin Loader Wheels", new RunCommand(() -> m_loader.setMotorSpeed(m_lifter.getMotorSpeed()*10/9), m_loader)); // always follows lifter speed
    // lifterCommands.add("Idle Loader Wheels", new RunCommand(() -> m_loader.setMotorPower(-0.1)));

    // //Shuffleboard Gate Commands
    // ShuffleboardLayout gateCommands = Shuffleboard.getTab("Commands")
    //   .getLayout("Gate", BuiltInLayouts.kList)
    //   .withSize(2,2)
    //   .withPosition(6,0)
    //   .withProperties(Map.of("Label Position", "LEFT"));
    // NetworkTableEntry gateSpeedEntry = 
    //   gateCommands.add("Gate Speed", 0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();
    // gateCommands.add("Gate Slider", new FunctionalCommand(
    //   ()->{},
    //   () -> m_gate.setGate(gateSpeedEntry.getDouble(0)), 
    //   interrupted -> m_gate.setGate(0), 
    //   () -> false , 
    //   m_gate));
    // gateCommands.add("Gate Default", new FunctionalCommand(
    //   ()->{},
    //   () -> m_gate.setGate(GATE_DEFAULT_SPEED), 
    //   interrupted -> m_gate.setGate(0), 
    //   () -> false , 
    //   m_gate));
    // gateCommands.addNumber("Default Gate Speed",() -> GATE_DEFAULT_SPEED);

    // //Shuffleboard Collector Commands to ShuffleBoard
    // ShuffleboardLayout collectorCommands = Shuffleboard.getTab("Commands")
    //   .getLayout("Collector", BuiltInLayouts.kList)
    //   .withSize(2,3)
    //   .withPosition(4,0)
    //   .withProperties(Map.of("Label Position", "LEFT"));
    // NetworkTableEntry armSpeedEntry = 
    //   collectorCommands.add("Arm Speed", 0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();
    // collectorCommands.add("Collector Slider", new FunctionalCommand(
    //   ()->{},
    //   () -> m_collector.setArm(armSpeedEntry.getDouble(0)), 
    //   interrupted -> m_collector.setArm(0), 
    //   () -> false , 
    //   m_collector));
    // collectorCommands.add("Collector Default", new FunctionalCommand(
    //   ()->{},
    //   () -> m_collector.setArm(COLLECTOR_DEFAULT_SPEED), 
    //   interrupted -> m_collector.setArm(0), 
    //   () -> false , 
    //   m_collector));
    // collectorCommands.addNumber("Default Arm Speed",() -> COLLECTOR_DEFAULT_SPEED);
    // collectorCommands.add("forward", new InstantCommand(m_collector::pushCollectorOut));
    // collectorCommands.add("reverse", new InstantCommand(m_collector::pullCollectorIn));

    // devTab.add("Drive for 5", new DriveArcade(() ->.5, () -> 0.0, m_drivetrain).withTimeout(5));
    // devTab.add("Drive for 2", new DriveArcade(()->.5, () -> 0.0, m_drivetrain).withTimeout(2));
    // devTab.add("Turn to Angle", new TurnToAngle(targetAngleEntry.getDouble(0.0), m_drivetrain));
   
    // ************  DEFAULT COMMANDS  ***************
    // Standard driving command. Be very sure you know what you're doing before you change this
    // m_drivetrain.setDefaultCommand(new DriveArcade(
    //   () -> -driverStick.getY(),
    //   () -> driverStick.getZ(),
    //   m_drivetrain));
    m_drivetrain.setDefaultCommand(new DriveArcade(
      () -> -driverStick.getY(),
      () -> driverStick.getZ(),
      m_drivetrain));

// 4/4/22 Shooter Spin up default command added
    // m_shooter.setDefaultCommand(new SpinUpShooter(m_shooter, m_turret, m_lifter));
      
    // Track hub w/ limelight by default
    m_turret.setDefaultCommand((new TrackTargetWithLimelight(m_turret)));
   
    // By default allow opstick to raise/lower hanger
    m_hanger.setDefaultCommand(new ControlHanger(
      () -> operatorStick.getY(),
      () -> operatorStick.getRawAxis(SLIDER_AXIS),  //slider up means it's OK to Hang
      m_hanger));
      
    // Run lifter upwards until a ball is in the upper slot
    m_lifter.setDefaultCommand(new IdleLifterLoader(
      m_lifter,
     () -> LIFTER_DEFAULT_SPEED,
     m_loader,
     () -> -10/9*m_lifter.getMotorSpeed()));
     
    }
    
    /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     
    // ************  DRIVER STICK  ***************
    // Toggle the collector
    huntForBallsButton = new JoystickButton(driverStick, HUNT_FOR_BALLS);
    huntForBallsButton.whenHeld(
      new ParallelCommandGroup(
        new IdleLifterLoader(
          m_lifter,
          () -> LIFTER_DEFAULT_SPEED,
          m_loader,
          // 10/9 is gear ratio
          () -> -10/9*m_lifter.getMotorSpeed()
          ),
          new HuntForBalls(m_collector, m_gate, m_lifter)
          )
          );
          
          // Help driver by locking onto the closer ball (center robot w/ ball)
    huntBallAssistButton = new JoystickButton(driverStick, 2);
    huntBallAssistButton.whileHeld(new TurnToBall(
      () -> -driverStick.getY(),
      m_drivetrain));

    // turn180Button = new POVButton(driverStick, 0);  
    // turn180Button.whenHeld(new InstantCommand(() -> m_drivetrain.resetYaw()).andThen(new TurnToAngle(180, m_drivetrain)));

    // turnRobotToHubButton = new JoystickButton(driverStick, 5);
    // turnRobotToHubButton.whenHeld(new TurnToHub(() -> -driverStick.getY(), m_drivetrain, m_turret));
            
    //Raises hanger to max height
    hangerToMaxHeightButton = new POVButton(driverStick, ELEVATOR_UP);
    hangerToMaxHeightButton.whileHeld(new ConditionalCommand(
      new RaiseHangerToHeight(MAX_HEIGHT, m_hanger), 
      new InstantCommand(() -> {}),
      () -> driverStick.getRawAxis(3) < -0.8)); 

    stopLifterLoaderButton = new POVButton(driverStick, ELEVATOR_UP);
    stopLifterLoaderButton.whenPressed(new InstantCommand(() -> m_lifter.setMotorPower(0), m_lifter)
      .andThen(new RunCommand(() -> m_loader.setMotorPower(0),m_loader)));

    stopShooterMotorsButton = new POVButton(driverStick, ELEVATOR_UP);
    stopShooterMotorsButton.whenPressed(new RunCommand(() -> m_shooter.setShooterSpeed(0), m_shooter).until(() -> driverStick.getRawAxis(3)>0.8));
    
    restartLifterLoaderButton = new JoystickButton(driverStick, 7);
    restartLifterLoaderButton.whenPressed(
        //requires hanger so operator can resume control in 'Hang Mode'
        new InstantCommand(() -> {}, m_lifter, m_loader));
        
    //     new InstantCommand(() -> {}, m_lifter, m_loader),
            
    //     () -> driverStick.getRawAxis(3) < -0.8)); 
    
    //Pulls hanger on to mid bar
    
    // Used for manually testing certain things
    
    // armToggleButton = new JoystickButton(driverStick, Constants.Buttons.ARM_TOGGLE);
    // armToggleButton.whenPressed(new ToggleCollector(m_collector));
    
    // invertCollectorButton = new JoystickButton(driverStick, COLLECTOR_REVERSE);
    // invertCollectorButton.whenPressed(new RaiseHangerToHeight(50, m_hanger));
    
    // invertCollectorButton = new JoystickButton(driverStick, Constants.Buttons.COLLECTOR_REVERSE);
    // invertCollectorButton.whenPressed(m_collector::invertDirection);
    // armToggleButton.whenPressed(m_collector::toggleCollector);
    
    
    // ************  OPERATOR STICK  ***************
    //Hanger pneumatics buttons 
    
    shootCargoButton = new JoystickButton(operatorStick, Constants.Buttons.SHOOT_ALLIANCE_BALL);
    shootCargoButton.whileHeld(
      new ConditionalCommand(
        new InstantCommand(()-> {}, m_hanger),
        new ConditionalCommand(
          new ShootingSequenceAtSpeed(3400.0, m_shooter, m_gate, m_lifter, m_loader), 
          new ConditionalCommand(
            new ShootingSequenceAtSpeed(3000.0, m_shooter, m_gate, m_lifter, m_loader),
            new ShootingSequenceAtSpeed(3200.0, m_shooter, m_gate, m_lifter, m_loader),
            () -> (operatorStick.getY() > 0.3)), 
          () -> (operatorStick.getY() < -0.3)),
        () -> operatorStick.getRawAxis(3) < 0.8));
            
    burpButton = new JoystickButton(operatorStick, 11);
    burpButton.whenPressed(new ShootingSequenceAtSpeed(2000.0, m_shooter, m_gate, m_lifter, m_loader));
    
    lifterLoaderRunButton = new JoystickButton(operatorStick, 9);
    lifterLoaderRunButton.whenHeld(new RunLifterLoader(m_lifter, LIFTER_DEFAULT_SPEED, m_loader, LIFTER_DEFAULT_SPEED*10/9));

    hangerPneumaticsReverseButton = new JoystickButton(operatorStick, Constants.Buttons.HANGER_PNEUMATICS_REVERSE);
    hangerPneumaticsReverseButton.whenPressed(m_hanger::pullHangerIn);
    
    hangerPneumaticsForwardButton = new JoystickButton(operatorStick, Constants.Buttons.HANGER_PNEUMATICS_FORWARD);
    hangerPneumaticsForwardButton.whenPressed(m_hanger::pushHangerOut);

    // pullRobotHalfwayUpButton = new POVButton(operatorStick, 0);
    // pullRobotHalfwayUpButton.whenHeld(new ConditionalCommand(
    //   new PullUpHalfway(m_hanger),
    //   new InstantCommand(() -> {}),
    //   () -> operatorStick.getRawAxis(3)<-0.8));
    
    // pullRobotUpButton = new POVButton(operatorStick, ELEVATOR_DOWN);
    // pullRobotUpButton.whenPressed(new PullUpToMidBar(m_hanger));
  
    searchForHubButton = new JoystickButton(operatorStick, 2);
    searchForHubButton.whenPressed(
      new RunCommand(() -> m_turret.turretMotor.set(0.25), m_turret)
        .until(() -> m_turret.limelightHasValidTarget()));
        
        
    autoShootingButton = new JoystickButton(operatorStick, 3);
    autoShootingButton.whenPressed(new InstantCommand(() -> m_turret.setTurretMode("auto"), m_turret));
    
    manualShootingButton = new JoystickButton(operatorStick, 5);
    manualShootingButton.whenPressed(new InstantCommand(() -> m_turret.setTurretMode("manual"))
      .andThen(new TurnTurretToAngle(Constants.Turret.ZERO, m_turret)
      .andThen(new RunCommand(() -> {}, m_turret))));
    
    // turnTurretToZeroButton = new JoystickButton(operatorStick, 5);
    // turnTurretToZeroButton.whenPressed(new TurnTurretToAngle(Constants.Turret.ZERO, m_turret)
    //   .andThen(new RunCommand(() -> {}, m_turret)));
            
    // pullRobotUpButton = new POVButton(operatorStick, ELEVATOR_DOWN);
    // pullRobotUpButton.whenPressed(new ConditionalCommand(new PullUpStep2(m_hanger), new PullUpStep1(m_hanger), ()-> m_hanger.getHangerPosition() < STEP_1+10));

    // holdHangerButton = new JoystickButton(operatorStick, 2);
    // holdHangerButton.whenPressed(new ControlHanger(() -> 0.07, () -> operatorStick.getRawAxis(3), m_hanger));
    
    //pullRobotUpWithPitchButton = new POVButton(operatorStick, ELEVATOR_UP);
    //pullRobotUpWithPitchButton.whenPressed(new PullRobotUpWithPitch(m_hanger, m_drivetrain));
      
    // shootSliderButton = new JoystickButton(operatorStick, Constants.Buttons.SHOOT_WITH_SLIDER);
    // shootSliderButton.whileHeld(new ShootDefault(() -> m_shooter.getSliderValue(), m_shooter, () -> Constants.Lifter.DEFAULT_SPEED, m_lifter, m_turret));
    
    // runLifterReverseButton = new JoystickButton(operatorStick, Constants.Buttons.REVERSE_LIFTER);
    // runLifterReverseButton.whileHeld(new RunLifter(m_lifter, () -> -0.2).withTimeout(1));

    // runShooterAtSlider = new JoystickButton(operatorStick, 12);
    // runShooterAtSlider.whileHeld(new RunShooterAtSpeed(() -> m_shooter.getSliderValue(), m_shooter));
          
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
