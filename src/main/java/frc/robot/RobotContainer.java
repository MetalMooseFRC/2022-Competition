// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Buttons;
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
  JoystickButton armToggleButton, armCollectButton, hangerPneumaticsToggleButton;

  public static final Joystick operatorStick = new Joystick(Constants.DSPorts.OPERATOR_STICK_PORT);
  JoystickButton shootCargoButton, turnTurretToZeroButton, shootingSpeedUpButton, shootingSpeedDownButton, turretAimToggleButton,
   invertCollectorButton, runShooterToggleButton, shootSliderButton, runLifterReverseButton, burpBallButton, runShooterAtSlider;
  POVButton turnTurretTo90Button, turnTurretToN90Button;


  // ************  Subsystems  **************
  private Drivetrain m_drivetrain = new Drivetrain();
  private Shooter m_shooter = new Shooter();
  private Lifter m_lifter = new Lifter();
  private Collector m_collector = new Collector();
  private Hanger m_hanger = new Hanger();
  private Turret m_turret = new Turret();

  //************ Autonomous Command **********/
  // private final ParallelRaceGroup m_autoCommand = new DriveArcade(() -> 0.5, () -> 0, m_drivetrain).withTimeout(1);

  // Sequentially: 
  private final SequentialCommandGroup m_autoCommand = new SequentialCommandGroup(
    // Spin up shooter wheels
    new RunShooterAtSpeed(() -> 0.6285, m_shooter).withTimeout(1),
    // Drop Collector Arm, Turn on Gate
    new ToggleCollector(m_collector),
    // Drive Forward
    new DriveArcade(() -> 0.5, () -> 0, m_drivetrain).withTimeout(3),
    // Do nothing for one loop
    new DriveArcade(() -> 0.0, () -> 0, m_drivetrain).withTimeout(1),
    //spin up shooter
    new TrackTargetWithLimelight(m_turret).withTimeout(2),
    new RunShooterAtSpeed(() -> 0.6285, m_shooter).withTimeout(1),
    // Shoot shooter
    new ShootDefault(() -> 0.6285, m_shooter, () -> Constants.Lifter.DEFAULT_SPEED, m_lifter, m_turret).withTimeout(5),
    // Toggle Collector
    new ToggleCollector(m_collector),
    // Drive Forward
    new DriveArcade(() -> 0.3, () -> 0, m_drivetrain).withTimeout(15)
    //zero turret
  );


  //************ Pneumatics **********/
  Compressor phCompressor = new Compressor(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    
    // ************  DEFAULT COMMANDS  ***************
    m_drivetrain.setDefaultCommand(new DriveArcade(
      () -> -driverStick.getY(),
      () -> driverStick.getZ(),
      m_drivetrain));
      
      m_lifter.setDefaultCommand(new LifterControl(
        () -> m_lifter.getSliderValue(), 
        m_lifter));
        
        //m_collector.setDefaultCommand(new RunCollectorVariable(
          //() -> m_collector.getArmSliderValue(),
          //() -> m_collector.getGateSliderValue(),  
     //m_collector));
     
     
     m_turret.setDefaultCommand((new TrackTargetWithLimelight(m_turret)));
     
    m_shooter.setDefaultCommand((new RunShooter(() -> 0, m_shooter, m_turret)));
    
    m_hanger.setDefaultCommand(new HangerControl(
      () -> operatorStick.getY(), 
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
    armToggleButton = new JoystickButton(driverStick, Constants.Buttons.ARM_TOGGLE);
    armToggleButton.whenPressed(new ToggleCollector(m_collector));
    
    invertCollectorButton = new JoystickButton(driverStick, Constants.Buttons.COLLECTOR_REVERSE);
    invertCollectorButton.whenPressed(m_collector::invertDirection);
    //armToggleButton.whenPressed(m_collector::toggleCollector);
    
    // ************  OPERATOR STICK  ***************
    hangerPneumaticsToggleButton = new JoystickButton(operatorStick, Constants.Buttons.HANGER_PNEUMATICS_TOGGLE);
    hangerPneumaticsToggleButton.whenPressed(m_hanger::toggleSolenoid);
    
    shootCargoButton = new JoystickButton(operatorStick, Constants.Buttons.SHOOT_ALLIANCE_BALL);
    shootCargoButton.whileHeld(new RunLifter(
      m_lifter,
      () -> Constants.Lifter.DEFAULT_SPEED
      ));
      
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

    burpBallButton = new JoystickButton(operatorStick, 9);
    burpBallButton.whileHeld(new ShootDefault(() -> 0.3, m_shooter,() -> 0.3, m_lifter, m_turret));

    runShooterAtSlider = new JoystickButton(operatorStick, 12);
    runShooterAtSlider.whileHeld(new RunShooterAtSpeed(() -> m_shooter.getSliderValue(), m_shooter));
       // turnTurretTo90Button = new POVButton(operatorStick, Constants.Buttons.ELEVATOR_MAX_UP);
    // turnTurretTo90Button.whenPressed(new ElevatorToHeight(90, m_turret));
    
    // turnTurretToN90Button = new POVButton(operatorStick, Constants.Buttons.ELEVATOR_MAX_DOWN);
    // turnTurretToN90Button.whenPressed(new ElevatorToHeight(-90, m_turret));
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("Getting autonomous command!");
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

}
