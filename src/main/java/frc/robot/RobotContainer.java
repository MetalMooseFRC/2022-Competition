// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveArcadeWithJoystick;
import frc.robot.commands.HangerControl;
import frc.robot.commands.LifterControl;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.TurnTurretWithJoystick;
import frc.robot.commands.RunCollectorVariable;
import frc.robot.commands.RunCollectorDefault;
import frc.robot.commands.ToggleCollector;
import frc.robot.commands.ShootDefault;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Turret;


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
  JoystickButton shootCargoButton, turnTurretToZeroButton;



  // ************  Subsystems  **************
  private Drivetrain m_drivetrain = new Drivetrain();
  private Shooter m_shooter = new Shooter();
  private Lifter m_lifter = new Lifter();
  private Collector m_collector = new Collector();
  private Hanger m_hanger = new Hanger();
  private Turret m_turret = new Turret();

  //************ Pneumatics **********/
  Compressor phCompressor = new Compressor(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // ************  DEFAULT COMMANDS  ***************
    m_drivetrain.setDefaultCommand(new DriveArcadeWithJoystick(
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

    

    m_turret.setDefaultCommand((new TurnTurretWithJoystick(
      () -> operatorStick.getZ(),
       m_turret)));

    m_shooter.setDefaultCommand((new ShooterControl(() -> m_shooter.getSliderValue(), m_shooter)));

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
      //armToggleButton.whenPressed(m_collector::toggleCollector);
      
    // ************  OPERATOR STICK  ***************
    hangerPneumaticsToggleButton = new JoystickButton(operatorStick, Constants.Buttons.HANGER_PNEUMATICS_TOGGLE);
    hangerPneumaticsToggleButton.whenPressed(m_hanger::toggleSolenoid);

    shootCargoButton = new JoystickButton(operatorStick, Constants.Buttons.SHOOT_ALLIANCE_BALL);
    shootCargoButton.whileHeld(new ShootDefault(
      () -> Constants.Shooter.DEFAULT_SPEED,
      m_shooter,
      () -> Constants.Lifter.DEFAULT_SPEED,
      m_lifter));

    turnTurretToZeroButton = new JoystickButton(operatorStick, Constants.Buttons.TURRET_TO_ZERO);
    turnTurretToZeroButton.whenPressed(new TurnTurretToAngle(
       Constants.Turret.ZERO,
       m_turret));

  }
}
