// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CollectorControl;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.LifterControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // ************  OI Controller  ***************
  private static final Joystick driverStick = new Joystick(Constants.DSPorts.DRIVER_STICK_PORT);
  JoystickButton armToggleButton, armCollectButton;

  // ************  Subsystems  **************
  private Drivetrain m_drivetrain = new Drivetrain();
  private Shooter m_shooter = new Shooter();
  private Lifter m_lifter = new Lifter();
  private Collector m_collector = new Collector();

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

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    armToggleButton = new JoystickButton(driverStick, Constants.Buttons.ARM_TOGGLE_BUTTON);
      armToggleButton.whenPressed(m_collector::toggleArm);

    //armCollectButton = new JoystickButton(driverStick, 1);
    //armCollectButton.whenPressed(m_collector::setArm(.4));

  }
}
