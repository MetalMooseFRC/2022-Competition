// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger extends SubsystemBase {
  DoubleSolenoid m_hangerSolenoid = new DoubleSolenoid(Constants.CANIDs.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, Constants.PneumaticsIDs.HANGER_A, Constants.PneumaticsIDs.HANGER_B);
  public CANSparkMax m_motorClimberLeft = new CANSparkMax(Constants.CANIDs.HA_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public CANSparkMax m_motorClimberRight = new CANSparkMax(Constants.CANIDs.HA_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  /** Creates a new Hanger. */
  public Hanger() {
    m_hangerSolenoid.set(kForward);
    m_motorClimberLeft.setInverted(true);
  }

  @Override
  public void periodic() {

    
    SmartDashboard.putNumber("Right Elevator Encoder", m_motorClimberRight.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Elevator Encoder", m_motorClimberLeft.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }

    /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value Value to clip.
   * @param deadband Range around zero.
   * @return The value after the deadband is applied.
   */
  private static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }


  public void toggleSolenoid() {
    System.out.print(m_hangerSolenoid.get());
    m_hangerSolenoid.toggle();
  }

  public void controlHanger(double speed) {
    double adjustedSpeed = applyDeadband(speed, Constants.Preferences.DEADBAND);
    m_motorClimberLeft.set(adjustedSpeed);
    m_motorClimberRight.set(adjustedSpeed);
  }
}
