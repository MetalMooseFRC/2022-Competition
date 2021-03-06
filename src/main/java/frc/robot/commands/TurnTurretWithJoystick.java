// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;


// Turn the turret to an angle based on joystick input
public class TurnTurretWithJoystick extends CommandBase {
  /** Creates a new TurnTurretWithJoystick. */
  private final Turret m_turret;
  private final DoubleSupplier m_speedSupplier;

  public TurnTurretWithJoystick(DoubleSupplier speedSupplier, Turret turret) {
    m_speedSupplier =speedSupplier;
    m_turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setLimelightLights(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.turretMotor.set(MathUtil.clamp(m_turret.getDeadbandSpeed(m_speedSupplier.getAsDouble()),-Constants.Turret.CLAMP,Constants.Turret.CLAMP));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
