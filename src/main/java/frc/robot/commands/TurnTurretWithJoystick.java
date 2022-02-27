// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurnTurretWithJoystick extends CommandBase {
  /** Creates a new TurnTurretWithJoystick. */
  private final Turret m_turret;
  private final DoubleSupplier m_speedSupplier;

  public TurnTurretWithJoystick(DoubleSupplier speedSupplier, Turret turret) {
    m_speedSupplier =speedSupplier;
    m_turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.m_turretMotor.set(MathUtil.clamp(m_speedSupplier.getAsDouble(),-0.3,0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
