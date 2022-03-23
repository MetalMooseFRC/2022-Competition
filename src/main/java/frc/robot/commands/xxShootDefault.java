// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;

import java.util.function.DoubleSupplier;

public class xxShootDefault extends CommandBase {
  private Shooter m_shooter;
  private Lifter m_lifter;
  private Loader m_loader;
  private final DoubleSupplier m_shooterSpeedSupplier;
  private final DoubleSupplier m_lifterSpeedSupplier;
  private final Turret m_turret;

  /** Creates a new StaticShooting. */
  public xxShootDefault(DoubleSupplier speedSupplier, Shooter shooter, DoubleSupplier speedSupplier2, Lifter lifter, Turret turret, Loader loader) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSpeedSupplier = speedSupplier;
    m_lifterSpeedSupplier = speedSupplier2;

    m_shooter = shooter;
    m_lifter = lifter;
    m_turret = turret;
    m_loader = loader;
    addRequirements(m_shooter, m_lifter, m_loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double dis = m_turret.getTurretDistance();

    // double power = 0.587 + (-0.000404 * dis) + (0.00000105 * (Math.pow(dis, 2)));
    // Set shooter wheels to calculated speeds
    m_shooter.m_motorLeft.set(m_shooterSpeedSupplier.getAsDouble());
    m_shooter.m_motorRight.set(m_shooterSpeedSupplier.getAsDouble());
    m_lifter.m_motor.set(m_lifterSpeedSupplier.getAsDouble());
    m_loader.m_motor.set(m_lifterSpeedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_shooter.m_motorLeft.set(0);
   m_shooter.m_motorRight.set(0);
   m_lifter.m_motor.set(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
