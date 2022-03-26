// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends ParallelCommandGroup {
  /** Creates a new ShootingSequence. */
  private Shooter m_shooter;
  private Turret m_turret;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Gate m_gate;
  private Collector m_collector;
  
  public ShootingSequence(Shooter shooter, Turret turret, Drivetrain drivetrain, Lifter lifter, Loader loader, Gate gate, Collector collector) {

    m_collector = collector;
    m_shooter = shooter;
    m_turret = turret;
    m_drivetrain = drivetrain;
    m_lifter = lifter;
    m_loader = loader;
    m_gate = gate;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
          new RunShooterWithTurret(m_shooter, m_turret),
          new DriveArcade(() -> 0.0, () -> 0.0, m_drivetrain),
          new RunCommand(() -> m_gate.setGate(0.0)),
          new RunCommand(() -> m_collector.stopCollecting()),
          new WaitUntilCommand(() -> (m_shooter.getLeftWheelSpeed() >= m_turret.getRequiredVelocity())),
          new RunLifterLoader(m_lifter, Constants.Lifter.LIFTER_DEFAULT_SPEED, m_loader, Constants.Lifter.LIFTER_DEFAULT_SPEED*10/9).withTimeout(0.7)
          );
    
  }
}
