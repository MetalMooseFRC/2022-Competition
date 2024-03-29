// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.concurrent.locks.Condition;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Loader;

import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FxShootWhileMoving extends ParallelCommandGroup {
  /** Creates a new ShootWhileMoving. */
  private Shooter m_shooter;
  private Drivetrain m_drivetrain;
  private Lifter m_lifter;
  private Loader m_loader;
  private Double m_driveSpeed;
  private DoubleSupplier m_driverStickY;
  private Gate m_gate;
  private Turret m_turret;
   
  public FxShootWhileMoving(Drivetrain drivetrain, Shooter shooter, Lifter lifter, Loader loader, DoubleSupplier driverStickY, Double driveSpeed, Gate gate, Turret turret) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_lifter = lifter;
    m_loader = loader;
    m_driverStickY = driverStickY;
    m_driveSpeed = driveSpeed;
    m_gate = gate;
    m_turret = turret;

    addCommands(
      //first check if the hub is in front or directly behind (we can adjust turret based on that)
      new ConditionalCommand(
        new TrackTargetLimelightSetpoint(m_turret, () -> -10 * Math.pow(Math.sin(m_turret.getTurretAngle()*Math.PI/180), 2) * Math.signum(m_turret.getTurretAngle()) ),
        new TrackTargetLimelightSetpoint(m_turret, () -> 10 * Math.pow(Math.sin(m_turret.getTurretAngle()*Math.PI/180), 2)  * Math.signum(m_turret.getTurretAngle()) ),
        () -> m_driverStickY.getAsDouble()<=0),
// TURRET ADJUSTMENT ^^^^

      new ConditionalCommand(
        new DriveArcade(() -> m_driveSpeed,() ->  0, m_drivetrain),
        new DriveArcade(() -> -m_driveSpeed, () -> 0, m_drivetrain),
        () -> (m_driverStickY.getAsDouble() <= 0.0)),

// DRIVE DIRECTION ^^^^
      
      new SequentialCommandGroup(
        new ConditionalCommand(
          new InstantCommand(() -> m_shooter.setShooterSpeed(m_turret.getRequiredVelocity() * (m_driveSpeed/0.43*-0.09*Math.cos(m_turret.getTurretAngle()*Math.PI/180)+1.03) - 100), m_shooter),
          new InstantCommand(() -> m_shooter.setShooterSpeed(m_turret.getRequiredVelocity() * (m_driveSpeed/0.43* 0.09*Math.cos(m_turret.getTurretAngle()*Math.PI/180)+1.03) - 100), m_shooter),
          () -> (m_driverStickY.getAsDouble() <= 0)),
// SHOOTER SPEED ^^^^

        new WaitCommand(0.5),
        new RunLifterLoader(m_lifter, Constants.Lifter.LIFTER_DEFAULT_SPEED, m_loader, Constants.Lifter.LIFTER_DEFAULT_SPEED*10/9),
        new InstantCommand(() -> m_gate.setGate(Constants.Gate.GATE_DEFAULT_SPEED)))
// SHOOTING SEQUENCE ^^^^ 
    );
  }
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0);
    m_lifter.setMotorPower(0);
    m_loader.setMotorPower(0);
  }
}
