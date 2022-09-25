// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanger;

// Allows for control of hanger
public class ControlHanger extends CommandBase {

  private final DoubleSupplier m_speedSupplier, m_sliderAxis3Supplier;
  Boolean OKtoHang;
  private final Hanger m_hanger;

  /** Creates a new HangerControl. */
  public ControlHanger(DoubleSupplier speedSupplier, DoubleSupplier sliderAxis3Supplier, Hanger hanger) {

    m_speedSupplier = speedSupplier;
    m_sliderAxis3Supplier = sliderAxis3Supplier;    //indicates whether in 'hanging' mode
    m_hanger = hanger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    OKtoHang = m_sliderAxis3Supplier.getAsDouble() < -0.8;  //slider up to hang
    if(OKtoHang){
    m_hanger.set(m_speedSupplier.getAsDouble());
    // SmartDashboard.putNumber("Hanger Set V", m_speedSupplier.getAsDouble());
    }
    else{
      m_hanger.set(0);
    }
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
