// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hanger;
import static frc.robot.Constants.Hanger.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IncrementHangerUp_2 extends SequentialCommandGroup {

  private Hanger m_hanger;
  private double m_currentposition;
  /** Creates a new IncrementHangerUp_2. */
  public IncrementHangerUp_2(double startingPosition, Hanger hanger) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new RaiseHangerToHeight(STEP_1, hanger), //if below STEP_1-10, up to step 1
        new InstantCommand(() -> {}), 
        ()->startingPosition < STEP_1 -10),    
      new ConditionalCommand(         //***** add 'pull in hanger' here
        new RaiseHangerToHeight(STEP_2, hanger), //if between STEP_1-10 and STEP_2-10, up to step 2
        new InstantCommand(() -> {}), 
        ()-> startingPosition < STEP_2 -10 && startingPosition > STEP_1 -10)    //up to step 2
     // new ConditionalCommand(
        //new RaiseHangerToHeight(STEP_2, hanger), 
        //new InstantCommand(() -> {}), 
       // ()-> startingPosition < STEP_30 && startingPosition > STEP_1 -10))
        
       
       //new RaiseHangerToHeight(STEP_2, hanger), //if between STEP_1-10 and STEP_3, up to step 3
        //new InstantCommand(() -> {}), 
        //()-> startingPosition < STEP_30 && startingPosition > STEP_1 -10),    //up to step 2


    );
  }
}
