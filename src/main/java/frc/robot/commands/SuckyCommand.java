/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootyThing;

public class SuckyCommand extends CommandBase {
  /**
   * Creates a new SuckyCommand.
   */
  public final ShootyThing m_shootyThing;
  public SuckyCommand(ShootyThing subsystem) {
    m_shootyThing = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shootyThing);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shootyThing.sucky();
    //There will also be stuff with sensing how many balls there are in the robot, but I don't know how to do that yet. Maybe. 
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
