/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftyThing;

public class RightLiftyCommand extends CommandBase {
  /**
   * Creates a new RightLiftyCommand.
   */
  private final LiftyThing m_liftyThing;
  private final String upDown;
  public RightLiftyCommand(LiftyThing subsystem, String updown) {
    m_liftyThing = subsystem;
    upDown = updown;
    addRequirements(m_liftyThing);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (upDown == "up"){
      m_liftyThing.rightArmUp();
    }
    if (upDown == "down"){
      m_liftyThing.rightArmDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_liftyThing.rightStopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
