/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootyThing;

public class SimpleAutonomousTwoShooty extends CommandBase {
  /**
   * Creates a new SimpleAutonomousTwoShooty.
   */
  private final ShootyThing m_shootyThing;
  double timer;
  public SimpleAutonomousTwoShooty(ShootyThing subsystem) {
    m_shootyThing = subsystem;
    addRequirements(m_shootyThing);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer ++;
    System.out.println(timer);
    if (timer <= 100){
      m_shootyThing.shooty();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootyThing.shootyStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer >= 101;
  }
}
