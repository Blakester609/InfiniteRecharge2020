/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootyThing;

public class VariableShootingSpeedCommand extends CommandBase {
  /**
   * Creates a new VariableShootingSpeedCommand.
   */
  public final ShootyThing m_shootyThing;
  private final XboxController m_xbox;
  private double timer;
  public VariableShootingSpeedCommand(ShootyThing subsystem, XboxController xbox) {
    m_shootyThing = subsystem;
    m_xbox = xbox;
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
    timer++;
    double triggerAxisRight = m_xbox.getRawAxis(3);
    double triggerAxisLeft = m_xbox.getRawAxis(2);
    
    if(triggerAxisRight > 0.1) {
        m_shootyThing.shootyVariable(triggerAxisRight);
        if(triggerAxisLeft > 0.1) {
          m_shootyThing.sucky(-triggerAxisLeft);
        } else {
          m_shootyThing.sucky(0.0);
        }
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
      return false;
    
  }
}
