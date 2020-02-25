/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveTrain;
public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommand.
   */
  private final DriveTrain m_drivetrain; // Instantiating or creating a new object reference
  private final Joystick m_joystick;
  public DriveCommand(DriveTrain subsystem, Joystick joystick) { 
    m_drivetrain = subsystem;
    m_joystick = joystick;
    addRequirements(m_drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sliderValue = -1.0 * m_joystick.getRawAxis(3);
    sliderValue = (sliderValue /2) + 0.5;
    double joyForward = m_joystick.getY();
    double joyTurn = m_joystick.getZ();
    if(Math.abs(joyForward) < 0.05) {
      m_drivetrain.setArcadeDrive(sliderValue * joyForward, joyTurn, true);
    } else {
      m_drivetrain.setArcadeDrive(sliderValue * joyForward, joyTurn, false);
    }

    
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
