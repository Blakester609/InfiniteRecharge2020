/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommand.
   */
  
  private final DriveTrain m_drivetrain; // Instantiating or creating a new object reference
  private final XboxController m_xbox;
  public DriveCommand(DriveTrain subsystem, XboxController xbox) { 
    m_drivetrain = subsystem;
    m_xbox = xbox;
    addRequirements(m_drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double sliderValue = -1.0 * m_xbox.getRawAxis(3);
    //sliderValue = (sliderValue /2) + 0.5;
    double joyForward = m_xbox.getRawAxis(Constants.OI.yAxis);
    double joyTurn = m_xbox.getRawAxis(Constants.OI.xAxis);
    if(Math.abs(joyForward) <  0.1 ) {
      joyForward = 0.0;
    }
    if(Math.abs(joyTurn) < 0.1) {
      joyTurn = 0.0;
    }
    m_drivetrain.setArcadeDrive(0.7*joyForward, 0.7 * joyTurn);
    // if(Math.abs(joyForward) < 0.1) {
    //   m_drivetrain.setArcadeDrive(sliderValue * joyForward, joyTurn, true);
    // } else {
    //   m_drivetrain.setArcadeDrive(sliderValue * joyForward, 0.8 * joyTurn, false);
    // }
    
    // 14 feet, 7 inches
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
