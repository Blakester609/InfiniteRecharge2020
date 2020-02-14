/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ControlPanelColorSensingCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SimpleAuton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick m_joystick = new Joystick(Constants.OI.joyPort);
  // private final DriveTrain sillyDriveTrainThing = new DriveTrain();
  // private final DriveCommand m_driveCommand = new DriveCommand(sillyDriveTrainThing ,m_joystick);

  private final Vision cameraVision = new Vision();
  private final ControlPanelColorSensingCommand colorSensingCommand = new ControlPanelColorSensingCommand(cameraVision);
  // sillyDriveTrainThing = DriveTrain()
  //subsystem = sillyDriveTrainThing
  //m_drivetrain = subsystem = sillyDriveTrainThing
  /// m_drivetrain = sillyDriveTrainThing


  // private final SimpleAuton m_autoCommand = new SimpleAuton(sillyDriveTrainThing);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
   // CommandScheduler.getInstance().setDefaultCommand(sillyDriveTrainThing, m_driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(cameraVision, colorSensingCommand);
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return null;
  }
}

