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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LiftyCommand;
import frc.robot.commands.ShootyCommand;
import frc.robot.commands.SimpleAuton;
import frc.robot.commands.SpinnyCommand;
import frc.robot.commands.SuckyCommand;
import frc.robot.commands.SuckyWithSensor;
import frc.robot.commands.TopSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LiftyThing;
import frc.robot.subsystems.ShootyThing;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick m_joystick = new Joystick(Constants.OI.joyPort);
  
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrain ,m_joystick);
  private final SimpleAuton m_autoCommand = new SimpleAuton(m_driveTrain);
  
  private final ShootyThing m_shootyThing = new ShootyThing();
  private final ShootyCommand m_shootyCommand = new ShootyCommand(m_shootyThing);
  private final SuckyCommand m_suckyCommand = new SuckyCommand(m_shootyThing);
  private final SuckyWithSensor m_suckyWithSensorCommand = new SuckyWithSensor(m_shootyThing);
  private final SpinnyCommand m_leftspinnyCommand = new SpinnyCommand(m_shootyThing, "left");
  private final SpinnyCommand m_rightspinnyCommand = new SpinnyCommand(m_shootyThing, "right");
  private final TopSensor m_topSensor = new TopSensor(m_shootyThing);
  private final LiftyThing m_liftyThing = new LiftyThing();

  private final LiftyCommand m_upLiftyCommand = new LiftyCommand(m_liftyThing, "up");
  private final LiftyCommand m_downLiftyCommand = new LiftyCommand(m_liftyThing, "down");

   
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_driveTrain, m_driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_shootyThing, m_suckyWithSensorCommand);
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
    final JoystickButton shootyButton;
    final JoystickButton suckyButton;
    final JoystickButton spinnyClockButton;
    final JoystickButton spinnyCounterclockButton;
    final JoystickButton liftyUpButton;
    final JoystickButton liftyDownButton;  
    final JoystickButton clawOneOnButton; 
    final JoystickButton clawTwoOnButton; 
    final JoystickButton liftPistonOnButton;
    final JoystickButton frontGateOnButton;
    final JoystickButton backGateOnButton;

    shootyButton = new JoystickButton(m_joystick, 1);
    suckyButton = new JoystickButton(m_joystick, 2);
    spinnyClockButton = new JoystickButton(m_joystick, 6);
    spinnyCounterclockButton = new JoystickButton(m_joystick, 5);
    liftyUpButton = new JoystickButton(m_joystick, 8);
    liftyDownButton = new JoystickButton(m_joystick, 7);
    clawOneOnButton = new JoystickButton(m_joystick, 3);
    clawTwoOnButton = new JoystickButton(m_joystick, 4);
    liftPistonOnButton = new JoystickButton(m_joystick, 9);
    frontGateOnButton = new JoystickButton(m_joystick, 11);
    backGateOnButton = new JoystickButton(m_joystick, 12);


    shootyButton.toggleWhenPressed(m_shootyCommand);
    suckyButton.toggleWhenPressed(m_suckyCommand);
    spinnyClockButton.whileHeld(m_rightspinnyCommand);
    spinnyCounterclockButton.whileHeld(m_leftspinnyCommand);
    liftyUpButton.whileHeld(m_upLiftyCommand);
    liftyDownButton.whileHeld(m_downLiftyCommand);
    clawOneOnButton.whenPressed(new InstantCommand(m_liftyThing::clawOneSolenoidOn, m_liftyThing));
    clawTwoOnButton.whenPressed(new InstantCommand(m_liftyThing::clawTwoSolenoidOn, m_liftyThing));
    liftPistonOnButton.whenPressed(new InstantCommand(m_liftyThing::setLiftStopPiston, m_liftyThing));

    frontGateOnButton.whenPressed(new InstantCommand(m_driveTrain::frontGateSolenoidOn, m_driveTrain));
    backGateOnButton.whenPressed(new InstantCommand(m_driveTrain::backGateSolenoidOn, m_driveTrain));
    


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}

