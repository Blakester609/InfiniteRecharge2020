/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.AimingCommandTwo;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStraightAutoCommand;
import frc.robot.commands.LeftLiftyCommand;

import frc.robot.commands.RightLiftyCommand;
import frc.robot.commands.SetLauncherVelicityFarTrench;
import frc.robot.commands.ShootCloseRange;
import frc.robot.commands.ShootyCommand;
import frc.robot.commands.SimpleAuton;
import frc.robot.commands.SimpleAutonomousTwo;
import frc.robot.commands.SimpleAutonomousTwoShooty;
import frc.robot.commands.SpinnyCommand;
import frc.robot.commands.SuckyCommand;
import frc.robot.commands.SuckyWithSensor;
import frc.robot.commands.TopSensor;
import frc.robot.commands.VariableShootingSpeedCommand;
import frc.robot.controller.XboxButton;
import frc.robot.controller.XboxButton.Button;
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
  private final XboxController driveController = new XboxController(Constants.OI.XPort);
  private final Joystick m_climbingJoystick = new Joystick(Constants.OI.climbingJoystick);
  private final ShootyThing m_shootyThing = new ShootyThing();

  private final DriveTrain m_driveTrain = new DriveTrain();
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrain, driveController); 
  private final SimpleAuton m_autoCommand = new SimpleAuton(m_driveTrain, m_shootyThing);
  private final SimpleAutonomousTwo m_basicAuto = new SimpleAutonomousTwo(m_driveTrain, m_shootyThing);
  private final SimpleAutonomousTwoShooty m_shootyAuto = new SimpleAutonomousTwoShooty(m_shootyThing);
  private final DriveStraightAutoCommand m_driveStraightAutoCommand = new DriveStraightAutoCommand(m_driveTrain);
  private final ShootyCommand m_shootyCommand = new ShootyCommand(m_shootyThing);
  // private final SuckyCommand m_suckyCommand = new SuckyCommand(m_shootyThing);
  private final SuckyWithSensor m_suckyWithSensorCommand = new SuckyWithSensor(m_shootyThing);
  private final SpinnyCommand m_leftspinnyCommand = new SpinnyCommand(m_shootyThing, "left");
  private final SpinnyCommand m_rightspinnyCommand = new SpinnyCommand(m_shootyThing, "right");
  private final TopSensor m_topSensor = new TopSensor(m_shootyThing);
  private final LiftyThing m_liftyThing = new LiftyThing();
  private final VariableShootingSpeedCommand m_variableShootingSpeedCommand= new VariableShootingSpeedCommand(m_shootyThing, driveController);
  private final SetLauncherVelicityFarTrench m_setLauncherVelicityFarTrench = new SetLauncherVelicityFarTrench(m_shootyThing, driveController);

  private final RightLiftyCommand m_upRightLiftyCommand = new RightLiftyCommand(m_liftyThing, "up");
  private final RightLiftyCommand m_downRightLiftyCommand = new RightLiftyCommand(m_liftyThing, "down");
  private final LeftLiftyCommand m_downLeftLiftyCommand = new LeftLiftyCommand(m_liftyThing, "down");
  private final LeftLiftyCommand m_upLeftLiftyCommand = new LeftLiftyCommand(m_liftyThing, "up"); 
  private final AimingCommandTwo m_aiming = new AimingCommandTwo(m_driveTrain);
  private final ShootCloseRange m_shootCloseRange = new ShootCloseRange(m_shootyThing, driveController);
  //Box where autonomous selection occurs is created here.
  private SendableChooser<String> autonomousChooser = new SendableChooser<>();
  Command autoCommand = m_basicAuto;
  String selectedAuto;
  private final String inFrontOfPowerGenerator = "Power Generator Auto";
   private final String inFrontOfCargoBay = "Cargo Bay Auto";
   private final String nearSide = "Near Side Auto";
   private final String driveStraight = "Drive Straight Auto";
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_driveTrain, m_driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_shootyThing, m_suckyWithSensorCommand);
    // CommandScheduler.getInstance().setDefaultCommand(m_shootyThing, m_variableShootingSpeedCommand);
    // Configure the button bindings
    selectedAuto = "";
   //Allows you to select an autonomous routine from Shuffleboard
   autonomousChooser.addOption("Power Generator Auto", inFrontOfPowerGenerator);
   autonomousChooser.addOption("Cargo Bay Auto", inFrontOfCargoBay);
   autonomousChooser.addOption("Near Side Auto", nearSide);
   autonomousChooser.addOption("Drive Straight Auto", driveStraight);
   autonomousChooser.setDefaultOption("Power Generator Auto", inFrontOfPowerGenerator);
   SmartDashboard.putData(autonomousChooser);
   

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final XboxButton shootyButton;
    // final JoystickButton suckyButton;
    final XboxButton spinnyClockButton;
    final XboxButton variableShootyButton;
    final XboxButton spinnyCounterclockButton;
    final JoystickButton clawOneOnButton; 
    final JoystickButton clawTwoOnButton; 
    final JoystickButton liftPistonOnButton;
    final XboxButton frontGateOnButton;
    final XboxButton backGateOnButton;
    final JoystickButton upLeftLiftyButton;
    final JoystickButton downLeftLiftyButton;
    final JoystickButton upRightLiftyButton;
    final JoystickButton downRightLiftyButton;
    final XboxButton aimingButton;
    shootyButton = new XboxButton(driveController, Button.Y);
   //  suckyButton = new JoystickButton(m_joystick, 2);
    spinnyClockButton = new XboxButton(driveController, Button.B);
    spinnyCounterclockButton = new XboxButton(driveController, Button.X);
    clawOneOnButton = new JoystickButton(m_climbingJoystick, 5);
    clawTwoOnButton = new JoystickButton(m_climbingJoystick, 4);
    liftPistonOnButton = new JoystickButton(m_climbingJoystick, 1);
    //frontGateOnButton = new XboxButton(driveController, Button.BumperRight);
    // backGateOnButton = new XboxButton(driveController, Button.BumperLeft);
    upLeftLiftyButton = new JoystickButton(m_climbingJoystick, 3);
    // upRightLiftyButton = new JoystickButton(m_climbingJoystick, 8);
    downLeftLiftyButton = new JoystickButton(m_climbingJoystick, 2);
    // downRightLiftyButton = new JoystickButton(m_climbingJoystick, 9);
    aimingButton = new XboxButton(driveController, Button.A);
    variableShootyButton = new XboxButton(driveController, Button.Start);
    // shootyButton.toggleWhenPressed(m_shootyCommand);
   // suckyButton.toggleWhenPressed(m_suckyCommand);
    spinnyClockButton.whileHeld(m_rightspinnyCommand);
    variableShootyButton.whileHeld(m_variableShootingSpeedCommand);
    // Kira's amazing code
    // shpoofy 
    // downLeftLiftyButton.bootlegging@gmail.command
    // Happiness.bananas; #forbreakfast:/Fix
    shootyButton.whileHeld(m_setLauncherVelicityFarTrench);
    spinnyCounterclockButton.whileHeld(m_leftspinnyCommand);
    //The next three lines allow you to create an instant command, or a command that would only take up one line in a command interface. Convenience feature.
    clawOneOnButton.whenPressed(new InstantCommand(m_liftyThing::clawOneSolenoidOn, m_liftyThing));
    clawTwoOnButton.whenPressed(new InstantCommand(m_liftyThing::clawTwoSolenoidOn, m_liftyThing));
    liftPistonOnButton.whenPressed(new InstantCommand(m_liftyThing::setLiftStopPiston, m_liftyThing));

    // frontGateOnButton.whenPressed(new InstantCommand(m_driveTrain::frontGateSolenoidOn, m_driveTrain));
    //frontGateOnButton.whileHeld(m_shootCloseRange);
    // backGateOnButton.whenPressed(new InstantCommand(m_driveTrain::backGateSolenoidOn, m_driveTrain));
    upLeftLiftyButton.whileHeld(m_upLeftLiftyCommand);
    // upRightLiftyButton.whileHeld(m_upRightLiftyCommand);
    // downRightLiftyButton.whileHeld(m_downRightLiftyCommand);
    downLeftLiftyButton.whileHeld(m_downLeftLiftyCommand);
    aimingButton.toggleWhenPressed(m_aiming);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //This is where the actual logic involving autonomous selection on Shuffleboard occurs.
 public Command getAutonomousCommand() {
  selectedAuto = autonomousChooser.getSelected();
  switch (selectedAuto) {
     case inFrontOfPowerGenerator:
       autoCommand = m_basicAuto;
       break;
     case driveStraight:
       autoCommand = m_driveStraightAutoCommand;
       break;
      case nearSide:
        break;
      case inFrontOfCargoBay:
        break;
     default:
       break;
  }
   
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}

