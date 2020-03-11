/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//This is a failed class because the Trajectory class is made for DifferentialDrive robots.
package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShootyThing;

public class SimpleAuton extends CommandBase {
  /**
   * Creates a new SimpleAuton.
   */
  private final DriveTrain m_driveTrain;
  private final ShootyThing m_shootyThing;
  public SimpleAuton(DriveTrain subsystem, ShootyThing subsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = subsystem;
    m_shootyThing = subsystem2;
    addRequirements(m_driveTrain);
    addRequirements(m_shootyThing);
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.PIDConstantsDrivetrain.ksVolts, 
        Constants.PIDConstantsDrivetrain.ksVoltSecondsPerMeter,
        Constants.PIDConstantsDrivetrain.kaVoltSecondsSquaredPerMeter),
        Constants.PIDConstantsDrivetrain.kDriveKinematics,
        10);
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.PIDConstantsDrivetrain.kMaxSpeedFeetPerSecond, 
      Constants.PIDConstantsDrivetrain.kMaxAccelerationFeetPerSecond) 
      .setKinematics(Constants.PIDConstantsDrivetrain.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.autonomousConfig();
    String trajectoryJSON = "paths/pathweaver.wpilib.json";
    Trajectory trajectory;
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
