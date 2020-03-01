/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class AimingCommandTwo extends CommandBase {
  /**
   * Creates a new AimingCommandTwo.
   */
  private DriveTrain m_driveTrain;
  double KpAim = Constants.Limelight.kpAim;
  double KpDistance = Constants.Limelight.kpDistance;
  double min_aim_command = Constants.Limelight.minTurnPower;
  double steeringAdjustment;
  double distanceAdjust;
  public AimingCommandTwo(DriveTrain subsystem) {
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    steeringAdjustment = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingError = -1.0 * m_driveTrain.getData().xOffset;
    System.out.println("X offset: " + headingError);
    double distanceError = -1.0 * m_driveTrain.getData().yOffset;
    System.out.println("Y offset: " + distanceError);
    if (m_driveTrain.getData().xOffset > 1.0)
    {
      steeringAdjustment = KpAim*headingError - min_aim_command;
    }
    else if (m_driveTrain.getData().xOffset < 1.0)
    {
      steeringAdjustment = KpAim*headingError + min_aim_command;
    }
    distanceAdjust = KpDistance*distanceError;
    m_driveTrain.setArcadeDriveTwo(distanceAdjust, steeringAdjustment);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(steeringAdjustment) <= 0.05 && Math.abs(distanceAdjust) <= 0.05) {
      return true;
    }
    return false;
  }
}
