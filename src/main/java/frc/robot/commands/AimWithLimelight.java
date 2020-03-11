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
//This command is used to aim the DRIVE TRAIN at the target that Limelight detects.
public class AimWithLimelight extends CommandBase {
  /**
   * Creates a new AimWithLimelight.
   */
  private DriveTrain m_driveTrain;
  private double driveAdjustment;
  private double headingError;
  // private double delay = Math.ceil(RobotSettings.LL_DELAY * 50.0); //converts seconds to iterative values (1 sec = 50)
  private double currentTimer = 0; //current timer
  public AimWithLimelight(DriveTrain subsystem) {
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveAdjustment = 0;
    headingError = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.getData().targetExists != 0.0) {
      driveAdjustment =m_driveTrain.estimatingDistance();
      m_driveTrain.aimingInRange(driveAdjustment, headingError);
    }
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((driveAdjustment - m_driveTrain.getData().yOffset) == 0) {
      System.out.println("aiming command finished");
      return true;
    } else {
      return false;
    }
    
  }
}
