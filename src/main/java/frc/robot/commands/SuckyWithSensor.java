/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootyThing;

public class SuckyWithSensor extends CommandBase {
  /**
   * Creates a new SuckyWithSensor.
   */
  private final ShootyThing m_shootyThing;
  public SuckyWithSensor(ShootyThing subsystem) {
    m_shootyThing = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shootyThing);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if((m_shootyThing.detectInfraredFromSensor()[0] <= 9.0 && m_shootyThing.detectInfraredFromSensor()[1] <= 9.0) || m_shootyThing.getTopSensorReading()) {
    //   m_shootyThing.suckyStop();
    // } else
    // if(m_shootyThing.detectInfraredFromSensor()[0] >= 10.0 || m_shootyThing.detectInfraredFromSensor()[1] >= 10.0) {
    //   m_shootyThing.sucky(-0.6);
    // } 
    m_shootyThing.getColorFromSensor();
     
    
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
