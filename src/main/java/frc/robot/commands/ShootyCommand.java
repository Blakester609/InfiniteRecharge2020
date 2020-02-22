/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootyThing;

public class ShootyCommand extends CommandBase {
  /**
   * Creates a new ShootyCommand.
   */
  public final ShootyThing m_shootyThing;
  public ShootyCommand(ShootyThing subsystem) {
    m_shootyThing = subsystem;
    addRequirements(m_shootyThing);
  }
  boolean isMotorOn = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isMotorOn = !isMotorOn;
    if(isMotorOn) {
      m_shootyThing.shooty();  
    } else {
      m_shootyThing.sucky(0.0);
      m_shootyThing.shootyStop();
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_shootyThing.getShootyEncoderVel());
    if (m_shootyThing.getShootyEncoderVel() >= 19140){
        m_shootyThing.sucky(-1.0); 
        isMotorOn = true;    
    } 
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootyThing.shootyStop();
    m_shootyThing.suckyStop();
    isMotorOn =false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !isMotorOn;
  }
}
