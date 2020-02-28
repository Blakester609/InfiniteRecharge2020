/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  double rampRate = 0.0;
  double suckyMotorSpeed = 0.0;
  double timer = 0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    suckyMotorSpeed = 0.2;
    isMotorOn = !isMotorOn;
    if(isMotorOn) {
      m_shootyThing.shooty();
     m_shootyThing.sucky(-suckyMotorSpeed);  
    } else {
      m_shootyThing.sucky(0.0);
      m_shootyThing.shootyStop();
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    timer++;
    suckyMotorSpeed += 0.025;
    // if(((m_shootyThing.getColorTwo().green < 0.5 && m_shootyThing.getColorTwo().red < 0.3) && (m_shootyThing.getColorOne().green < 0.5 && m_shootyThing.getColorOne().red < 0.3))) {
    //   rampRate += 0.02;
    //   m_shootyThing.sucky(suckyMotorSpeed + rampRate);
    // } 
    // if( (m_shootyThing.getColorTwo().green >= 0.48 && m_shootyThing.getColorTwo().red >= 0.26) || (m_shootyThing.getColorOne().green >= 0.49 && m_shootyThing.getColorOne().red >= 0. && m_shootyThing.getColorOne().blue >= 0.1)
    //   ) {
        System.out.println(m_shootyThing.getShootyEncoderVel());
        if (m_shootyThing.getShootyEncoderVel() >= 14000 && timer >= 70){
          m_shootyThing.sucky(-suckyMotorSpeed); 
          isMotorOn = true;    
        } 
      // }
      
      
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootyThing.shootyStop();
    m_shootyThing.suckyStop();
    isMotorOn =false;
    timer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !isMotorOn;
  }
}
