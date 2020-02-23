/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class LiftyThing extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

    public LiftyThing() {
      leftMotor = new CANSparkMax(Constants.Lifty.motor1, MotorType.kBrushless);
      rightMotor = new CANSparkMax(Constants.Lifty.motor2, MotorType.kBrushless);
      leftMotor.follow(rightMotor); //this is so I wouldn't have to set both to the same value
  }
  public void armUp(){
    rightMotor.set(0.5);
  }
  public void armDown(){
    rightMotor.set(-0.5);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
