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
  private Solenoid clawOne;
  private Solenoid clawTwo;
  private Solenoid liftStopPiston;

  private boolean clawOneOn = false;
  private boolean clawTwoOn = false;
  private boolean liftStopPistonOn = false;
    public LiftyThing() {
      // leftMotor = new CANSparkMax(Constants.Lifty.motor1, MotorType.kBrushless);
      // rightMotor = new CANSparkMax(Constants.Lifty.motor2, MotorType.kBrushless);
      // leftMotor.follow(rightMotor); 
      //this is so I wouldn't have to set both to the same value
      clawOne = new Solenoid(Constants.pcmChannel, Constants.Lifty.clawOne);
      clawTwo = new Solenoid(Constants.pcmChannel, Constants.Lifty.clawTwo);
      liftStopPiston = new Solenoid(Constants.pcmChannel,Constants.Lifty.liftStopPiston);

      clawOne.set(false);
      clawTwo.set(false);
      liftStopPiston.set(false);
  }
  public void armUp(){
    rightMotor.set(0.5);
  }
  public void armDown(){
    rightMotor.set(-0.5);
  }

  public void clawOneSolenoidOn() {
    System.out.println("changed claw solenoid state");
    clawOneOn = !clawOneOn;
    clawOne.set(clawOneOn);
  }

  public void clawTwoSolenoidOn(){
    clawTwoOn = !clawTwoOn;
    clawTwo.set(clawTwoOn);
  }

  public void setLiftStopPiston() {
    liftStopPistonOn = !liftStopPistonOn;
    liftStopPiston.set(liftStopPistonOn);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
