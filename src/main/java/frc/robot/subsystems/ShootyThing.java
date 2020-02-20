/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ShootyThing extends SubsystemBase {
  //This class is full of errors because I don't know what kind of motors the robot has yet. They'll be taken care of later.
  private WPI_TalonFX shootyMotor;
  private VictorSPX suckyMotor;
  private VictorSPX spinnyMotor;
  private NetworkTable table;
  private NetworkTableEntry xOffset;
  private NetworkTableEntry yOffset;
  private NetworkTableEntry area;
  public ShootyThing() {
    shootyMotor = new WPI_TalonFX(Constants.Shooty.shootyMotor);
    suckyMotor = new VictorSPX(Constants.Shooty.suckyMotor);
    spinnyMotor = new VictorSPX(Constants.Shooty.spinnyMotor);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = table.getEntry("tx");
    yOffset = table.getEntry("ty");
    area = table.getEntry("area");
  }
  public void shooty(){
    shootyMotor.set(0.4);
    //put Limelight stuff in later
  }
  public void sucky(){
    suckyMotor.set(ControlMode.PercentOutput ,0.8);
    //put sensor stuff in later
  }
  public void spinny(String leftRight){
    if (leftRight == "right"){
      spinnyMotor.set(ControlMode.PercentOutput,0.5);
    }
    else if (leftRight == "left"){
      spinnyMotor.set(ControlMode.PercentOutput, -0.5);
    }  
    //Set actual motor values later
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
