/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ShootyThing extends SubsystemBase {
  // This class is full of errors because I don't know what kind of motors the robot has yet. They'll be taken care of later.
  public WPI_TalonFX shootyMotor;
  public VictorSPX suckyMotor;
  public VictorSPX spinnyMotor;
  public NetworkTable table;
  public NetworkTableEntry xOffset;
  public NetworkTableEntry yOffset;
  public NetworkTableEntry area;
  public DigitalInput topSensor;
  public ShootyThing() {
    shootyMotor = new WPI_TalonFX(Constants.Shooty.shootyMotor);
    suckyMotor = new VictorSPX(Constants.Shooty.suckyMotor);
    spinnyMotor = new VictorSPX(Constants.Shooty.spinnyMotor);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = table.getEntry("tx");
    yOffset = table.getEntry("ty");
    area = table.getEntry("area");
    topSensor = new DigitalInput(Constants.Shooty.topShooterSensorPort);
    shootyMotor.configFactoryDefault();
    shootyMotor.setSensorPhase(false);
    shootyMotor.setInverted(TalonFXInvertType.Clockwise);
    shootyMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
  }
  public void shooty(){
    shootyMotor.set(1.0);
    //put Limelight stuff in later
  }
  public boolean getTopSensorReading() {
    return topSensor.get();
  }
  public void sucky(double motorSpeed){
    suckyMotor.set(ControlMode.PercentOutput ,motorSpeed);
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
  public double getSuckyMotorValue() {
    return suckyMotor.getMotorOutputPercent();
  }

  public void shootyStop(){
    shootyMotor.stopMotor();
  }
  public void suckyStop(){
    suckyMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public void printValue(){
    System.out.println(topSensor.get());
  }

  public void startStopSuckyThing(boolean isLoading) {
    if(isLoading) {
      sucky(-0.7);
    } else if(isLoading == false) {
      suckyStop();
    }
  }
  public double getShootyEncoderVel(){
    return shootyMotor.getSelectedSensorVelocity(0);
  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    //printValue();
  }
}
