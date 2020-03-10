/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ShootyThing extends SubsystemBase {
  // This class is full of errors because I don't know what kind of motors the robot has yet. They'll be taken care of later.
  private WPI_TalonFX shootyMotor;
  private VictorSPX suckyMotor;
  private VictorSPX spinnyMotor;
  private NetworkTable table;
  private NetworkTableEntry xOffset;
  private NetworkTableEntry yOffset;
  private NetworkTableEntry area;
  private DigitalInput topSensor;
  private ColorSensorV3 colorSensor;
  private ColorSensorV3 colorSensor2;
  private Color detectedColor;
  private Color detectedColor2;

  public ShootyThing() {
    shootyMotor = new WPI_TalonFX(Constants.Shooty.shootyMotor);
    suckyMotor = new VictorSPX(Constants.Shooty.suckyMotor);
    spinnyMotor = new VictorSPX(Constants.Shooty.spinnyMotor);
    spinnyMotor.setNeutralMode(NeutralMode.Brake);
    topSensor = new DigitalInput(Constants.Shooty.topShooterSensorPort);
    shootyMotor.configFactoryDefault();
    // shootyMotor.setSensorPhase(true);
    
    shootyMotor.setInverted(TalonFXInvertType.Clockwise);
    shootyMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PIDConstantsShooter.pidSlot0, Constants.PIDConstantsShooter.kTimeoutMS);
    // shootyMotor.configVoltageCompSaturation(11);
    // shootyMotor.enableVoltageCompensation(true);
    // shootyMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 15, 1.0));
     // shootyMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 8, 12, 1.0));
    shootyMotor.config_kF(Constants.PIDConstantsShooter.pidSlot0, Constants.PIDConstantsShooter.kFShooter);
    shootyMotor.config_kP(Constants.PIDConstantsShooter.pidSlot0, Constants.PIDConstantsShooter.kPShooter);
    shootyMotor.config_kI(Constants.PIDConstantsShooter.pidSlot0, Constants.PIDConstantsShooter.kIShooter);
    shootyMotor.config_kD(Constants.PIDConstantsShooter.pidSlot0, Constants.PIDConstantsShooter.kDShooter);
    shootyMotor.configNominalOutputForward(0, Constants.PIDConstantsShooter.kTimeoutMS);
		shootyMotor.configNominalOutputReverse(0, Constants.PIDConstantsShooter.kTimeoutMS);
		shootyMotor.configPeakOutputForward(1, Constants.PIDConstantsShooter.kTimeoutMS);
		shootyMotor.configPeakOutputReverse(-1, Constants.PIDConstantsShooter.kTimeoutMS);


     shootyMotor.clearStickyFaults();
    shootyMotor.setNeutralMode(NeutralMode.Brake);
    colorSensor = new ColorSensorV3(Port.kOnboard);
    colorSensor2 = new ColorSensorV3(Port.kMXP);
    
    
    // suckyMotor.configVoltageCompSaturation(10, 20);
  }


  public void shooty(){
    shootyMotor.set(ControlMode.PercentOutput, 0.7);
    //put Limelight stuff in later
    System.out.println(getShootyEncoderVel());
  }

  public void shootyVariable(double speed) {
    shootyMotor.set(ControlMode.PercentOutput, speed);
    System.out.println("shootyMotor value: " + shootyMotor.getSelectedSensorVelocity());
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
      spinnyMotor.set(ControlMode.PercentOutput,0.7);
    }
    else if (leftRight == "left"){
      spinnyMotor.set(ControlMode.PercentOutput, -0.7);
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

  // public boolean isObjectInFrontOfColorSensors() {
  //   if( (detectedColor2.green >= 0.48 && detectedColor2.red >= 0.26) || (detectedColor.green >= 0.49 && detectedColor.red >= 0. && detectedColor.blue >= 0.1)) {
  //     return true;
  //   }
  //   return false;
  // }

  public void getColorFromSensor() {
    detectedColor = colorSensor.getColor();
    detectedColor2 = colorSensor2.getColor();
    boolean isSensorsTriggered = ((detectedColor2.green < 0.5 && detectedColor2.red < 0.3) && (detectedColor.green < 0.5 && detectedColor.red < 0.3));
    boolean isSensorsNotTriggered = (detectedColor2.green >= 0.48 && detectedColor2.red >= 0.26) || (detectedColor.green >= 0.49 && detectedColor.red >= 0. && detectedColor.blue >= 0.1);
    // System.out.println("Color From sensor: "+ detectedColor);
    // System.out.println("Color From sensor2: "+ detectedColor2);
    // System.out.println("Red 2 -> " + detectedColor2.red);
    // System.out.println("Green 2 -> " + detectedColor2.green);
    // System.out.println("Blue 2 -> "+ detectedColor2.blue);

    if(getTopSensorReading()) {
      suckyStop();
    } else if(isSensorsTriggered) {
        suckyStop();
    } else if( isSensorsNotTriggered) {
        sucky(-0.70);
    }
    
  }


  public Color getColorOne() {
    return detectedColor = colorSensor.getColor();
    
  }

  public Color getColorTwo() {
    return detectedColor2 = colorSensor2.getColor();
  }

  public void stopSpinnyMotor() {
    spinnyMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double[] detectInfraredFromSensor() {
    double ir = colorSensor.getIR();
    double ir2 = colorSensor2.getIR();
    System.out.println("Infrared: "+ ir);
    double[] irValues = {ir, ir2};
    return irValues;
  }

  public void startStopSuckyThing(boolean isLoading) {
    if(isLoading) {
      sucky(-0.65);
    } else if(isLoading == false) {
      suckyStop();
    }
  }
  public double getShootyEncoderVel(){
    return shootyMotor.getSelectedSensorVelocity(0);
  }

  public double getShootingMotorSpeed() {
    return shootyMotor.get();
  }
  
  public void setShooty22FeetVelocity() {
    double targetVelocity = 15800;
    shootyMotor.set(TalonFXControlMode.Velocity, targetVelocity);
    System.out.println("Shooting Motor Velocity: " + getShootyEncoderVel());
  }

  public void setShootyCloseRangeVelocity() {
    double targetVelocity = 14500;
    shootyMotor.set(TalonFXControlMode.Velocity, targetVelocity);
    System.out.println("Shooting Motor Velocity: " + getShootyEncoderVel());

  }

  public void setShooty33FeetVelocity() {
    double targetVelocity = 19900;
    shootyMotor.set(TalonFXControlMode.Velocity, targetVelocity);
    System.out.println("Shooting Motor Velocity: " + getShootyEncoderVel());
  }

  @Override
  public void periodic() {
   // getColorFromSensor();
    // This method will be called once per scheduler run
    //printValue();
  }
}
