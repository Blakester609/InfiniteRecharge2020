/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;
  private WPI_TalonFX motor3;
  private WPI_TalonFX motor4;
  private DifferentialDrive diffDrive;
  private NetworkTable table;
  private NetworkTableEntry xOffset;
  private NetworkTableEntry yOffset;
  private NetworkTableEntry area; 
  private NetworkTableEntry validTarget;
  private NetworkTableEntry skew;
  private NetworkTableEntry tl;
  public DriveTrain() {
    motor1 = new WPI_TalonFX(Constants.Drive.motor1);
    motor2 = new WPI_TalonFX(Constants.Drive.motor2);
    motor3 = new WPI_TalonFX(Constants.Drive.motor3);
    motor4 = new WPI_TalonFX(Constants.Drive.motor4);
    motor3.follow(motor1);
    motor4.follow(motor2);
    diffDrive = new DifferentialDrive(motor1, motor2);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = table.getEntry("tx");
    yOffset = table.getEntry("ty");
    area = table.getEntry("area");
    validTarget = table.getEntry("tv"); //Whether the limelight has a valid target (either 0 or 1)
    skew = table.getEntry("ts"); //Skew or rotation of target (-90.0 to 0.0 deg.)
    tl = table.getEntry("tl");
   }

   public static class LLData {
    public final double xOffset, yOffset, area, targetExists, skew;

    public LLData(double xOffset, double yOffset, double area, double targetExists, double skew) {
        this.xOffset = xOffset;    
        this.yOffset = yOffset;
        this.area = area;
        this.targetExists = targetExists;     
        this.skew = skew;
      }
  }

  public double getHeadingError() {
    var limelightData = this.getData(); //Java 10 'var' automatically creates new LLData object.
 
    double minDrive = Constants.Limelight.minTurnPower; //speed the motor will move the robot regardless of how miniscule the error is
    double kP = Constants.Limelight.kpAim; //constant for turn power
    double xOffset = limelightData.xOffset;
    double heading = 0.0; //should be opposite of offset (in signs)

    if (xOffset > 1.0) {
        heading = ((kP * xOffset) + minDrive);
        System.out.println("GOING LEFT");
    } else { //xOffset less than or equal to 1.0
        heading = ((kP * xOffset) - minDrive);
        System.out.println("GOING RIGHT");
    }

    return heading;
}

public void aimTowardsTarget(double speed) {
  diffDrive.arcadeDrive(speed, getHeadingError());
}

  public LLData getData() {
    double x = this.area.getDouble(0.0);
    double y = this.yOffset.getDouble(0.0);
    double area = this.area.getDouble(0.0);
    double skew = this.skew.getDouble(0.0);
    double v = this.validTarget.getDouble(0.0);
    return new DriveTrain.LLData(x, y, area, v, skew);
  }
  public double estimatingDistance(){
    double distance;
    distance = (Constants.Limelight.targetHeight - Constants.Limelight.cameraHeight)/(Math.tan(Constants.Limelight.cameraAngle+this.getData().yOffset));
    return distance;
  }

  public void setArcadeDrive(double joyForward, double joyTurn){
       diffDrive.arcadeDrive(-joyForward, joyTurn);
  }
  public void aimingInRange(){
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
