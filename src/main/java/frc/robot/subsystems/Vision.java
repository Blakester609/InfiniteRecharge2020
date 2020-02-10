/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  SerialPort pixyPort;
  public Vision() {
    pixyPort = new SerialPort(115200, SerialPort.Port.kUSB);
    pixyPort.setReadBufferSize(1);
  }

  // public byte[] getPixyData() {
  //   return pixyPort.read(50);
  // }

  // public String getPixyBufferString() {
  //   return pixyPort.readString();
  // }

  public SerialPort getPixyPort() {
    return pixyPort;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}