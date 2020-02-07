/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drive {
        public static final int motor1victor = 1;
        public static final int motor2victor = 2;
        public static final int motor3victor = 3;
        public static final int motor4dmc60 = 4;
    }
    public static final class OI {
        public static final int joyPort = 0;
        public static final int xAxis = 0;
        public static final int yAxis = 1;
    }
    public static final class Lifty {
        public static final int motor1 = 0;
        public static final int motor2 = 0;
    }
    public static final class SpinnyShooty {
        public static final int shootyMotor = 0;
        public static final int suckyMotor = 0;
        public static final int spinnyMotor = 0;
    }
}