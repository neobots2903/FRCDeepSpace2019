/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static int TBD = 0;

  //Talon SRX IDs
  public static final int LeftFrontMotor = 37;
  public static final int RightFrontMotor = 40;
  public static final int LeftRearMotor = 38;
  public static final int RightRearMotor = 31;

  //Controller #'s
  public static final int DriveJoy = 0;
  public static final int OpJoy = 1;

  //DIO Ports
  public static final int LineSensorLeft = 1;
  public static final int LineSensorCenter = 2;
  public static final int LineSensorRight = 3;
  public static final int LineSensorFarLeft = 0;
  public static final int LineSensorFarRight = 4;
}
