/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LineSensor2903 extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DigitalInput farLeftLine;
  DigitalInput leftLine;
  DigitalInput centerLine;
  DigitalInput rightLine;
  DigitalInput farRightLine;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void init() {
    /*
    farLeftLine = new DigitalInput(RobotMap.LineSensorFarLeft);
    leftLine = new DigitalInput(RobotMap.LineSensorLeft);
    centerLine = new DigitalInput(RobotMap.LineSensorCenter);
    rightLine = new DigitalInput(RobotMap.LineSensorRight);
    farRightLine = new DigitalInput(RobotMap.LineSensorFarRight);
    */
  }

  public double MoveToCenter() {
    if (farLeftLine.get()) return -0.6;
    else if (farRightLine.get()) return 0.6;
    else if (leftLine.get()) return -0.3;
    else if (rightLine.get()) return 0.3;
    else if (centerLine.get()) return 0;
    else return 0;
  }

public String detectionString() {
  StringBuilder result = new StringBuilder();

  result.append(boolToInt(farLeftLine.get()));
  result.append(boolToInt(leftLine.get()));
  result.append(boolToInt(centerLine.get()));
  result.append(boolToInt(rightLine.get()));
  result.append(boolToInt(farRightLine.get()));

  return result.toString();
}

public int boolToInt(boolean val){
  if (val) 
    return 1;
  else
    return 0;
}


  public boolean leftDetected() {
    return !leftLine.get();
  }

  public boolean centerDetected() {
    return !centerLine.get();
  }

  public boolean rightDetected() {
    return !rightLine.get();
  }
}
