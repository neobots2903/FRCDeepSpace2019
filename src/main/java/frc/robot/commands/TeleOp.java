/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Lidar2903.LidarPosition;

/**
 * Super duper TeleOp command
 */
public class TeleOp extends Command {

  boolean lock1 = false;
  boolean lock2 = false;
  boolean lock3 = false;
  boolean lock4 = false;
  boolean lock5 = false;
  boolean lock6 = false;
  boolean lock7 = false;
  boolean lock8 = false;
  boolean lock9 = false;

  public TeleOp() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setTargetMode();
    Robot.dartController.enable();
    Robot.wristController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forward = -Robot.driveJoy.getRawAxis(1);
    double side = Robot.driveJoy.getRawAxis(0);
    double turn = Robot.driveJoy.getRawAxis(4);

    SmartDashboard.putNumber("Turn Speed", turn);

    SmartDashboard.putNumber("Left Distance(mm)", Robot.lidarSubsystem.getDistance(LidarPosition.Left));
    SmartDashboard.putNumber("Left Lidar Status", Robot.lidarSubsystem.getStatus(LidarPosition.Left));
    
    SmartDashboard.putNumber("Center Distance(mm)", Robot.lidarSubsystem.getDistance(LidarPosition.Center));
    SmartDashboard.putNumber("Center Lidar Status", Robot.lidarSubsystem.getStatus(LidarPosition.Center));
    
    SmartDashboard.putNumber("Right Distance(mm)", Robot.lidarSubsystem.getDistance(LidarPosition.Right));
    SmartDashboard.putNumber("Right Lidar Status", Robot.lidarSubsystem.getStatus(LidarPosition.Right));
    
    SmartDashboard.putNumber("Gyro Angle", Robot.navXSubsystem.turnAngle());
    SmartDashboard.putBoolean("Collision Detected", Robot.navXSubsystem.isColliding());

    //SmartDashboard.putNumber("Right Ramp Servo", Robot.rampSubsystem.rightRamp.get());
    //SmartDashboard.putNumber("Left Ramp Servo", Robot.rampSubsystem.leftRamp.get());
    //SmartDashboard.putNumber("Small Ramp Servo", Robot.rampSubsystem.smallRamp.get());
 

    /*
    SmartDashboard.putBoolean("Line left?", Robot.lineSubsystem.leftDetected());
    SmartDashboard.putBoolean("Line center?", Robot.lineSubsystem.centerDetected());
    SmartDashboard.putBoolean("Line right?", Robot.lineSubsystem.rightDetected());
    */

    Robot.driveSubsystem.arcadeDrive(forward, side, turn);

    if (Robot.driveJoy.getRawButton(3)) {
      if (!lock1)
        Robot.rampSubsystem.liftRamp();
      lock1 = true;
    } else {
      lock1 = false;
    } 

    if (Robot.driveJoy.getRawButton(1)) {
      if (!lock2)
        Robot.rampSubsystem.lowerRamp();
        lock2 = true;
    } else {
      lock2 = false;
    }

    if (Robot.driveJoy.getRawButton(4)) {
      if (!lock3)
        Robot.driveSubsystem.mecanumDown();
      lock3 = true;
    } else {
      lock3 = false;
    } 

    if (Robot.driveJoy.getRawButton(2)) {
      if (!lock4)
        Robot.driveSubsystem.tractionDown();
        lock4 = true;
    } else {
      lock4 = false;
    }

    if (Robot.driveJoy.getRawButton(7)) {
      if (!lock5)
        Robot.pickUpArmSubsystem.punch();
        lock5 = true;
    } else {
      lock5 = false;
    }
/*
    if (Robot.driveJoy.getPOV() == 270) {
      Robot.pickUpArmSubsystem.WristSetHa(0.50);
    } else if (Robot.driveJoy.getPOV() == 90) {
      Robot.pickUpArmSubsystem.WristSetHa(-0.40);
    } else {
      Robot.pickUpArmSubsystem.WristSetHa(0.0);
    }*/

    /*
    if (Robot.driveJoy.getPOV() == 180) {
      Robot.pickUpArmSubsystem.ElbowSetHa(-0.45);
    } else if (Robot.driveJoy.getPOV() == 0) {
      Robot.pickUpArmSubsystem.ElbowSetHa(0.75);
    } else {
      Robot.pickUpArmSubsystem.ElbowSetHa(0.0);
    }*/

    if (Robot.opJoy.getRawButton(4)) {
      if (!lock6)
        Robot.pickUpArmSubsystem.goToTop();
        lock6 = true;
    } else {
      lock6 = false;
    }

    if (Robot.opJoy.getRawButton(3)) {
      if (!lock7)
        Robot.pickUpArmSubsystem.goToMiddle();
        lock7 = true;
    } else {
      lock7 = false;
    }

    if (Robot.opJoy.getRawButton(1)) {
      if (!lock8)
        Robot.pickUpArmSubsystem.goToBottom();
        lock8 = true;
    } else {
      lock8 = false;
    }

    if (Robot.opJoy.getRawButton(2)) {
      if (!lock9)
        Robot.pickUpArmSubsystem.goToFloor();
        lock9 = true;
    } else {
      lock9 = false;
    }

    Robot.limelightSubsystem.getTA();
    Robot.limelightSubsystem.getTX();
    Robot.limelightSubsystem.getTS();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
