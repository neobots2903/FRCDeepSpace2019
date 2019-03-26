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
import frc.robot.subsystems.PickUpArm2903.ArmState;
import frc.robot.subsystems.Drive2903.DriveState;

/**
 * Super duper TeleOp command
 */
public class TeleOp extends Command {

  boolean RampLiftLock = false;
  boolean RampLowerLock = false;
  boolean MecanumLock = false;
  boolean TractionLock = false;
  boolean PunchLock = false;
  boolean ArmTopLock = false;
  boolean ArmMiddleLock = false;
  boolean ArmBottomLock = false;
  boolean ArmFloorLock = false;
  boolean ArmConfinedLock = false;
  boolean autoPositionLock = false;

  boolean autoAligned = false;
  int targetArea = 35;
  int lidarTarget = 69;

  boolean HandLock = false;
  boolean HandState = false;

  boolean AutoAimLock = false;
  boolean AutoAimState = false;

  public TeleOp() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.limelightSubsystem.setTargetMode();
    Robot.limelightSubsystem.setLight(false);

    Robot.dartController.setSetpoint(Robot.pickUpArmSubsystem.getElbow());
    Robot.wristController.setSetpoint(Robot.pickUpArmSubsystem.getWrist());
    
    Robot.dartController.enable();
    Robot.wristController.enable();

    Robot.rampSubsystem.closeRamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forward = -Robot.driveJoy.getRawAxis(1);
    double side = Robot.driveJoy.getRawAxis(0);
    double turn = Robot.driveJoy.getRawAxis(4);

    if(Math.abs(turn) > 0.9) {
      Robot.driveSubsystem.setFront(DriveState.Traction);
      Robot.driveSubsystem.setRear(DriveState.Mecanum);
    } else {
      Robot.driveSubsystem.setAll(Robot.driveSubsystem.getDriveState());
    }

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

    if (Robot.driveJoy.getRawButton(4)) {
      if (!AutoAimLock)
          AutoAimState = !AutoAimState;
        AutoAimLock = true;
    } else {
      AutoAimLock = false;
    }

    if (AutoAimState) {
      Robot.limelightSubsystem.setTargetMode();
      Robot.visionStrafeController.enable();//Autonomous awarddddddddddddd
      turn = Robot.visionStrafeValue;

      double tx = Robot.limelightSubsystem.getTX();
      double ta = Robot.limelightSubsystem.getTA();
      if(Math.abs(tx) < 5) {  //if within turn error
        if (Robot.lidarSubsystem.getStatus(LidarPosition.Left) == 0 && 
        Robot.lidarSubsystem.getStatus(LidarPosition.Right) == 0) { //if lidar working
          if (Robot.lidarSubsystem.getDistance(LidarPosition.Left) < lidarTarget || 
          Robot.lidarSubsystem.getDistance(LidarPosition.Right) < lidarTarget) { //if too far away
            Robot.driveSubsystem.tankDrive( //drive closer to target
              percentToTarget(Robot.lidarSubsystem.getDistance(LidarPosition.Left), lidarTarget),
              percentToTarget(Robot.lidarSubsystem.getDistance(LidarPosition.Right), lidarTarget)
            );
          } else {  //if close enough to target
          autoAligned = true;
          }
        } else if (Robot.limelightSubsystem.getTA() < targetArea) {  //if lidar values are unreliable, use vision
          forward = (Math.abs(ta) > targetArea) ? 0 : percentToTarget(ta,targetArea)/3;
        } else {
          autoAligned = true;
        }
      }

      if (autoAligned) {
        Robot.driveSubsystem.arcadeDrive(0,0);
        HandState = !HandState; //toggle arm (false = panel locked in)
          if (HandState) {
            Robot.pickUpArmSubsystem.eject();
          } else {
            doForTime(300, () -> Robot.driveSubsystem.arcadeDrive(0.2,0));
            Robot.pickUpArmSubsystem.retract();
          }
          doForTime(400, () -> Robot.driveSubsystem.arcadeDrive(-0.7,0));
          autoAligned = false;
          AutoAimState = false;
      }
    } else {
      autoAligned = false;
      Robot.visionStrafeController.disable();
      Robot.limelightSubsystem.setLight(false);
    }

    if (!Robot.driveSubsystem.getDriveState().equals(DriveState.Mecanum))
      Robot.driveSubsystem.cartesianDrive(forward, side, turn);
    else
      Robot.driveSubsystem.arcadeDrive(forward, turn);

    if (Robot.opJoy.getRawButton(7)) {
      if (!RampLiftLock)
        Robot.rampSubsystem.closeRamp();
      RampLiftLock = true;
    } else {
      RampLiftLock = false;
    } 

    if (Robot.opJoy.getRawButton(8)) {
      if (!RampLowerLock)
        Robot.rampSubsystem.openRamp();
        RampLowerLock = true;
    } else {
      RampLowerLock = false;
    }

    if (Robot.driveJoy.getRawButton(1)) {
      if (!MecanumLock)
        Robot.driveSubsystem.setAll(DriveState.Mecanum);
      MecanumLock = true;
    } else {
      MecanumLock = false;
    } 

    if (Robot.driveJoy.getRawButton(2)) {
      if (!TractionLock)
        Robot.driveSubsystem.setAll(DriveState.Traction);
        TractionLock = true;
    } else {
      TractionLock = false;
    }

    if (Robot.opJoy.getRawButton(6)) {
      if (!HandLock)
        HandState = !HandState;
        HandLock = true;
    } else {
      HandLock = false;
    }

    if (HandState)
      Robot.pickUpArmSubsystem.eject();
    else
      Robot.pickUpArmSubsystem.retract();
    
    if (Robot.opJoy.getRawButton(4)) {
      if (!ArmTopLock)
        Robot.pickUpArmSubsystem.setArmAsync(ArmState.Top);
        ArmTopLock = true;
    } else {
      ArmTopLock = false;
    }

    if (Robot.opJoy.getRawButton(3)) {
      if (!ArmMiddleLock)
        Robot.pickUpArmSubsystem.setArmAsync(ArmState.Middle);
        ArmMiddleLock = true;
    } else {
      ArmMiddleLock = false;
    }

    if (Robot.opJoy.getRawButton(1)) {
      if (!ArmBottomLock)
        Robot.pickUpArmSubsystem.setArmAsync(ArmState.Bottom);
        ArmBottomLock = true;
    } else {
      ArmBottomLock = false;
    }

    if (Robot.opJoy.getRawButton(2)) {
      if (!ArmFloorLock)
        Robot.pickUpArmSubsystem.setArmAsync(ArmState.Floor);
        ArmFloorLock = true;
    } else {
      ArmFloorLock = false;
    }

    /*
    if (Robot.opJoy.getRawButton(9)) { //Button is random, needs to change (probably)
      if (!ArmConfinedLock)
        Robot.pickUpArmSubsystem.setArm(ArmState.Confined);
        ArmConfinedLock = true;
    } else {
      ArmConfinedLock = false;
    }
    */


    if (Robot.opJoy.getRawButton(5)) { //Button is random, needs to change (probably)
      if (!autoPositionLock)
        Robot.pickUpArmSubsystem.autoPosition = !Robot.pickUpArmSubsystem.autoPosition;
        autoPositionLock = true;
    } else {
      autoPositionLock = false;
    }

    // Uncomment if manual arm / wrist control is needed
    if (Robot.opJoy.getPOV() == 270 ||
    Robot.opJoy.getPOV() == 270 - 45||
    Robot.opJoy.getPOV() == 270 + 45) {
      Robot.pickUpArmSubsystem.WristSetHa(0.3);
    } else if (Robot.opJoy.getPOV() == 90 ||
    Robot.opJoy.getPOV() == 90 - 45||
    Robot.opJoy.getPOV() == 90 + 45) {
      Robot.pickUpArmSubsystem.WristSetHa(-0.35);
    } else {
      Robot.pickUpArmSubsystem.WristSetHa(0.0);
    }

    
    if (Robot.opJoy.getPOV() == 180 ||
    Robot.opJoy.getPOV() == 180 - 45||
    Robot.opJoy.getPOV() == 180 + 45) {
      Robot.pickUpArmSubsystem.ElbowSetHa(-0.4);
    } else if (Robot.opJoy.getPOV() == 0 ||
    Robot.opJoy.getPOV() == 0 - 45||
    Robot.opJoy.getPOV() == 0 + 45) {
      Robot.pickUpArmSubsystem.ElbowSetHa(0.7);
    } else {
      Robot.pickUpArmSubsystem.ElbowSetHa(0.0);
    }
    

    Robot.limelightSubsystem.getTA();
    Robot.limelightSubsystem.getTX();
    Robot.limelightSubsystem.getTS();
  }

  double percentToTarget(double value, double target) {
    double sign = (value < 0) ? -1 : 1;
    return ((target - Math.abs(value)) / target) * sign;
  }

  void doForTime(double millis, Runnable work) {  // Loop any piece of code for a set amount of time
    double end = System.currentTimeMillis() + millis;
    while (System.currentTimeMillis() < end)
      work.run();
    return;
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
