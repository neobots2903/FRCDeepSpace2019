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

  public TeleOp() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setTargetMode();
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

    /*
    SmartDashboard.putBoolean("Line left?", Robot.lineSubsystem.leftDetected());
    SmartDashboard.putBoolean("Line center?", Robot.lineSubsystem.centerDetected());
    SmartDashboard.putBoolean("Line right?", Robot.lineSubsystem.rightDetected());
    */

    Robot.driveSubsystem.arcadeDrive(forward, side, turn);

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
