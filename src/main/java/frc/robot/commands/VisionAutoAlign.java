/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Lidar2903.LidarPosition;

/**
 * Command that auto-aligns the robot to the vision target (Using only vision sensing!)
 */
public class VisionAutoAlign extends Command {
  double maxTA = 3.4;
  double maxLidar = 300;
  double forward = 0;
  double side = 0;
  double turn = 0;

  public VisionAutoAlign() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setTargetMode();

    Robot.visionTurnController.setSetpoint(0);
    Robot.visionStrafeController.setSetpoint(0);

    if (SmartDashboard.getBoolean("Target Vision: Turn Only", true))
      SmartDashboard.putBoolean("Target Vision: Turn Only", true);

    Robot.visionTurnController.enable();
    Robot.visionStrafeController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double tkP = SmartDashboard.getNumber("VisTurn kP", Robot.visionTurnPIDF[0]);
    double tkI = SmartDashboard.getNumber("VisTurn kI", Robot.visionTurnPIDF[1]);
    double tkD = SmartDashboard.getNumber("VisTurn kD", Robot.visionTurnPIDF[2]);
    double tkF = SmartDashboard.getNumber("VisTurn kF", Robot.visionTurnPIDF[3]);
    Robot.visionTurnController.setPID(tkP, tkI, tkD, tkF);

    double skP = SmartDashboard.getNumber("VisStrf kP", Robot.visionStrafePIDF[0]);
    double skI = SmartDashboard.getNumber("VisStrf kI", Robot.visionStrafePIDF[1]);
    double skD = SmartDashboard.getNumber("VisStrf kD", Robot.visionStrafePIDF[2]);
    double skF = SmartDashboard.getNumber("VisStrf kF", Robot.visionStrafePIDF[3]);
    Robot.visionStrafeController.setPID(skP, skI, skD, skF);

    double ta = Robot.limelightSubsystem.getTA();
    double lidar = Robot.lidarSubsystem.getDistance(LidarPosition.Center);

    if (Robot.limelightSubsystem.getTV() != 1) {
        forward = 0;
        side = 0;
        turn = 0;
    } else {
        forward = percentToTarget(ta, maxTA) * 0.4;
        side = Robot.visionStrafeValue * 0.75;
        turn = -Robot.visionTurnValue;
    }

    if (lidar <= maxLidar && Robot.lidarSubsystem.getStatus(LidarPosition.Center) == 0)
        forward = -percentToTarget(lidar, maxLidar) / 2;

    if (SmartDashboard.getBoolean("Target Vision: Turn Only", true))
        Robot.driveSubsystem.arcadeDrive(0, side, turn);
    else
        Robot.driveSubsystem.arcadeDrive(forward, side, turn);

    SmartDashboard.putNumber("Turn speed", turn);
    SmartDashboard.putNumber("Forward speed", forward);
    SmartDashboard.putNumber("Strafe speed", side);
  }

  //Returns a value from -1 to 1, depending on distance to target
  double percentToTarget(double value, double target) {
    double sign = (value < 0) ? -1 : 1;
    return ((target - Math.abs(value)) / target) * sign;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
      if (Robot.limelightSubsystem.getTV() != 1) return true;
      else return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.visionTurnController.disable();
    Robot.visionStrafeController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.visionTurnController.disable();
    Robot.visionStrafeController.disable();
  }
}
