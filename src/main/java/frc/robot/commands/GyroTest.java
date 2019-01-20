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
import frc.robot.subsystems.Drive2903;

/**
 * An example command.  You can replace me with your own command.
 */
public class GyroTest extends Command {

  public GyroTest() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double kP = SmartDashboard.getNumber("kP",Robot.kP);
    double kI = SmartDashboard.getNumber("kI",Robot.kI);
    double kD = SmartDashboard.getNumber("kD",Robot.kD);
    double kF = SmartDashboard.getNumber("kF",Robot.kF);
    Robot.turnController.setPID(kP, kI, kD, kF);

    boolean rotateToAngle = false;

   if (Robot.driveJoy.getRawButton(4)) {
     Robot.turnController.setSetpoint(0.0f);
     rotateToAngle = true;
   } else if (Robot.driveJoy.getRawButton(2)) {
    Robot.turnController.setSetpoint(90.0f);
    rotateToAngle = true;
   } else if (Robot.driveJoy.getRawButton(1)) {
    Robot.turnController.setSetpoint(179.9f);
    rotateToAngle = true;
   } else if (Robot.driveJoy.getRawButton(3)) {
    Robot.turnController.setSetpoint(-90.0f);
    rotateToAngle = true;
   }

   if (rotateToAngle) {
     Robot.turnController.enable();
     Robot.driveSubsystem.arcadeDrive(0, Robot.rotateToAngleRate);
   } else {
     Robot.turnController.disable();
     Robot.driveSubsystem.arcadeDrive(0, 0);
   }

   SmartDashboard.putNumber("Gyro Angle", Robot.navXSubsystem.turnAngle());
   SmartDashboard.putBoolean("Rotate to Angle?", rotateToAngle);
   SmartDashboard.putNumber("Target Angle", Robot.turnController.getSetpoint());
   SmartDashboard.putNumber("RotateToAngleRate", Robot.rotateToAngleRate);
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
