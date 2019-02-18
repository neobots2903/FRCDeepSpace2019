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
    double kP = SmartDashboard.getNumber("Gyro kP",Robot.gyroPIDF[0]);
    double kI = SmartDashboard.getNumber("Gyro kI",Robot.gyroPIDF[1]);
    double kD = SmartDashboard.getNumber("Gyro kD",Robot.gyroPIDF[2]);
    double kF = SmartDashboard.getNumber("Gyro kF",Robot.gyroPIDF[3]);
    Robot.gyroController.setPID(kP, kI, kD, kF);

    boolean rotateToAngle = false;

   if (Robot.driveJoy.getRawButton(4)) {
     Robot.gyroController.setSetpoint(0.0f);
     rotateToAngle = true;
   } else if (Robot.driveJoy.getRawButton(2)) {
    Robot.gyroController.setSetpoint(90.0f);
    rotateToAngle = true;
   } else if (Robot.driveJoy.getRawButton(1)) {
    Robot.gyroController.setSetpoint(179.9f);
    rotateToAngle = true;
   } else if (Robot.driveJoy.getRawButton(3)) {
    Robot.gyroController.setSetpoint(-90.0f);
    rotateToAngle = true;
   }

   if (rotateToAngle) {
     Robot.gyroController.enable();
     Robot.driveSubsystem.arcadeDrive(0, Robot.gyroPIDTurn);
   } else {
     Robot.gyroController.disable();
     Robot.driveSubsystem.arcadeDrive(0, 0);
   }

   SmartDashboard.putNumber("Gyro Angle", Robot.navXSubsystem.turnAngle());
   SmartDashboard.putBoolean("Rotate to Angle?", rotateToAngle);
   SmartDashboard.putNumber("Target Angle", Robot.gyroController.getSetpoint());
   SmartDashboard.putNumber("RotateToAngleRate", Robot.gyroPIDTurn);
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
