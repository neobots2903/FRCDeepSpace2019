/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class CargoVisionTest extends Command {
  double turn = 0;
  double maxSpeed = 0.5;
  double maxError = 1;

  public CargoVisionTest() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setCargoMode();
    Robot.pidSubsystem.setSetpoint(0);

    if (SmartDashboard.getNumber("P", 0) == 0 &&
    SmartDashboard.getNumber("I", 0) == 0 &&
    SmartDashboard.getNumber("D", 0) == 0) {
      SmartDashboard.putNumber("P", Robot.pidSubsystem.getP());
      SmartDashboard.putNumber("I", Robot.pidSubsystem.getI());
      SmartDashboard.putNumber("D", Robot.pidSubsystem.getD());
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    double p = SmartDashboard.getNumber("P", 0);
    double i = SmartDashboard.getNumber("I", 0);
    double d = SmartDashboard.getNumber("D", 0);

    Robot.pidSubsystem.setPID(p, i, d);

    double txD = -getEntryValue(Robot.limelightSubsystem.tx);

    //if (Math.abs(turn) > maxSpeed) turn = maxSpeed * sign;

    if (getEntryValue(Robot.limelightSubsystem.tv) == 0) {
      Robot.driveSubsystem.arcadeDrive(0, turn);
    } else {
      turn = Robot.pidSubsystem.getOutput(txD) / 100;
      //turn = percentToTarget(getEntryValue(tx),27);
      Robot.driveSubsystem.arcadeDrive(0, turn);
    }
    SmartDashboard.putNumber("Turn speed", turn);
    SmartDashboard.putBoolean("Tv", getEntryValue(Robot.limelightSubsystem.tv) == 1);
    SmartDashboard.putNumber("Tx", txD);
  }

  double getEntryValue(NetworkTableEntry entry) {
    return entry.getNumber(0).doubleValue();
  }

  double percentToTarget(double value, double target) {
    double sign = (value < 0) ? -1 : 1;
    return ((target - Math.abs(value)) / target) * sign;
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
