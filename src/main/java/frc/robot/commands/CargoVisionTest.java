/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class CargoVisionTest extends Command {

  NetworkTableEntry tx; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  NetworkTableEntry ta; //Target Area (0% of image to 100% of image)
  NetworkTableEntry tv; //Whether the limelight has any valid targets (0 or 1)
  double turn;
  double maxSpeed = 0.5;

  public CargoVisionTest() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    table.getEntry("ledMode").setNumber(1); // forces LED off
    table.getEntry("pipeline").setNumber(1); // sets cargo pipeline
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (getEntryValue(tv) == 0) {
      Robot.driveSubsystem.arcadeDrive(0, 0);
    } else {
      turn = percentToTarget(getEntryValue(tx),27);
      Robot.driveSubsystem.arcadeDrive(0, (turn > maxSpeed) ? maxSpeed : turn);
    }
    SmartDashboard.putNumber("Turn speed", 
      (turn > maxSpeed) ? maxSpeed : turn);
    SmartDashboard.putNumber("Tx", getEntryValue(tx));
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
