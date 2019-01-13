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
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class Autonomous extends Command {

  NetworkTableEntry tx; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  NetworkTableEntry ty; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  NetworkTableEntry ta; //Target Area (0% of image to 100% of image)
  NetworkTableEntry tv; //Whether the limelight has any valid targets (0 or 1)

  public Autonomous() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    table.getEntry("ledMode").setNumber(3); // forces LED on
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    doForTime(1000, () -> Robot.driveSubsystem.arcadeDrive(1, 0));  //Fancy way to drive off platform for 1 second
    while (tv.getNumber(0).intValue() == 0) { //while no target is seen -
      //wander around using lidar? Drive in the general direction of target?
      //we simply need a method to get to a target.
    }
    //here, we need to approach the target, and make sure we are relatively parallel.
    while (ta.getNumber(0).intValue() < 30) {
      Robot.driveSubsystem.arcadeDrive( percentToTarget(getEntryValue(ta),30), 0);
      //this simply drives towards the target, slowing down as it approaches.
      //we need to find a way to calculate the turn/strafe required to approach mostly parallel.
      //we'll probably have to turn towards target using gyro, 
      //unless we can use sensors to detect if we are parallel to the wall.
      //if we can get the width and height of our target, we can get an angle out of that as well.
      //limelight writes 'ts' (target skew) to NetworkTables, but I'm unsure if that's what we need.
    }
    //assuming we're lined up...
    //armSubsystem.insertCargo(); OR armSubsystem.placePanel();
  }

  double getEntryValue(NetworkTableEntry entry) {
    return entry.getNumber(0).doubleValue();
  }

  double percentToTarget(double value, double target) {
    return (target - value) / target;
  }

  void doForTime(double millis, Runnable work) {  // Loop any piece of code for a set amount of time
    double end = System.currentTimeMillis() + millis;
    while (System.currentTimeMillis() < end)
      work.run();
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
