/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class Autonomous extends Command {

  public Autonomous() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.rampSubsystem.closeRamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    doForTime(1000, () -> Robot.driveSubsystem.arcadeDrive(1, 0));  //Fancy way to drive off platform for 1 second
    //Robot.driveSubsystem.turnToDegree(-90);
    //doForTime(1000, () -> Robot.driveSubsystem.arcadeDrive(1, 0));
    Robot.driveSubsystem.arcadeDrive(0, 0);
    //assuming we're lined up...
    //armSubsystem.insertCargo(); OR armSubsystem.placePanel();
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
