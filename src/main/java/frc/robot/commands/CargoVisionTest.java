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
  double forward = 0;

  public CargoVisionTest() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setCargoMode();
    Robot.visionController.setSetpoint(0);

    if (SmartDashboard.getNumber("kP", 0) == 0 &&
    SmartDashboard.getNumber("kI", 0) == 0 &&
    SmartDashboard.getNumber("kD", 0) == 0 &&
    SmartDashboard.getNumber("kF", 0) == 0) {
      SmartDashboard.putNumber("kP", Robot.visionController.getP());
      SmartDashboard.putNumber("kI", Robot.visionController.getI());
      SmartDashboard.putNumber("kD", Robot.visionController.getD());
      SmartDashboard.putNumber("kF", Robot.visionController.getF());
    }

    Robot.visionController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     
    double kP = SmartDashboard.getNumber("kP",Robot.vKP);
    double kI = SmartDashboard.getNumber("kI",Robot.vKI);
    double kD = SmartDashboard.getNumber("kD",Robot.vKD);
    double kF = SmartDashboard.getNumber("kF",Robot.vKF);
    Robot.visionController.setPID(kP, kI, kD, kF);

    double tv = Robot.limelightSubsystem.getTV();
    double tx = Robot.limelightSubsystem.getTX();
    double ta = Robot.limelightSubsystem.getTA();
    turn = (Math.abs(tx) <= Robot.kToleranceDegrees) ? 0 : -Robot.visionPIDTurn;

    if (tv != 0 && Robot.lidarSubsystem.getDistance() > 300)
      forward = (ta < 6.5) ? -percentToTarget(ta, 6.5)/3 : 0;
    else if (Robot.lidarSubsystem.getDistance() <= 300)
      forward = percentToTarget(Robot.lidarSubsystem.getDistance(), 300)/2;
    else
      forward = 0;

  Robot.driveSubsystem.arcadeDrive(forward,turn);

  SmartDashboard.putNumber("Turn speed", turn);
  SmartDashboard.putNumber("Tx", tx);
  SmartDashboard.putNumber("Ta", ta);
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
