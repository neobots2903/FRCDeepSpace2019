/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * The drive subsystem. Use this if you are planning on driving!
 */
public class Drive2903 extends Subsystem {

  WPI_TalonSRX LeftFrontMotor;
  WPI_TalonSRX RightFrontMotor;
  WPI_TalonSRX LeftRearMotor;
  WPI_TalonSRX RightRearMotor;
  
  MecanumDrive mecanumDrive;
  PowerDistributionPanel pdp;

  Solenoid driveFrontLower;
  Solenoid driveFrontLift;
  Solenoid driveRearLower;
  Solenoid driveRearLift;

  DriveState currentDriveState = DriveState.Traction;

  static double idealVoltage = 12.8;
  static double speedScale = 1;
  static double strafeFrontRestrict = 1; //Used only when strafing, slows down front wheels to account for unbalanced weight
  static double gyroError = 2;
  static double lastGyroAngle = 0;
  static boolean withGyro = false;

  static final boolean experimentalTurn = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void init () {
    pdp = new PowerDistributionPanel();
    LeftFrontMotor = new WPI_TalonSRX(RobotMap.LeftFrontMotor);
    RightFrontMotor = new WPI_TalonSRX(RobotMap.RightFrontMotor);
    LeftRearMotor = new WPI_TalonSRX(RobotMap.LeftRearMotor);
    RightRearMotor = new WPI_TalonSRX(RobotMap.RightRearMotor);

    driveFrontLower = new Solenoid(RobotMap.driveFrontLower);
    driveFrontLift = new Solenoid(RobotMap.driveFrontLift);
    driveRearLower = new Solenoid(RobotMap.driveRearLower);
    driveRearLift = new Solenoid(RobotMap.driveRearLift);

    LeftFrontMotor.set(ControlMode.PercentOutput, 0);
    LeftRearMotor.set(ControlMode.PercentOutput, 0);
    RightFrontMotor.set(ControlMode.PercentOutput, 0) ;
    RightRearMotor.set(ControlMode.PercentOutput, 0);

    setPowerPercent(1);

    mecanumDrive = new MecanumDrive(LeftFrontMotor, LeftRearMotor, RightFrontMotor, RightRearMotor);
    mecanumDrive.setRightSideInverted(true);
    setAll(DriveState.Traction);
    SmartDashboard.putNumber("Strafe adjust", 0.00);
  }

  void setPowerPercent(double val) {
    double value = 
      (val > 1) ? 1 : 
      (val < 0) ? 0 : val;

    LeftFrontMotor.configPeakOutputForward(value, 0);
    LeftFrontMotor.configPeakOutputReverse(-value, 0);

    RightFrontMotor.configPeakOutputForward(value, 0);
		RightFrontMotor.configPeakOutputReverse(-value, 0);

		LeftRearMotor.configPeakOutputForward(value, 0);
		LeftRearMotor.configPeakOutputReverse(-value, 0);

		RightRearMotor.configPeakOutputForward(value, 0);
    RightRearMotor.configPeakOutputReverse(-value, 0);
  }
/*

    @ ---------------- @
    @ |  /        \  | @
      | / 1      3 \ |
      |              |
      | \ 2      4 / |
    @ |  \        /  | @
    @ ---------------- @
                   
Forward: Positive power to all
Strafe Left: 2 + 3 positive, 1 + 4 negative


*/

public void turnToDegree(double degree) {
  Robot.gyroController.enable();
  double target = Robot.navXSubsystem.turnAngle() + degree;
  Robot.gyroController.setSetpoint(target);
  while (Math.abs(Robot.navXSubsystem.turnAngle() - target) > gyroError)
    arcadeDrive(0,Robot.gyroPIDTurn);
  Robot.driveSubsystem.arcadeDrive(0, 0);
  Robot.gyroController.disable();
}

public void cartesianDrive(double forward, double side, double turn) {
  setPowerPercent(Math.pow(pdp.getVoltage()/idealVoltage,4));
  mecanumDrive.driveCartesian(side, -forward, -turn);
}

public void tankDrive (double leftSpeed, double rightSpeed) {
  setPowerPercent(Math.pow(pdp.getVoltage()/idealVoltage,4));
  LeftFrontMotor.set(ControlMode.PercentOutput, leftSpeed);
  LeftRearMotor.set(ControlMode.PercentOutput, leftSpeed);
  RightFrontMotor.set(ControlMode.PercentOutput, rightSpeed);
  RightRearMotor.set(ControlMode.PercentOutput, rightSpeed);
}
public void arcadeDrive(double forward, double side, double turn) {
  setPowerPercent(Math.pow(pdp.getVoltage()/idealVoltage,4));
  //THIS ATTEMPTS TO HOLD A HEADING WHILE DRIVING STRAIGHT / STRAFING
  //**EXPERIMENTAL!**
  double f_turn = 0;
  if (Math.abs(turn) <= 0.07) {
    if(!withGyro) lastGyroAngle = Robot.navXSubsystem.turnAngle();
    withGyro = true;
    // Robot.gyroController.setSetpoint(lastGyroAngle);
    // Robot.gyroController.enable();
    // if (Math.abs(Robot.navXSubsystem.turnAngle() - lastGyroAngle) > gyroError)
    //   f_turn = Robot.gyroPIDTurn;
    // SmartDashboard.putNumber("Target Gyro", Robot.gyroController.getSetpoint());
  } else {
    withGyro = false;
    lastGyroAngle = Robot.navXSubsystem.turnAngle();
    // SmartDashboard.putNumber("Target Gyro", Robot.gyroController.getSetpoint());
    // Robot.gyroController.disable();
    f_turn = turn;
  }
  
  double strafeAdj = SmartDashboard.getNumber("Strafe adjust", 0.00);
  // THIS ADJUSTS THE FRONT WHEEL SPEED WHILE STRAFING TO COUNTER IMBALANCE OF WHEELS
  //**EXPERIMENTAL!**
  // if (Math.abs(turn) <= 0.07 && side != 0)  
  // if (Robot.navXSubsystem.turnAngle() - lastGyroAngle > gyroError)
  // strafeFrontRestrict-=strafeAdj*((side < 0) ? 1 : -1);  //if we're turning right, adjust front speed
  // else if (Robot.navXSubsystem.turnAngle() - lastGyroAngle < gyroError)
  // strafeFrontRestrict+=strafeAdj*((side < 0) ? 1 : -1); //if we're turning left, adjust front speed
  
  LeftFrontMotor.set(ControlMode.PercentOutput, (speedScale * 
    (((experimentalTurn) ? -f_turn : -turn) - forward + strafeFrontRestrict*side)
    ));
    
  LeftRearMotor.set(ControlMode.PercentOutput, (speedScale * 
    (((experimentalTurn) ? -f_turn : -turn) - forward - side)
    ));

  RightFrontMotor.set(ControlMode.PercentOutput, (speedScale * 
    (((experimentalTurn) ? -f_turn : -turn) + forward + strafeFrontRestrict*side)
    ));

  RightRearMotor.set(ControlMode.PercentOutput, (speedScale * 
    (((experimentalTurn) ? -f_turn : -turn) + forward - side)
    ));
}

  public void arcadeDrive(double forward, double turn) {
    arcadeDrive(forward, 0, turn);
  }

  public enum DriveState {
    Traction, Mecanum
  }

  public void setRear(DriveState state) {
    if (DriveState.Traction == state) {
      driveRearLower.set(false);
      driveRearLift.set(true);
    } else if (DriveState.Mecanum == state){
      driveRearLower.set(true);
      driveRearLift.set(false);
    }
  }

  public void setFront(DriveState state) {
    if (DriveState.Traction == state) {
      driveFrontLower.set(false);
      driveFrontLift.set(true);
    } else if (DriveState.Mecanum == state){
      driveFrontLower.set(true);
      driveFrontLift.set(false);
    }
  }

  public void setAll(DriveState state) {
    setFront(state);
    setRear(state);
    currentDriveState = state;
  }

  public DriveState getDriveState() {
    return currentDriveState;
  }

}
