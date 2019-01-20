/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * The drive subsystem. Use this if you are planning on driving!
 */
public class Drive2903 extends Subsystem {

  TalonSRX LeftFrontMotor;
  TalonSRX RightFrontMotor;
  TalonSRX LeftRearMotor;
  TalonSRX RightRearMotor;

  static double maxOutput = 1; //Reduce if robot is drawing too much power

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void init () {
    LeftFrontMotor = new TalonSRX(RobotMap.LeftFrontMotor);
    RightFrontMotor = new TalonSRX(RobotMap.RightFrontMotor);
    LeftRearMotor = new TalonSRX(RobotMap.LeftRearMotor);
    RightRearMotor = new TalonSRX(RobotMap.RightRearMotor);

    
    LeftFrontMotor.configPeakOutputForward(maxOutput, 0);
    LeftFrontMotor.configPeakOutputReverse(-maxOutput, 0);

    RightFrontMotor.configPeakOutputForward(maxOutput, 0);
		RightFrontMotor.configPeakOutputReverse(-maxOutput, 0);

		LeftRearMotor.configPeakOutputForward(maxOutput, 0);
		LeftRearMotor.configPeakOutputReverse(-maxOutput, 0);

		RightRearMotor.configPeakOutputForward(maxOutput, 0);
    RightRearMotor.configPeakOutputReverse(-maxOutput, 0);

    LeftFrontMotor.set(ControlMode.PercentOutput, 0);
    LeftRearMotor.set(ControlMode.PercentOutput, 0);
    RightFrontMotor.set(ControlMode.PercentOutput, 0) ;
    RightRearMotor.set(ControlMode.PercentOutput, 0);
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

public void arcadeDrive(double forward, double side, double turn) {
  double scale = (forward + turn + side > 1) ? 1 / (forward + turn + side) : 1;
  LeftFrontMotor.set(ControlMode.PercentOutput, (-turn + forward + side));
  LeftRearMotor.set(ControlMode.PercentOutput, (-turn + forward - side));

  RightFrontMotor.set(ControlMode.PercentOutput, (-turn - forward - side)) ;
  RightRearMotor.set(ControlMode.PercentOutput, (-turn - forward + side));
}

  public void arcadeDrive(double forward, double turn) {
    arcadeDrive(forward, 0, turn);
  }

}
