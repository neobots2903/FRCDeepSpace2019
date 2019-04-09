package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class SwerveDrive2903 extends Subsystem {
Spark LeftRearForward;
Spark RightRearForward;
Spark LeftFrontForward;
Spark RightFrontForward;
WPI_TalonSRX LeftRearTurn;
WPI_TalonSRX RightRearTurn;
WPI_TalonSRX LeftFrontTurn;
WPI_TalonSRX RightFrontTurn;
final int TICKS_PER_REV = 1024;
final int DEG_PER_REV = 360;

  public void init() {
    LeftRearTurn = new WPI_TalonSRX(RobotMap.TBD);
    RightRearTurn = new WPI_TalonSRX(RobotMap.TBD);
    LeftFrontTurn = new WPI_TalonSRX(RobotMap.TBD);
    RightFrontTurn = new WPI_TalonSRX(RobotMap.TBD);
    LeftRearForward = new Spark(RobotMap.TBD);
    LeftFrontForward = new Spark(RobotMap.TBD);
    RightFrontForward = new Spark(RobotMap.TBD);
    RightRearForward = new Spark(RobotMap.TBD);

    LeftRearTurn.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
  
    setPowerPercent(0.4); //set max turn power to just 40%;
  }

  double getTurnTicks (WPI_TalonSRX talon) {
   return talon.getSelectedSensorPosition();
  }

  double getTurnDegrees (WPI_TalonSRX talon) {
    return ticksToAngle(getTurnTicks(talon));
  }

  void setTurnTicks(WPI_TalonSRX talon, double ticks) {
    talon.set(ControlMode.Position, ticks);
  }

  void setTurnDegrees(WPI_TalonSRX talon, double degrees) {
    talon.set(ControlMode.Position, angleToTicks(degrees));
  }

  double ticksToAngle (double ticks) {
    double remainder = ticks % TICKS_PER_REV;
    remainder /= TICKS_PER_REV;
    return remainder * DEG_PER_REV;
  }

  double angleToTicks (double angle) {
    double remainder = angle % DEG_PER_REV;
    remainder /= DEG_PER_REV;
    return remainder * TICKS_PER_REV;
  }

  void setPowerPercent(double val) {
    double value = 
      (val > 1) ? 1 : 
      (val < 0) ? 0 : val;

    LeftFrontTurn.configPeakOutputForward(value, 0);
    LeftFrontTurn.configPeakOutputReverse(-value, 0);

    RightFrontTurn.configPeakOutputForward(value, 0);
		RightFrontTurn.configPeakOutputReverse(-value, 0);

		LeftRearTurn.configPeakOutputForward(value, 0);
		LeftRearTurn.configPeakOutputReverse(-value, 0);

		RightRearTurn.configPeakOutputForward(value, 0);
    RightRearTurn.configPeakOutputReverse(-value, 0);
  }

  public double joystickAngle(double x, double y) {
    return Math.atan2(y, x);
  }

  public void swerveDrive(double power, double angle, double turn) {
    setTurnDegrees(LeftRearTurn,angle-(turn*-45));
    setTurnDegrees(RightRearTurn,angle-(turn*-45));
    setTurnDegrees(LeftFrontTurn,angle-(turn*-135));
    setTurnDegrees(RightFrontTurn,angle-(turn*135));

    LeftRearForward.set(power);
    LeftFrontForward.set(power);
    RightFrontForward.set(power);
    RightRearForward.set(power);
  }

  @Override
  protected void initDefaultCommand() {

  }
}