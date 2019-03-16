package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PickUpArm2903.ArmState;

public class Ramp2903 extends Subsystem {

    public static final double R_RAMP_CLOSE = 0.0;
    public static final double R_RAMP_OPEN = 1.0;
    public static final double L_RAMP_CLOSE = 1.0;
    public static final double L_RAMP_OPEN = 0.0;

    public Solenoid rampLower = new Solenoid(RobotMap.rampLower);
    public Solenoid rampLift = new Solenoid(RobotMap.rampLift);
    public Servo rightRamp = new Servo(RobotMap.rampServoRight);
    public Servo leftRamp = new Servo(RobotMap.rampServoLeft);

    protected void initDefaultCommand() {
    }

    public void init() {
        closeRamp();
    }

    public void openRamp() {
        Robot.pickUpArmSubsystem.setArm(ArmState.Confined);
        liftRamp();
        rightRamp.setPosition(R_RAMP_CLOSE);
        leftRamp.setPosition(L_RAMP_CLOSE);
    }

    public void closeRamp() {
        Robot.pickUpArmSubsystem.setArm(ArmState.Away);
        rightRamp.setPosition(R_RAMP_OPEN);
        leftRamp.setPosition(L_RAMP_OPEN);
        lowerRamp();
    }

    public void liftRamp() {
        rampLower.set(true);
        rampLift.set(false);
    }

    public void lowerRamp() {
        rampLower.set(false);
        rampLift.set(true);
    }

}