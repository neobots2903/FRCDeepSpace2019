package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PickUpArm2903.ArmState;

public class Ramp2903 extends Subsystem {

    public static final double R_RAMP_CLOSE = -0.5;
    public static final double R_RAMP_OPEN = 0.5;
    public static final double L_RAMP_CLOSE = -0.5;
    public static final double L_RAMP_OPEN = 0.5;
    public static final double S_RAMP_CLOSE = -0.5;
    public static final double S_RAMP_OPEN = 0.5;

    public Solenoid rampLower = new Solenoid(RobotMap.rampLower);
    public Solenoid rampLift = new Solenoid(RobotMap.rampLift);
    //public Servo rightRamp = new Servo(RobotMap.TBD);
    //public Servo leftRamp = new Servo(RobotMap.TBD);

    protected void initDefaultCommand() {
    }

    public void init() {
        closeRamp();
    }

    public void openRamp() {
        Robot.pickUpArmSubsystem.setArm(ArmState.Away);
        //rightRamp.set(R_RAMP_OPEN);
        //leftRamp.set(L_RAMP_OPEN);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        liftRamp();
    }

    public void closeRamp() {
        Robot.pickUpArmSubsystem.setArm(ArmState.Confined);
        lowerRamp();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //rightRamp.set(R_RAMP_CLOSE);
        //leftRamp.set(L_RAMP_CLOSE);
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