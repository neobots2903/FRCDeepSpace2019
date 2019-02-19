package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Ramp2903 extends Subsystem {

    public Solenoid rampToggler = new Solenoid(RobotMap.TBD);

    public Solenoid platformToggler = new Solenoid(RobotMap.TBD);

    protected void initDefaultCommand() {
        
    }

    public void init() {
        rampToggler.set(true);
        platformToggler.set(true);
    }

    public void lowerRamp() {
        rampToggler.set(false);
    }

    public void raiseRamp() {
        rampToggler.set(true);
    }

    public void raisePlatform() {
        platformToggler.set(true);
    }

    public void lowerPlatform() {
        platformToggler.set(false);
    }

}