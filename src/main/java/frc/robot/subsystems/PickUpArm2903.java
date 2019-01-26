package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

public class PickUpArm2903 extends Subsystem {
    double speed = 0.5;
    private TalonSRX leftIntakeMotor;
    private TalonSRX rightIntakeMotor;
    private TalonSRX wristMotor;
    private TalonSRX elbowMotor;
    public Solenoid leftPunch;
    public Solenoid rightPunch;

    @Override
    protected void initDefaultCommand() {
    }
    public PickUpArm2903(){
        leftIntakeMotor = new WPI_TalonSRX(deviceNumber);
        rightIntakeMotor = new WPI_TalonSRX(deviceNumber);
        wristMotor = new WPI_TalonSRX(deviceNumber);
        elbowMotor = new WPI_TalonSRX(deviceNumber);
        leftPunch = new Solenoid(channel);
        rightPunch = new Solenoid(channel);
    }

    public void TakeIn() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,speed);
        rightIntakeMotor.set(ControlMode.PercentOutput ,speed);
    }
    public void TakeOut() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,-speed);
        rightIntakeMotor.set(ControlMode.PercentOutput ,-speed);
    }
    public void TakeStop() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,0);
        rightIntakeMotor.set(ControlMode.PercentOutput ,0);
    }
    public void WristUp(){
        wristMotor.set(ControlMode.PercentOutput ,speed);
    }
    public void WristDown(){
        wristMotor.set(ControlMode.PercentOutput ,-speed);
    }
    public void WristStop(){
        wristMotor.set(ControlMode.PercentOutput, 0);
    }
    public void ElbowUp(){
        elbowMotor.set(ControlMode.PercentOutput ,speed);
    }
    public void ElbowDown(){
        elbowMotor.set(ControlMode.PercentOutput ,-speed);
    }
    public void ElbowStop(){
        elbowMotor.set(ControlMode.PercentOutput, 0);
    }
    public void punch(){
        leftPunch.set(true);
        rightPunch.set(true);
        //wait?tbh
        leftPunch.set(false);
        rightPunch.set(false);
    }
    
}