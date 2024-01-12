package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class PivotingMotorSubsystem extends SubsystemBase {
    private RobotHardware robot;

    private double power=0;

    public PivotingMotorSubsystem(RobotHardware robot){
        this.robot=robot;
    }
    public void setPivotingMotorTarget(int pos){
        robot.PivotingMotor.setTargetPosition(pos);
    }
    public void setPivotingMotorTargetAngle(int angle){
        int pos=angle;//TODO
        robot.PivotingMotor.setTargetPosition(pos);
    }
    public int getPivotingMotorPosition(){
        return robot.PivotingMotor.getCurrentPosition();
    }
    public void manualMovement(double power)
    {
        robot.PivotingMotor.setPower(power);
    }
}
