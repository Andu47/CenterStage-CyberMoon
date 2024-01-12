package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class ServoControlSubsystem extends SubsystemBase {
    private RobotHardware robot;
    public ServoControlSubsystem(RobotHardware robot){
        this.robot= robot;
    }
    public void setServoControl(double pos){
        robot.AngleControlServo.setPosition(pos);
    }
    public void setAngleServoControl(int angle){
        double pos=angle/300.00;
        robot.AngleControlServo.setPosition(pos);
    }
}
