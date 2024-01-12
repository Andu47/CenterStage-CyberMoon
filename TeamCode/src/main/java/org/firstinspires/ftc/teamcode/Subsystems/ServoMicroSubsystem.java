package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class ServoMicroSubsystem extends SubsystemBase {
    private RobotHardware robot;

    public ServoMicroSubsystem(RobotHardware robot){
        this.robot=robot;
    }
    public void setMicroServo1(double pos){
        robot.MicroServo1.setPosition(pos);
    }
    public void setMicroServo2(double pos){
        robot.MicroServo2.setPosition(pos);
    }
    public void setMicroServo12(double pos){
        robot.MicroServo1.setPosition(pos);
        robot.MicroServo2.setPosition(pos);
    }
}
