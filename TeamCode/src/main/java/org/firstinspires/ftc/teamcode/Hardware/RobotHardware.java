package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class RobotHardware {

    Servo IntakeServo;
    private HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        //TODO declaram motoare
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");

    }

    public void loop(MecanumDrive drive, IntakeSubsystem intake) {
        try {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    ),
                    gamepad1.left_stick_x
            ));

            drive.updatePoseEstimate();
        } catch (Exception ignored) {
        }
//        try {
//            intake.loop2();
//        } catch (Exception ignored){
    }

    public void read(IntakeSubsystem intake) {
//        try {
//            intake.read();
//        } catch (Exception ignored) {
    }

    public void write(IntakeSubsystem intake) {
//            try {
//                intake.write();
//            } catch (Exception ignored){}
    }
}
