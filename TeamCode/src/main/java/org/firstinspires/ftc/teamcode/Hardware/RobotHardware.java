package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotHardware {

    Servo IntakeServo;
    public SleeveDetection sleeveDetection;
    public OpenCvCamera backCamera;
    public MecanumDrive drive;

    private static RobotHardware instance = null;

    public boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        //TODO declaram motoare

    }

    public void loop(IntakeSubsystem intake) {
        try {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    ),
                    gamepad1.left_stick_x
            ));
            drive.updatePoseEstimate();
        } catch (Exception ignored){}

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
