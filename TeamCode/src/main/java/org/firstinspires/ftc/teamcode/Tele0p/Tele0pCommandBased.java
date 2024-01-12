package org.firstinspires.ftc.teamcode.Tele0p;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name = "Tele0pCommandBased")
public class Tele0pCommandBased extends CommandOpMode {
    private ElapsedTime timer;
    private RobotHardware robot= RobotHardware.getInstance();
    private MecanumDrive drive;


    OpenCvCamera backCamera;
    SleeveDetection.SkystoneDeterminationPipeline pipeline;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        //gamepadEx = new GamepadEx(gamepad1);
        //gamepadEx2 = new GamepadEx(gamepad2);

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new SleeveDetection.SkystoneDeterminationPipeline();
        backCamera.setPipeline(pipeline);

        backCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                backCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(backCamera, 10);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void run() {
        super.run();
        //robot.read(intake);
        //robot.loop(intake);
        //robot.write(intake);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                ),
                gamepad1.left_stick_x
        ));

        drive.updatePoseEstimate();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}
