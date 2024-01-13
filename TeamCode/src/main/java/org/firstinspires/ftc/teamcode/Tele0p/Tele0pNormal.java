package org.firstinspires.ftc.teamcode.Tele0p;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingMotorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ServoControlSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ServoMicroSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp
public class Tele0pNormal extends LinearOpMode {
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    public DcMotorEx PivotingMotor;
    public DcMotorEx ExtentionMotor;
    private MecanumDrive drive;
    int extTarget=0;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");
        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");

        PivotingMotor= hardwareMap.get(DcMotorEx.class, "PivotingMotor");
        PivotingMotor.setTargetPosition(0);
        PivotingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtentionMotor= hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtentionMotor.setTargetPosition(0);
        ExtentionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            extTarget=ExtentionMotor.getCurrentPosition();
            //robot.read(intake);
            //robot.loop(intake);
            //robot.write(intake);
            //* MUTARE SERVOCONTROL LA POZITIE MAXIMA DUPA PivotMID
            if(PivotingMotor.getCurrentPosition()> RobotHardware.PivotMID)
                AngleControlServo.setPosition(RobotHardware.ServoControlMAX);

            //* MUTARE EXTENTION DE LA JOYSTICK
            if(gamepad2.left_stick_y!=0){
                extTarget+=(int)(gamepad2.left_stick_y*10);
                extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
            }
            ExtentionMotor.setTargetPosition(extTarget);

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
}

