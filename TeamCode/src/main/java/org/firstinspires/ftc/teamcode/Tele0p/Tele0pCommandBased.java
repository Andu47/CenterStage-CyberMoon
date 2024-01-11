package org.firstinspires.ftc.teamcode.Tele0p;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "Tele0pCommandBased")
public class Tele0pCommandBased extends CommandOpMode {
    private ElapsedTime timer;
    private RobotHardware robot= RobotHardware.getInstance();
    private MecanumDrive drive;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new IntakeSubsystem(robot);
        //gamepadEx = new GamepadEx(gamepad1);
        //gamepadEx2 = new GamepadEx(gamepad2);

    }

    @Override
    public void run() {
        super.run();

        robot.read(intake);





        robot.loop(intake);
        robot.write(intake);
        //double loop = System.nanoTime();
//        telemetry.addData ();
//        telemetry.addData();
        //telemetry.update();

    }
}
