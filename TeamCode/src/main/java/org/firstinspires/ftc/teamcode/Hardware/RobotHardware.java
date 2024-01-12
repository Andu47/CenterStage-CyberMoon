package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class RobotHardware {

    public static int pipelineStage = 0;
    public static double BLUR_RADIUS = 7;
    public static double HUE_MIN = 0;
    public static double HUE_MAX = 90;
    public static double SATURATION_MIN = 150;
    public static double SATURATION_MAX = 255;
    public static double VALUE_MIN = 150;
    public static double VALUE_MAX = 255;
    public static double MIN_CONTOUR_AREA = 2500;
    public static String BLUR = "Box Blur";
    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    public DcMotorEx PivotingMotor;
    public DcMotorEx ExtentionMotor;

//    public DcMotorEx FataDr;
//    public DcMotorEx SpateDr;
//    public DcMotorEx FataSt;
//    public DcMotorEx SpateSt;

    public SleeveDetection.SkystoneDeterminationPipeline pipeline;
    public OpenCvCamera backCamera;
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
        MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");
        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");

        AngleControlServo= hardwareMap.get(Servo.class, "ControlServo");

        PivotingMotor= hardwareMap.get(DcMotorEx.class, "PivotingMotor");
        ExtentionMotor= hardwareMap.get(DcMotorEx.class, "ExtentionMotor");
        ExtentionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        FataDr= hardwareMap.get(DcMotorEx.class, "FataDr");
//        FataSt= hardwareMap.get(DcMotorEx.class, "FataSt");
//        SpateDr= hardwareMap.get(DcMotorEx.class, "SpateDr");
//        SpateSt= hardwareMap.get(DcMotorEx.class, "SpateSt");

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new SleeveDetection.SkystoneDeterminationPipeline();
        backCamera.setPipeline(pipeline);

    }

    public void loop() {
        /*try {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    ),
                    gamepad1.left_stick_x
            ));
            drive.updatePoseEstimate();
        } catch (Exception ignored){}*/

//        try {
//            intake.loop2();
//        } catch (Exception ignored){
    }

    public void read() {
//        try {
//            intake.read();
//        } catch (Exception ignored) {
    }

    public void write() {
//            try {
//                intake.write();
//            } catch (Exception ignored){}
    }
}
