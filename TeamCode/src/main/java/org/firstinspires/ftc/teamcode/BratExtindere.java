package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Hardware.Constante.ExtindatoareMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Constante.ExtindatoareMIN;

import org.firstinspires.ftc.teamcode.Hardware.Constante;

//@TeleOp
public class BratExtindere extends LinearOpMode {


    DcMotor extindatoarea;

    boolean stare = false;

    private Constante constants;
    //Gamepad gamepad1=new Gamepad();
    @Override
    public void runOpMode() {

        extindatoarea = hardwareMap.get(DcMotor.class, "extindatoarea");
        extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extindatoarea.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extindatoarea.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        waitForStart();
        while(opModeIsActive()) {
            //if(gamepad1.right_stick_button) extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("pozitie: ",extindatoarea.getCurrentPosition());
            telemetry.update();
            //extindatoarea.setPower(-gamepad1.right_stick_y);
            if (gamepad1.y) {
                SchimbareStareExtindatoare();
            }

        }
    }

    void SchimbareStareExtindatoare() {
        if (stare == false) {
            stare = true;
            extindatoarea.setTargetPosition(ExtindatoareMAX);
        }
        if (stare == true) {
            stare = false;
            extindatoarea.setTargetPosition(ExtindatoareMIN);
        }
    }
}
