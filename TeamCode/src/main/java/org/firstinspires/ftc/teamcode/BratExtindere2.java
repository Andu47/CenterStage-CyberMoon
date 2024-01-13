package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Constante.ExtindatoareMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Constante.ExtindatoareMIN;

import org.firstinspires.ftc.teamcode.Hardware.Constante;

@TeleOp
public class BratExtindere2 extends LinearOpMode {


    DcMotor extindatoarea;

   // boolean stare = false;

    private Constante constants;
    //Gamepad gamepad1=new Gamepad();
    @Override
    public void runOpMode() {

        int pozitie = 0;
        int x = 0;

        extindatoarea = hardwareMap.get(DcMotor.class, "extindatoarea");
        extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extindatoarea.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extindatoarea.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extindatoarea.setTargetPosition(pozitie);

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.right_stick_button) extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("pozitie: ",extindatoarea.getCurrentPosition());
            telemetry.update();
            //extindatoarea.setPower(-gamepad1.right_stick_y);
            //pozitie = Math.max((0,Math.min(1600,pozitie + (int)(10 * gamepad1.right_trigger) - (int)(10 * gamepad1.left_trigger);
            x = pozitie + (int)(10 * gamepad1.right_trigger) - (int)(10 * gamepad1.left_trigger);
            pozitie = Math.max(0,Math.min(1000,x));
        }
    }
}
