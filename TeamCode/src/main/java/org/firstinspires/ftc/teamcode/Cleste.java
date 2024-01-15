package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Hardware.Constante.ClesteApasat1;
import static org.firstinspires.ftc.teamcode.Hardware.Constante.ClesteLasat1;
import static org.firstinspires.ftc.teamcode.Hardware.Constante.ClesteLasat2;
import static org.firstinspires.ftc.teamcode.Hardware.Constante.ClesteApasat2;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Constante;


//@TeleOp
public class Cleste extends LinearOpMode
{
    Servo servo1; //dreptul
    Servo servo2; //stangul
    private Constante constants;
    double pozitie1 = ClesteLasat1;
    double pozitie2 = ClesteLasat2;
    boolean stareServo1 = false;
    boolean stareServo2 = false;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("pozitie1: ", servo1.getPosition());
            telemetry.addData("pozitie2: ", servo2.getPosition());
            telemetry.update();

            if(gamepad1.a) {
                SchimbareStareCleste1();
            }
            if(gamepad1.b) {
                SchimbareStareCleste2();
            }

            //if (gamepad1.right_stick_y!=0){
            //    pozitie = -gamepad1.right_stick_y;
            //}
            servo1.setPosition(pozitie1);
            servo2.setPosition(pozitie2);
        }
    }
    void SchimbareStareCleste1() {
        if (stareServo1 == false) {
            stareServo1 = true;
            pozitie1 = ClesteApasat1;
        }
        if (stareServo1 == true) {
            stareServo1 = false;
            pozitie1 = ClesteLasat1;
        }
    }
    void SchimbareStareCleste2() {
        if (stareServo2 == false) {
            stareServo2 = true;
            pozitie2 = ClesteApasat2;
        }
        if (stareServo2 == true) {
            stareServo2 = false;
            pozitie2 = ClesteLasat2;
        }
    }
}

