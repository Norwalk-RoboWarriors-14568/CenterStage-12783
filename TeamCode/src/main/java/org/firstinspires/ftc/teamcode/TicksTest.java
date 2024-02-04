package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * All the code aside from importing, goes within a class file - essentially telling Android Studio-
 * to run this using java
 */
@TeleOp(name = "TicksTest")
public class TicksTest extends OpMode{
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2;

    @Override
    public void init() {

//rev hub 1
        motorLeft = hardwareMap.dcMotor.get("front_Left");
        motorRight = hardwareMap.dcMotor.get("front_Right");
        motorLeft2 = hardwareMap.dcMotor.get("back_Left");
        motorRight2 = hardwareMap.dcMotor.get("back_Right");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        telemetry.addData("motorLeftTicks",motorLeft.getCurrentPosition());
        telemetry.addData("motorRightTicks",motorRight.getCurrentPosition());
        telemetry.addData("motorLeft2Ticks",motorLeft2.getCurrentPosition());
        telemetry.addData("motorRight2Ticks",motorRight2.getCurrentPosition());
        telemetry.update();

    }
    }


