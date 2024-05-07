package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "RedLeft")
public class RedLeft extends LinearOpMode {
    Webcam webcam;
    private AutoMethods autoMethods;
    AprilTagTest aprilTag;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake, motorHang;
    private boolean FindTagStrafe;
    ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);
        motorLeft = hardwareMap.dcMotor.get("front_Left");
        motorRight = hardwareMap.dcMotor.get("front_Right");
        motorLeft2 = hardwareMap.dcMotor.get("back_Left");
        motorRight2 = hardwareMap.dcMotor.get("back_Right");
        motorIntake = hardwareMap.dcMotor.get("Intake");
        motorHang = hardwareMap.dcMotor.get("Hanger");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Webcam.Position pos = Webcam.Position.Left;
        CameraName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        RevBlinkinLedDriver ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        autoMethods = new AutoMethods(motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang, telemetry, ledDriver, -1);
        webcam = new Webcam(hardwareMap.get(WebcamName.class, "Webcam 1"), true);
        aprilTag = new AprilTagTest(camera);
        webcam.visionPortal.setProcessorEnabled(webcam.tfod, true);
        webcam.visionPortal.setProcessorEnabled(webcam.tagProcessor, false);
        autoMethods.Color(78);
        while(!opModeIsActive()){

             pos = webcam.CheckCamera();

            telemetry.addData("detected x", pos);
            telemetry.update();
            sleep (2000);

            if(pos == Webcam.Position.Left){
                autoMethods.Color(85);
            }
            else if (pos == Webcam.Position.Right){
                autoMethods.Color(96);
            }
            else{
                autoMethods.Color(87);
            }

        }
        webcam.visionPortal.setProcessorEnabled(webcam.tfod, false);
        webcam.visionPortal.setProcessorEnabled(webcam.tagProcessor, true);
        waitForStart();
        if(pos == Webcam.Position.Left){
            aprilTag.setId(4);
            RunLeft(autoMethods);
            FindTagStrafe = true;
        }
        else if (pos == Webcam.Position.Right){
            aprilTag.setId(6);
            RunRight(autoMethods);
            FindTagStrafe = true;
        }
        else{
            aprilTag.setId(5);
            RunCenter(autoMethods);
            FindTagStrafe = false;
        }
        autoMethods.GetToBoard(aprilTag, webcam,0.2,FindTagStrafe);
        sleep(3000);
        color.enableLed(false);
        autoMethods.Color(100);
    }
    void RunRight(AutoMethods blar) throws InterruptedException {
        //sleep(5000);
        blar.RunMotors(20,0.4);
        blar.Turn90(false, 0.3);
        blar.StrafeByInch(11,false,0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(18, false, 0.4);
        blar.RunMotorHang(6.5,0.75);
        blar.RunMotors(72,0.4);
        /*blar.StrafeByInch(31, true, 0.2);
        motorHang.setPower(0);
        blar.RunMotors(5, 0.2);
        blar.RunMotorHang(-6.5,1);
        blar.RunMotors(-4,0.5);
        sleep(4000);
        motorHang.setPower(0);

        */


    }
    void RunLeft(AutoMethods blar) throws InterruptedException {
        //sleep(5000);
        blar.RunMotors(17,0.4);
        blar.StrafeByInch(12, false, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(12, true, 0.4);
        blar.RunMotors(32,0.4);
        blar.Turn90(false, 0.3);
        blar.RunMotorHang(6.5,1);
        blar.RunMotors(69, 0.4);
        /*blar.StrafeByInch(14, true, 0.2);
        blar.RunMotors(4,0.2);
        blar.RunMotors(-4,0.2);
        blar.RunMotorHang(-6.5,0.75);
        sleep(5000);
        motorHang.setPower(0);

         */
    }
    void RunCenter(AutoMethods blar) throws InterruptedException {
        sleep(5000);
        blar.RunMotors(25,0.4);
        blar.StrafeByInch(4, true, 0.2);
        motorIntake.setPower(-0.45);
        sleep(1500);
        motorIntake.setPower(0);
        blar.RunMotorHang(6.5,0.4);
        blar.StrafeByInch(72, true, 0.2);
        blar.Turn90(false, 0.2);
        /*//blar.StrafeByInch(4, false, 0.2);
        blar.RunMotors(9, 0.2);
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,1);
        blar.RunMotors(-4,0.5);
        blar.ZeroMotors();
        sleep(4000);
        motorHang.setPower(0);

         */
    }

}
