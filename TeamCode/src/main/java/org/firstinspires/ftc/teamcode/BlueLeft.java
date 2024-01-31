package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode {
    Webcam webcam;
    private AutoMethods autoMethods;
    AprilTagTest aprilTag;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake, motorHang;
    private double StrafeInches;
    private boolean FindTagStrafe;

    @Override
    public void runOpMode() throws InterruptedException {
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
        Webcam.Position pos= Webcam.Position.Left;
        CameraName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        autoMethods = new AutoMethods(motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang, telemetry,-0.5);
        webcam = new Webcam(hardwareMap.get(WebcamName.class, "Webcam 1"), false);
        aprilTag = new AprilTagTest(camera);
        webcam.visionPortal.setProcessorEnabled(webcam.tfod, true);
        webcam.visionPortal.setProcessorEnabled(webcam.tagProcessor, false);
        while(!opModeIsActive()) {

            pos = webcam.CheckCamera();

            telemetry.addData("detected x", pos);
            telemetry.update();
            sleep (2000);


        }
        webcam.visionPortal.setProcessorEnabled(webcam.tfod, false);
        webcam.visionPortal.setProcessorEnabled(webcam.tagProcessor, true);
        if(pos == Webcam.Position.Left){
            aprilTag.setId(1);
            RunLeft(autoMethods);
            StrafeInches = 24;
            FindTagStrafe = false;
        }
        else if (pos == Webcam.Position.Right){
            aprilTag.setId(3);
            RunRight(autoMethods);
            StrafeInches = 30;
            FindTagStrafe = true;
        }
        else{
            aprilTag.setId(2);
            RunCenter(autoMethods);
            StrafeInches = 30;
            FindTagStrafe = false;
        }
        autoMethods.GetToBoard(aprilTag, webcam,0.2,FindTagStrafe);
        autoMethods.StrafeByInch(StrafeInches,false,0.2);
        sleep(3000);
        //autoMethods.GetWhitePixel(aprilTag, webcam, false, StrafeInches , 0.4, 9);
    }
    void RunLeft(AutoMethods blar) throws InterruptedException {
        autoMethods.Drive(17,11,0,0.3);
        /*blar.RunMotors(17,0.4);
        blar.RunMotorHang(6.5,1);
        blar.StrafeByInch(11, false, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(21,false,0.4);
        blar.Turn90(true, 0.3);
        motorHang.setPower(0);
        /*blar.StrafeByInch(3, true, 0.4);
        blar.RunMotors(11, 0.2);

        blar.RunMotorHang(-6.5,1);
        blar.RunMotors(-4,0.5);
        blar.StrafeByInch(20, false, 0.4);
        sleep(2000);
        motorHang.setPower(0);

         */




    }
    void RunRight(AutoMethods blar) throws InterruptedException {
        blar.RunMotorHang(6.5,0.75);
        blar.RunMotors(25,0.4);
        blar.StrafeByInch(10, true, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(42,false,0.4);
        blar.Turn90(true, 0.3);
        motorHang.setPower(0);
        /*
        blar.StrafeByInch(7, true, 0.4);
        blar.RunMotors(3.5, 0.2);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        blar.StrafeByInch(32, false, 0.4);
        sleep(3000);
        motorHang.setPower(0);

        */
    }
    void RunCenter(AutoMethods blar) throws InterruptedException {
        blar.RunMotorHang(6.5,0.75);
        blar.RunMotors(26,0.4);
        blar.StrafeByInch(4, false, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.RunMotors(-2,0.4);
        blar.Turn90(true, 0.3);
        motorHang.setPower(0);
        blar.RunMotors(24, 0.4);
        /*blar.StrafeByInch(3, true, 0.4);
        motorHang.setPower(0);
        blar.RunMotors(5, 0.2);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        blar.StrafeByInch(25, false, 0.4);
        sleep(2000);
        motorHang.setPower(0);

         */
    }
}
