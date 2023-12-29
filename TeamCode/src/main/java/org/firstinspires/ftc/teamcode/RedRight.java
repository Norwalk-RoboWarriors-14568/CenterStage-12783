package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "RedRight")
public class RedRight extends LinearOpMode {
    Webcam webcam;
    AprilTagTest aprilTag;
    private AutoMethods autoMethods;

    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake, motorHang;
    private double StrafeInches;

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
        Webcam.Position pos = Webcam.Position.Left;
        CameraName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        autoMethods = new AutoMethods(motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang);
        webcam = new Webcam(camera, true);
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
        if (pos == Webcam.Position.Left) {
            aprilTag.setId(4);
            RunLeft(autoMethods);
            StrafeInches = 36;
        }
        else if (pos == Webcam.Position.Right){
            aprilTag.setId(6);
            RunRight(autoMethods);
            StrafeInches = 24;
        }
        else{
            aprilTag.setId(5);
            RunCenter(autoMethods);
            StrafeInches = 30;
        }
        autoMethods.GetToBoard(aprilTag, webcam,0.2);
        autoMethods.StrafeByInch(StrafeInches,true,0.2);

    }

    void RunRight(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(17, 0.5);
        blar.RunMotorHang(6.5, 1);
        blar.StrafeByInch(10, true, 0.4);
        //motorIntake.setPower(-0.4);
        //sleep(1500);
        //motorIntake.setPower(0);
        blar.StrafeByInch(13, true, 0.4);
        blar.Turn90(false, 0.4);
        //blar.StrafeByInch(3, false, 0.4);
        blar.RunMotors(8.25, 0.2);
        //motorHang.setPower(0);
        //blar.RunMotorHang(-6.5,1);
        //blar.RunMotors(-4,0.5);
        //blar.StrafeByInch(18, true, 0.4);
        //sleep(2000);
        motorHang.setPower(0);


    }

    void RunLeft(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(25, 0.4);
        blar.StrafeByInch(13, false, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        blar.RunMotorHang(6.5, 1);
        motorIntake.setPower(0);
        blar.StrafeByInch(46, true, 0.4);
      /*  motorHang.setPower(0);
        blar.Turn90(false, 0.4);
        blar.StrafeByInch(7, false, 0.4);
        blar.RunMotors(4.5, 0.2);
        blar.RunMotorHang(-6.5, 0.75);
        blar.RunMotors(-4, 0.5);
        blar.StrafeByInch(31, true, 0.4);
        sleep(3000);
        motorHang.setPower(0);

       */
    }

    void RunCenter(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(26, 0.4);
        blar.RunMotorHang(6.5, 0.75);
        blar.StrafeByInch(4, true, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.RunMotors(-2, 0.4);
        blar.Turn90(false, 0.4);
        blar.StrafeByInch(3, false, 0.4);
        blar.RunMotors(18, 0.4);
       /* blar.RunMotors(3, 0.2);
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5, 0.75);
        blar.RunMotors(-4, 0.5);
        blar.StrafeByInch(25, true, 0.4);
        sleep(2000);
        motorHang.setPower(0);

        */
    }


}
