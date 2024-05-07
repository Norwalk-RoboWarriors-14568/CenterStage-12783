package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoMethods {
    double TicsPerRevolution = 537.6;
    double Circumference = 11.87;
    double HangInchesPerRev = 0.25;
    double TPI = TicsPerRevolution / Circumference;
    double StrafeTPI = 50.2512563;
    double TicksPerDeg = 10.17;
    double DegAtBoard = -7.5;
    private DcMotor motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang;
    private int leftTarget,left2Target,rightTarget,right2Target;
    private Telemetry telemetry;
    private double currentPitch;
    private double xOffset;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public AutoMethods(DcMotor left, DcMotor left2, DcMotor right, DcMotor right2, DcMotor inTake, DcMotor hang, Telemetry telemetryIn, RevBlinkinLedDriver blinkinLedDriverIn, double xOffSetIn) {
        motorLeft = left;
        motorLeft2 = left2;
        motorRight = right;
        motorRight2 = right2;
        motorIntake = inTake;
        motorHang = hang;
        telemetry = telemetryIn;
        xOffset = xOffSetIn;
        blinkinLedDriver = blinkinLedDriverIn;
    }

    int StrafeInchesToTicks(double inches) {
        return (int) (inches * StrafeTPI);
    }

    void StrafeByInch(double inches, boolean sendright, double motorPower) throws InterruptedException {
        int Ticks = StrafeInchesToTicks(inches);
        if (sendright) {
            right2Target = motorRight2.getCurrentPosition() - Ticks;
            rightTarget = motorRight.getCurrentPosition() + Ticks;
            left2Target = motorLeft2.getCurrentPosition() + Ticks;
            leftTarget = motorLeft.getCurrentPosition() - Ticks;
        } else {
            right2Target = motorRight2.getCurrentPosition() + Ticks;
            rightTarget = motorRight.getCurrentPosition() - Ticks;
            left2Target = motorLeft2.getCurrentPosition() - Ticks;
            leftTarget = motorLeft.getCurrentPosition() + Ticks;
        }
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();
    }

    void Strafe( boolean sendright, double motorPower){
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (sendright) {
            motorLeft.setPower(-motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(-motorPower);
            motorLeft2.setPower(motorPower);
        } else {
            motorLeft.setPower(motorPower);
            motorRight.setPower(-motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(-motorPower);
        }

    }

    void Turn90( boolean turnLeft, double motorPower)    {
        int Ticks = 915 ;
        if (turnLeft) {
            right2Target = motorRight2.getCurrentPosition() + Ticks;
            rightTarget = motorRight.getCurrentPosition() + Ticks;
            left2Target = motorLeft2.getCurrentPosition() - Ticks;
            leftTarget = motorLeft.getCurrentPosition() - Ticks;
        } else {
            right2Target = motorRight2.getCurrentPosition() - Ticks;
            rightTarget = motorRight.getCurrentPosition() - Ticks;
            left2Target = motorLeft2.getCurrentPosition() + Ticks;
            leftTarget = motorLeft.getCurrentPosition() + Ticks;
        }
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();
    }

    //TPI is ticks per inches
    int ConvertInchesToTicks(double inches) {
        return (int) (inches * TPI);
    }
    int ConvertInchesToTicksForHang(double inches) { return (int) (inches * 306 / HangInchesPerRev); }

    void RunMotors(double inches, double motorPower) {
        int Ticks = ConvertInchesToTicks(inches);
        leftTarget = motorLeft.getCurrentPosition() + Ticks;
        left2Target = motorLeft2.getCurrentPosition() + Ticks;
        rightTarget = motorRight.getCurrentPosition() + Ticks;
        right2Target = motorRight2.getCurrentPosition() + Ticks;
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();
    }

    void RunMotorHang(double inches, double hangPower) {
        int Ticks = ConvertInchesToTicksForHang(inches);
        motorHang.setTargetPosition(motorHang.getCurrentPosition() + Ticks);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(hangPower);
    }
    void ZeroMotors(){
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight2.setPower(0);
        motorLeft2.setPower(0);
    }
    private boolean AtTarget() {
        return motorLeft.getCurrentPosition() > leftTarget - 10 && motorLeft.getCurrentPosition() < leftTarget + 10 &&
                motorLeft2.getCurrentPosition() > left2Target - 10 && motorLeft2.getCurrentPosition() < left2Target + 10 &&
                motorRight.getCurrentPosition() > rightTarget - 10 && motorRight.getCurrentPosition() < rightTarget + 10 &&
                motorRight2.getCurrentPosition() > right2Target - 10 && motorRight2.getCurrentPosition() < right2Target + 10;
    }

    void FixPitch( double motorPower){
        double degree = currentPitch - DegAtBoard;
        int angleTicks = Math.abs((int)( degree * TicksPerDeg));
        boolean turnRight = degree <= 0;
        telemetry.addData("pitch: ", currentPitch);
        telemetry.addData("DAB: ", DegAtBoard);
        telemetry.addData("AT: ", angleTicks);
        telemetry.addData("TR: ", turnRight);
        telemetry.update();
        if(turnRight){
            leftTarget = motorLeft.getCurrentPosition() + angleTicks;
            left2Target = motorLeft2.getCurrentPosition() + angleTicks;
            rightTarget = motorRight.getCurrentPosition() - angleTicks;
            right2Target = motorRight2.getCurrentPosition() - angleTicks;

        }
        else {
            leftTarget = motorLeft.getCurrentPosition() - angleTicks;
            left2Target = motorLeft2.getCurrentPosition() - angleTicks;
            rightTarget = motorRight.getCurrentPosition() + angleTicks;
            right2Target = motorRight2.getCurrentPosition() + angleTicks;
        }
        motorLeft.setTargetPosition(leftTarget);
        motorRight2.setTargetPosition(right2Target);
        motorLeft2.setTargetPosition(left2Target);
        motorRight.setTargetPosition(rightTarget);

        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!AtTarget()) {
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);
            motorRight2.setPower(motorPower);
            motorLeft2.setPower(motorPower);
        }
        ZeroMotors();



    }

    void SquareOnTag (double x, double yaw, double motorPower){
        int rightTicks, right2Ticks, leftTicks, left2Ticks;
        int strafeTick = Math.abs(StrafeInchesToTicks(x - xOffset));
        double degree = yaw - DegAtBoard;
        int angleTicks = Math.abs((int)(degree * TicksPerDeg));
        boolean strafeRight = x - xOffset >= 0;
        boolean turnRight = degree <= 0;
        if(strafeRight && turnRight){
            right2Ticks =  -strafeTick - angleTicks;
            rightTicks =  strafeTick - angleTicks;
            left2Ticks = strafeTick + angleTicks;
            leftTicks = -strafeTick + angleTicks;
        }
        else if(strafeRight){
            right2Ticks = -strafeTick + angleTicks;
            rightTicks = strafeTick + angleTicks;
            left2Ticks = strafeTick - angleTicks;
            leftTicks = -strafeTick - angleTicks;
        }
        else if(turnRight){
            right2Ticks = strafeTick - angleTicks;
            rightTicks = -strafeTick - angleTicks;
            left2Ticks = -strafeTick + angleTicks;
            leftTicks = strafeTick + angleTicks;
        }
        else{
            right2Ticks = strafeTick + angleTicks;
            rightTicks = -strafeTick + angleTicks;
            left2Ticks = -strafeTick - angleTicks;
            leftTicks = strafeTick - angleTicks;
        }
        int maxTicks = Math.max(Math.abs(left2Ticks), Math.max(Math.abs(leftTicks), Math.max(Math.abs(right2Ticks), Math.abs(rightTicks))));
        leftTarget = motorLeft.getCurrentPosition() + leftTicks;
        left2Target = motorLeft2.getCurrentPosition() + left2Ticks;
        rightTarget = motorRight.getCurrentPosition() + rightTicks;
        right2Target = motorRight2.getCurrentPosition() + right2Ticks;
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double LMP = ((double)Math.abs(leftTicks) / maxTicks) * 0.5;
        double RMP = ((double)Math.abs(rightTicks) / maxTicks) * 0.5;
        double R2MP = ((double)Math.abs(right2Ticks) / maxTicks) * 0.5;
        double L2MP = ((double)Math.abs(left2Ticks) / maxTicks) * 0.5;
        while(!AtTarget()) {
            motorLeft.setPower(LMP);
            motorRight.setPower(RMP);
            motorRight2.setPower(R2MP);
            motorLeft2.setPower(L2MP);
            telemetry.addData("FR - CP - TP - P: ",Integer.toString(motorRight.getCurrentPosition()) + " " + Integer.toString(motorRight.getTargetPosition()) + " " + RMP);
            telemetry.addData("BR - CP - TP - P: ",Integer.toString(motorRight2.getCurrentPosition()) + " " + Integer.toString(motorRight2.getTargetPosition()) + " " + R2MP);
            telemetry.addData("FL - CP - TP - P: ",Integer.toString(motorLeft.getCurrentPosition()) + " " + Integer.toString(motorLeft.getTargetPosition()) + " " + LMP);
            telemetry.addData("BL - CP - TP - P: ",Integer.toString(motorLeft2.getCurrentPosition()) + " " + Integer.toString(motorLeft2.getTargetPosition()) + " " + L2MP);
            telemetry.addData("Max Ticks", Integer.toString(maxTicks) + " " + rightTicks + " " + right2Ticks + " " + leftTicks + " " + left2Ticks);
            telemetry.addData("ST", strafeTick);
            telemetry.addData("AT", angleTicks);
            telemetry.addData("DAB", Double.toString(degree) +  " " + yaw);
            telemetry.update();
        }
        ZeroMotors();
    }
    void GetToBoard(AprilTagTest aprilTag, Webcam webcam, double motorPower, boolean strafeRight)throws InterruptedException {
        GetToBoard(aprilTag, webcam, motorPower, strafeRight, 7);
    }
    void GetToBoard(AprilTagTest aprilTag, Webcam webcam, double motorPower, boolean strafeRight, double yOffSet)throws InterruptedException{
        //double ct = motorLeft.getCurrentPosition();
        AprilTagTest.TagLocation location = null;
        AprilTagTest.TagLocation tempLocation = null;
        boolean tagFound = false;
        // && Math.abs(motorLeft.getCurrentPosition() - ct) < 450
        while(!tagFound) {
            if (location != null && !tagFound) {
                tagFound = true;
            }
            tempLocation = aprilTag.GetPositon(webcam.tagProcessor);
            if (tempLocation != null && (tempLocation.yaw < 0 || tempLocation.yaw > -15)) location = tempLocation;
            Strafe(strafeRight, motorPower);
        }
        ZeroMotors();
        if(tagFound) {
            SquareOnTag(location.x, location.yaw, motorPower);
            //StrafeByInch(location.x - XOffSet, true, motorPower);
            location = null;
            while (location == null) {
                location = aprilTag.GetPositon(webcam.tagProcessor);
            }
        /*currentYaw = location.pitch;
        RunMotors(location.y - 12.5,0.2);
        location = aprilTag.GetPositon(webcam.tagProcessor);
        FixPitch(0.2);
        location = aprilTag.GetPositon(webcam.tagProcessor);
        StrafeByInch(location.x - XOffSet, true, motorPower);
        location = aprilTag.GetPositon(webcam.tagProcessor);

        */
            //FixPitch(location.pitch, motorPower);
            //location = aprilTag.GetPositon(webcam.tagProcessor);
            RunMotors(location.y - yOffSet, motorPower);
            RunMotors(-2, motorPower);
            RunMotorHang(-6.5, 1);
        }
    }

    void Drive (double driveInches, double driveStrafe, double turnpower, int maxTicks, double motorPower){
        double coe = 1;
        double driveCoe = driveInches * coe;
        double C = Math.sqrt(Math.pow(driveInches,2) + Math.pow(driveStrafe,2));
        double angle = driveStrafe < 0 ? Math.PI + Math.atan(driveCoe/driveStrafe):Math.atan(driveCoe/driveStrafe);
        //Math.signum(driveInches);
        double LMP = Math.sin(angle - (Math.PI)/4);
        double RMP = Math.cos(angle - (Math.PI)/4);
        //LMP *= -1;
        //L2MP *=  -1;
        double maxPower = Math.max(Math.abs(LMP),Math.abs(RMP));
        LMP = (LMP/maxPower)*motorPower;
        RMP = (RMP/maxPower)*motorPower;
        double L2MP = RMP;
        double R2MP = LMP;
        int startTicks = motorLeft.getCurrentPosition();
        int currentTicks = motorLeft.getCurrentPosition();
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(Math.abs(currentTicks-startTicks) < maxTicks){
            motorLeft.setPower(LMP);
            motorRight.setPower(RMP);
            motorRight2.setPower(R2MP);
            motorLeft2.setPower(L2MP);
            currentTicks = motorLeft.getCurrentPosition();
            telemetry.addData("LMP", LMP);
            telemetry.addData("L2MP", L2MP);
            telemetry.addData("RMP", RMP);
            telemetry.addData("R2MP", R2MP);
            telemetry.addData("Angle", angle);
            telemetry.addData("Start", startTicks);
            telemetry.addData("Current", currentTicks);
            telemetry.addData("LMPC", motorLeft.getCurrentPosition());
            telemetry.addData("L2MPC", motorLeft2.getCurrentPosition());
            telemetry.addData("RMPC", motorRight.getCurrentPosition());
            telemetry.addData("R2MPC", motorRight2.getCurrentPosition());
            telemetry.update();
        }
        ZeroMotors();

    }
    private int GetLateralTicks(double driveInches, double driveStrafe){
        return StrafeInchesToTicks(driveStrafe) + ConvertInchesToTicks(driveInches);
    }
    public void Color(int color){
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(color);
        blinkinLedDriver.setPattern(pattern);
    }


}
