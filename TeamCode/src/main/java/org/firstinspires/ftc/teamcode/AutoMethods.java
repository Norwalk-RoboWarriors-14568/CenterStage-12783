package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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

    public AutoMethods(DcMotor left, DcMotor left2, DcMotor right, DcMotor right2, DcMotor inTake, DcMotor hang, Telemetry telemetryIn, double xOffSetIn) {
        motorLeft = left;
        motorLeft2 = left2;
        motorRight = right;
        motorRight2 = right2;
        motorIntake = inTake;
        motorHang = hang;
        telemetry = telemetryIn;
        xOffset = xOffSetIn;
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

    void GetToBoard(AprilTagTest aprilTag, Webcam webcam, double motorPower, boolean strafeRight)throws InterruptedException{
        AprilTagTest.TagLocation location = null;
        AprilTagTest.TagLocation tempLocation = null;
        boolean tagFound = false;
        while(!tagFound) {
            if (location != null && !tagFound) {
                tagFound = true;
            }
            tempLocation = aprilTag.GetPositon(webcam.tagProcessor);
            if (tempLocation != null && (tempLocation.yaw < 0 || tempLocation.yaw > -15)) location = tempLocation;
            Strafe(strafeRight, motorPower);
        }
        ZeroMotors();
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
        RunMotors(location.y - 7,motorPower);
        RunMotors(-2,motorPower);
        RunMotorHang(-6.5,1);
    }
    void GetWhitePixel(AprilTagTest aprilTag ,Webcam webcam, boolean strafeRight, double strafeInches, double motorPower, int tagNeeded) throws InterruptedException{
        StrafeByInch(strafeInches, strafeRight, motorPower);
        FixPitch( motorPower);
        RunMotors(-105,motorPower);
        Turn90(false,motorPower);
        Turn90(false,motorPower);
        motorIntake.setPower(1);
        StrafeByInch(24,strafeRight,motorPower);
        aprilTag.setId(tagNeeded);
        AprilTagTest.TagLocation location = null;
        boolean tagFound = false;
        while(!tagFound) {
            if (location != null && !tagFound) {
                tagFound = true;
            }
            location = aprilTag.GetPositon(webcam.tagProcessor);
            RunMotors(-0.25, 0.1);
            //Strafe(strafeRight, motorPower);
        }
        motorIntake.setPower(0);
        ZeroMotors();
        /* StrafeByInch(location.x, true, motorPower);
        location = aprilTag.GetPositon(webcam.tagProcessor);
        FixPitch(location.pitch, motorPower);
        location = aprilTag.GetPositon(webcam.tagProcessor);
        motorIntake.setPower(0.4);
        RunMotors(location.y - 2.5,motorPower);
        motorIntake.setPower(0);
        RunMotors(-4,motorPower);
        Turn90(strafeRight,0.2);
        Turn90(strafeRight,0.2);
        StrafeByInch(strafeInches,strafeRight,motorPower );
        RunMotors(108,motorPower);
        motorIntake.setPower(0.4);

         */
    }
    void Drive (double driveInches, double driveStrafe, double motorPower){
        int rightTicks, right2Ticks, leftTicks, left2Ticks;
        int strafeTick = Math.abs(StrafeInchesToTicks(driveStrafe));
        //double degree = degreeTurn;
        //int angleTicks = Math.abs((int)(degree * TicksPerDeg));
        boolean strafeRight = driveStrafe >= 0;
        //boolean turnRight = degree <= 0;
        int leftLateralTicks = GetLateralTicks(driveInches, strafeRight ? -driveStrafe : driveStrafe);
        int left2LateralTicks = GetLateralTicks(driveInches, strafeRight ? driveStrafe : -driveStrafe);
        int rightLateralTicks = GetLateralTicks(driveInches, strafeRight ? driveStrafe : -driveStrafe);
        int right2LateralTicks = GetLateralTicks(driveInches, strafeRight ? -driveStrafe : driveStrafe);
        int maxLateralTicks = Math.max(Math.abs(left2LateralTicks), Math.max(Math.abs(leftLateralTicks), Math.max(Math.abs(right2LateralTicks), Math.abs(rightLateralTicks))));
        //double anglePower = (double)angleTicks / maxLateralTicks;
        /*right2Ticks = ConvertInchesToTicks(driveInches);
        rightTicks = ConvertInchesToTicks(driveInches);
        left2Ticks = ConvertInchesToTicks(driveInches);
        leftTicks = ConvertInchesToTicks(driveInches);



        if (strafeRight) {
            right2Ticks =  right2LateralTicks - angleTicks;
            rightTicks =  rightLateralTicks - angleTicks;
            left2Ticks = left2LateralTicks + angleTicks;
            leftTicks = leftLateralTicks + angleTicks;
        }
        else{
            right2Ticks =  right2LateralTicks + angleTicks;
            rightTicks =  rightLateralTicks + angleTicks;
            left2Ticks = left2LateralTicks - angleTicks;
            leftTicks = leftLateralTicks - angleTicks;
        }

        if(strafeRight && turnRight){
            right2Ticks +=  -strafeTick - angleTicks;
            rightTicks +=  strafeTick - angleTicks;
            left2Ticks += strafeTick + angleTicks;
            leftTicks += -strafeTick + angleTicks;
        }
        else if(strafeRight){
            right2Ticks += -strafeTick + angleTicks;
            rightTicks += strafeTick + angleTicks;
            left2Ticks += strafeTick - angleTicks;
            leftTicks += -strafeTick - angleTicks;
        }
        else if(turnRight){
            right2Ticks += strafeTick - angleTicks;
            rightTicks += -strafeTick - angleTicks;
            left2Ticks += -strafeTick + angleTicks;
            leftTicks += strafeTick + angleTicks;
        }
        else{
            right2Ticks += strafeTick + angleTicks;
            rightTicks += -strafeTick + angleTicks;
            left2Ticks += -strafeTick - angleTicks;
            leftTicks += strafeTick - angleTicks;
        }

         */
        //int maxTicks = Math.max(Math.abs(left2Ticks), Math.max(Math.abs(leftTicks), Math.max(Math.abs(right2Ticks), Math.abs(rightTicks))));
        double leftLatPower = ((double)leftLateralTicks/maxLateralTicks) * motorPower;
        double left2LatPower = ((double)left2LateralTicks/maxLateralTicks) * motorPower;
        double rightLatPower = ((double)rightLateralTicks/maxLateralTicks) * motorPower;
        double right2LatPower = ((double)right2LateralTicks/maxLateralTicks) * motorPower;
        leftTarget = motorLeft.getCurrentPosition() + leftLateralTicks;
        left2Target = motorLeft2.getCurrentPosition() + left2LateralTicks;
        rightTarget = motorRight.getCurrentPosition() + rightLateralTicks;
        right2Target = motorRight2.getCurrentPosition() + right2LateralTicks;
        motorRight2.setTargetPosition(right2Target);
        motorRight.setTargetPosition(rightTarget);
        motorLeft2.setTargetPosition(left2Target);
        motorLeft.setTargetPosition(leftTarget);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*double LMP = ((double)Math.abs(leftTicks) / maxTicks) * motorPower;
        double RMP = ((double)Math.abs(rightTicks) / maxTicks) * motorPower;
        double R2MP = ((double)Math.abs(right2Ticks) / maxTicks) * motorPower;
        double L2MP = ((double)Math.abs(left2Ticks) / maxTicks) * motorPower;

         */
        //double LMP = Math.abs(leftLatPower + (turnRight ? anglePower : -anglePower));
        //double RMP = Math.abs(rightLatPower + (turnRight ? -anglePower : anglePower));
        //double L2MP = Math.abs(left2LatPower + (turnRight ? anglePower : -anglePower));
        //double R2MP = Math.abs(right2LatPower + (turnRight ? -anglePower : anglePower));
        //double maxPower = Math.max(Math.abs(LMP), Math.max(Math.abs(L2MP), Math.max(Math.abs(R2MP), Math.abs(RMP))));
        //LMP = (LMP/maxPower) * motorPower;
        //RMP = (RMP/maxPower) * motorPower;
        //L2MP = (L2MP/maxPower) * motorPower;
        //R2MP = (R2MP/maxPower) * motorPower;

        while(!AtTarget()) {
            motorLeft.setPower(leftLatPower);
            motorRight.setPower(rightLatPower);
            motorRight2.setPower(right2LatPower);
            motorLeft2.setPower(left2LatPower);
            telemetry.addData("FR - CP - TP - P: ",Integer.toString(motorRight.getCurrentPosition()) + " " + Integer.toString(motorRight.getTargetPosition()) + " " + rightLatPower);
            telemetry.addData("BR - CP - TP - P: ",Integer.toString(motorRight2.getCurrentPosition()) + " " + Integer.toString(motorRight2.getTargetPosition()) + " " + right2LatPower);
            telemetry.addData("FL - CP - TP - P: ",Integer.toString(motorLeft.getCurrentPosition()) + " " + Integer.toString(motorLeft.getTargetPosition()) + " " + leftLatPower);
            telemetry.addData("BL - CP - TP - P: ",Integer.toString(motorLeft2.getCurrentPosition()) + " " + Integer.toString(motorLeft2.getTargetPosition()) + " " + left2LatPower);
            telemetry.addData("Max Ticks", Integer.toString(maxLateralTicks) + " " + rightLateralTicks + " " + right2LateralTicks + " " + leftLateralTicks + " " + left2LateralTicks);
            telemetry.addData("ST", strafeTick);
            telemetry.update();
        }
        ZeroMotors();
    }
    private int GetLateralTicks(double driveInches, double driveStrafe){
        return StrafeInchesToTicks(driveStrafe) + ConvertInchesToTicks(driveInches);
    }

}
