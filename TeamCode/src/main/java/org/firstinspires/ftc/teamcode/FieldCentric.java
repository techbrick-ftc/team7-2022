package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class FieldCentric extends LinearOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode()  {
        TelemetryPacket packet = new TelemetryPacket();
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor stringMotor = hardwareMap.get(DcMotor.class, "stringMotor");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        Servo grabbaServo = hardwareMap.get(Servo.class, "grabbaServo");
        TouchSensor armuptouch = hardwareMap.get(TouchSensor.class, "armuptouch");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stringMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double zeroAngle = 0;

        double position1 = 0;
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad cur2 = new Gamepad();


        boolean grabberOpen = false;



        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();


        while (opModeIsActive()) {

            try {
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {

            }


            if (gamepad2.right_trigger != 0) {
//                if (stringMotor.getCurrentPosition() < -255000) {
//                    stringMotor.setPower(0);
//                } else {
                    stringMotor.setPower(-gamepad2.right_trigger);
//                }
            }

            dashboard.sendTelemetryPacket(packet);

            if (gamepad2.left_trigger != 0) {
                if (stringMotor.getCurrentPosition() >= 0) {
                    stringMotor.setPower(0);
                } else {
                    stringMotor.setPower(gamepad2.left_trigger);
                }
            }




            if ((armMotor.getCurrentPosition() >= 50000 && gamepad2.left_stick_y<0 ) || (gamepad2.left_stick_y>0 && armuptouch.isPressed())) {
                stringMotor.setPower(0);
            } else {
                armMotor.setPower(gamepad2.left_stick_y);
            }
            telemetry.addData("button", armuptouch.isPressed());


            if (cur2.a && !previousGamepad2.a) {
                if (!grabberOpen) {
                    grabbaServo.setPosition(1);
                    grabberOpen = true;
                }
                else {
                    grabbaServo.setPosition(0);
                    grabberOpen = false;
                }
            }


            if (cur2.dpad_right && !previousGamepad2.dpad_right){
                position1 += 0.10;

            }
            if (cur2.dpad_left && !previousGamepad2.dpad_left){
                position1 -= 0.10;

            }

            if (cur2.dpad_up && !previousGamepad2.dpad_up){
                position1 = 1;

            }

            if (cur2.dpad_down && !previousGamepad2.dpad_down){
                position1 = 0;
            }

            if (position1 > 1){
                position1 = 1;
            }
           else if (position1 < 0){
                position1 = 0;
            }
            wristServo.setPosition(position1);


            telemetry.addData("position", wristServo.getPosition());
            telemetry.addData("end", position1);

            telemetry.update();


            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle - zeroAngle;

            if (gamepad1.y) {
                zeroAngle = -imu.getAngularOrientation().firstAngle;
            }

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator;
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;

            frontRight.setPower(frontRightPower);  //front
            frontLeft.setPower(frontLeftPower);    //left
            backRight.setPower(backRightPower);   //right
            backLeft.setPower(backLeftPower);   //back


            try {
                previousGamepad2.copy(cur2);
            } catch (RobotCoreException e) {

            }
        }

    }
}
