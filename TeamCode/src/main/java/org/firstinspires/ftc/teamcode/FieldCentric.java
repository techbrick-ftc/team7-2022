package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;@TeleOp

public class FieldCentric extends LinearOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode()  {
        TelemetryPacket packet = new TelemetryPacket();
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor back = hardwareMap.get(DcMotor.class, "back");
        DcMotor front = hardwareMap.get(DcMotor.class, "front");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        DcMotor stringMotor = hardwareMap.get(DcMotor.class, "stringMotor");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stringMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double zeroAngle = 0;





        left.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad2.right_trigger != 0) {
                if (stringMotor.getCurrentPosition() < -2550) {
                    stringMotor.setPower(0);
                } else {
                    stringMotor.setPower(-gamepad2.right_trigger);
                }
            }
            packet.put("Left Trigger", gamepad2.left_trigger);
            packet.put("position", stringMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            if (gamepad2.left_trigger != 0) {
                if (stringMotor.getCurrentPosition() >= 0) {
                    stringMotor.setPower(0);
                } else {
                    stringMotor.setPower(gamepad2.left_trigger);
                }
            }


            packet.put("LeftStickY", gamepad2.left_stick_y);
            packet.put("position ARM", armMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            if ((armMotor.getCurrentPosition() >= 50000 && gamepad2.left_stick_y<0) || (armMotor.getCurrentPosition() <= -500000 && gamepad2.left_stick_y>0)) {
                stringMotor.setPower(0);
            } else {
                    armMotor.setPower(gamepad2.left_stick_y);
                }



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

            front.setPower(frontRightPower);
            left.setPower(frontLeftPower);
            right.setPower(backRightPower);
            back.setPower(backLeftPower);
        }
    }
}
