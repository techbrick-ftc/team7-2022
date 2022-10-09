package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {
    public DcMotor left;
    public DcMotor back;
    public DcMotor front;
    public DcMotor right;
    public BNO055IMU imu;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        back = hardwareMap.get(DcMotor.class, "back");
        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

        waitForStart();
        while (opModeIsActive()) {
            drive(-0.3, 0, 0);
        }

        left.setPower(0);
        back.setPower(0);
        front.setPower(0);
        right.setPower(0);

    }

    public void drive(double x, double y, double turn) {
        double currentAngle = imu.getAngularOrientation().firstAngle;
        double rotatedX = x * Math.cos(-currentAngle) - y * Math.sin(-currentAngle);
        double rotatedY = y * Math.cos(currentAngle) - x * Math.sin(currentAngle);

        front.setPower(rotatedY - rotatedX + turn);
        left.setPower(rotatedY + rotatedX - turn);
        back.setPower(rotatedY + rotatedX + turn);
        right.setPower(rotatedY - rotatedX - turn);

        telemetry.addData("Current Angle", currentAngle);
        telemetry.addData("Rotated X", rotatedX);
        telemetry.addData("Rotated Y", rotatedY);
        telemetry.update();
    }

}