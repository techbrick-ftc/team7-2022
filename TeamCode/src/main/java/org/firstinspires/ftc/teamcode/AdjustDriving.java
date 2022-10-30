package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.lang.Math;

@TeleOp(name = "AdjustDriving", group = "Test")
public class AdjustDriving extends LinearOpMode {
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
        ColorSensor color1 = hardwareMap.colorSensor.get("color1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.REVERSE);


        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        waitForStart();

        double start = getRuntime();
        double startAngle = imu.getAngularOrientation().firstAngle;

        while(opModeIsActive() && getRuntime() - start < 10) {
            telemetry.addData("Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();

            double difference = imu.getAngularOrientation().firstAngle - startAngle;


            right.setPower(0.5 + difference);
            left.setPower(0.5 - difference);

        }






    }
}