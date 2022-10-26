package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

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
        ColorSensor color1 = hardwareMap.colorSensor.get("color1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        waitForStart();

        // Move to the left
        back.setPower(0.5);
        front.setPower(-0.5);

        //Continue moving until robot is on tape
        while(opModeIsActive() && color1.red() <750);

        back.setPower(0);
        front.setPower(0);
        sleep(1000);

        // Move forward
        right.setPower(-0.5);
        left.setPower(0.5);
        sleep(1000);

        //Continue moving until robot is on tape
        while (opModeIsActive() && color1.red() < 750 );

        right.setPower(0);
        left.setPower(0);
        sleep(1000);



    }
    }
