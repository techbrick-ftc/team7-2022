package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.lang.Math;

@TeleOp(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {
    public DcMotor left;
    public DcMotor back;
    public DcMotor front;
    public DcMotor right;
    public BNO055IMU imu;


    void drivingCorrectionStraight(double startAngle2, double power) {
       double difference = imu.getAngularOrientation().firstAngle - startAngle2;
       right.setPower(power + difference);
       left.setPower(power - difference);
    }

    void drivingCorrectionLeft(double startAngle, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle;
        front.setPower(power + difference);
        back.setPower(power - difference);
    }


    private double wrap(double theta) {
        double newTheta = theta;
        while(Math.abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }
    boolean shouldStopTurning(double targetAngle) {
        double currentAngle = imu.getAngularOrientation().firstAngle;
        return Math.abs(currentAngle - targetAngle) < .005 * Math.PI;

    }


    void turnRobot(double angle, double speed, boolean clockwise) {
        double directionalSpeed = clockwise ? -speed: speed;
        double targetAngle = wrap(imu.getAngularOrientation().firstAngle + angle);
        while(opModeIsActive() && !shouldStopTurning(targetAngle)) {
            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            back.setPower(directionalSpeed);
            front.setPower(-directionalSpeed);
            right.setPower(-directionalSpeed);
            left.setPower(directionalSpeed);
        }

        back.setPower(0);
        front.setPower(0);
        right.setPower(0);
        left.setPower(0);

    }

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

        double startAngle = imu.getAngularOrientation().firstAngle;

        //Continue moving until robot is on tape
       while(opModeIsActive() && color1.red() <750){
           drivingCorrectionLeft(startAngle, 0.5);
       }

       back.setPower(0);
       front.setPower(0);
        sleep(1000);


        double startAngle2 = imu.getAngularOrientation().firstAngle;

        right.setPower(0.5);
        left.setPower(0.5);
        sleep(1000);

        double startTime = getRuntime();
        //Continue moving until robot is on tape
        while (opModeIsActive() && color1.red()  < 750 || getRuntime() - startTime < 1 ){
            drivingCorrectionStraight(startAngle2, 0.5);
        }

        right.setPower(0);
        left.setPower(0);
        sleep(1000);

        turnRobot(-Math.PI/2, 0.3, true);

        }
    }