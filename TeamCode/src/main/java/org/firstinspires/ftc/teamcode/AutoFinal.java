package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Disabled
@Autonomous(name = "AutoFinal", group = "Auto")
public class AutoFinal extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public BNO055IMU imu;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    void drivingCorrectionStraight(double startAngle2, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle2;
        backRight.setPower(power + difference);
        frontLeft.setPower(power - difference);
    }

    void drivingCorrectionLeft(double startAngle, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle;
        frontRight.setPower(power + difference);
        backLeft.setPower(power - difference);
    }


    private double wrap(double theta) {
        double newTheta = theta;
        while (Math.abs(newTheta) > Math.PI) {
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
        double directionalSpeed = clockwise ? -speed : speed;
        double targetAngle = wrap(imu.getAngularOrientation().firstAngle + angle);
        while (opModeIsActive() && !shouldStopTurning(targetAngle)) {
            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            backLeft.setPower(directionalSpeed);
            frontRight.setPower(-directionalSpeed);
            backRight.setPower(-directionalSpeed);
            frontLeft.setPower(directionalSpeed);
        }

        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);

    }

    @Override
    public void runOpMode() {

        TelemetryPacket packet = new TelemetryPacket();
        frontLeft = hardwareMap.get(DcMotor.class, "left");
        backLeft = hardwareMap.get(DcMotor.class, "back");
        frontRight = hardwareMap.get(DcMotor.class, "front");
        backRight = hardwareMap.get(DcMotor.class, "right");
        ColorSensor colorFR = hardwareMap.colorSensor.get("colorFR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        waitForStart();

        double startAngle = imu.getAngularOrientation().firstAngle;

        //  packet.addLine("Ticks Before " + front.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        int frontpos = frontRight.getCurrentPosition();
        //Continue moving until robot is on tape
        while (opModeIsActive() && colorFR.red() < 400 && frontRight.getCurrentPosition() - frontpos < 5500) {
            drivingCorrectionLeft(startAngle, 0.5);
            //  packet.addLine("Red Color " + colorFR.red());
            packet.put("Red", colorFR.red());
            packet.put("Ticks", frontRight.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        packet.addLine("Ticks After " + frontRight.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
        backLeft.setPower(0);
        frontRight.setPower(0);
        sleep(1000);


        double startAngle2 = imu.getAngularOrientation().firstAngle;

        backRight.setPower(0.5);
        frontLeft.setPower(0.5);


        double startTime = getRuntime();
        //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
        // this is so robot is for sure off the tape
        int leftpos = frontLeft.getCurrentPosition();

        while (opModeIsActive() && ((colorFR.red() < 400 && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
            drivingCorrectionStraight(startAngle2, 0.5);
            // packet.addLine("Runtime" + (getRuntime() - startTime));
            //  packet.addLine("Color Red" + colorFR.red());
            packet.put("Red", colorFR.red());
            packet.put("Ticks", frontLeft.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        backRight.setPower(0);
        frontLeft.setPower(0);
        sleep(1000);

        turnRobot(-Math.PI / 2, 0.3, true);

        while(opModeIsActive()){


        }
    }

    }
