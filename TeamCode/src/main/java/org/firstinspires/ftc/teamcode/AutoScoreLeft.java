package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous
public class AutoScoreLeft extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        initialize();
        initAprilTags();
        imuAngle();

        double startAngle = imu.getAngularOrientation().firstAngle;
        double timeElap = 0;
        double frontRightTicks = 0;
        double frontLeftTicks = 0;
        double backRightTicks = 0;

        waitForStart();


        int tag = getAprilTag(5);

       //Score at the middle height pole
        scoreMiddlePole();

        // reposition to straight against wall
        frontRight.setPower(-0.50);
        backRight.setPower(-0.50);
        sleep(700);

        motorsStop();

        frontLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        sleep(1000);
        motorsStop();
        // reposition

        sleep(1000);

        if (tag == 1) {

            startAngle = imu.getAngularOrientation().firstAngle;
            timeElap = getRuntime();

            // Move left for 2 seconds
            while (opModeIsActive() && getRuntime() - timeElap < 2) {
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
                frontRight.setPower(0.5);
                frontLeft.setPower(-0.5);
            }

            // Move forward for 1.1 blocks
            startAngle = imu.getAngularOrientation().firstAngle;
            frontRightTicks = frontRight.getCurrentPosition();
            while (opModeIsActive() && frontRight.getCurrentPosition() - frontRightTicks < TICKSPERBLOCK * 1.1) {
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                frontRight.setPower(0.5);
                frontLeft.setPower(0.5);
            }
            motorsStop();

        } else if (tag == 2) {

            // Move straight for 1.25 blocks then stop
            startAngle = imu.getAngularOrientation().firstAngle;
            frontRightTicks = frontRight.getCurrentPosition();
            while (opModeIsActive() && frontRight.getCurrentPosition() - frontRightTicks < TICKSPERBLOCK * 1.25) {
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                frontRight.setPower(0.5);
                frontLeft.setPower(0.5);
            }
            motorsStop();

        } else if (tag == 3) {
            // Move right until hit diagonal color line
            startAngle = imu.getAngularOrientation().firstAngle;
            frontRightTicks = frontRight.getCurrentPosition();
            timeElap = getRuntime();
            while (opModeIsActive() && ((colorBR.red() < 400 && colorBR.blue() < 400) && frontRight.getCurrentPosition() - frontRightTicks < TICKSPERBLOCK) && getRuntime() - timeElap < 2) {
                drivingCorrectionLeft(startAngle, -0.3);
            }
            motorsStop();
            sleep(1000);


            // Move backwards for 1 second to reposition against wall
            timeElap = getRuntime();
            backRightTicks = backRight.getCurrentPosition();
            while (opModeIsActive() && (getRuntime() - timeElap < 1)) {
                drivingCorrectionStraight(startAngle, -0.3);
            }

            // Move back left to make sure youre on the colored line
//            frontRightTicks = frontRight.getCurrentPosition();
//            timeElap = getRuntime();
//            while(opModeIsActive() && ((colorBR.red() < 400  && colorBR.blue() < 400) && getRuntime() - timeElap < 1)){
//                drivingCorrectionLeft(startAngle, 0.3);
//            }
            motorsStop();
            sleep(500);

            // Move straight or 1.2 blocks until in the correct box
            timeElap = getRuntime();
            frontLeftTicks = frontLeft.getCurrentPosition();
            backRightTicks = backRight.getCurrentPosition();
            while (opModeIsActive() && ((backRight.getCurrentPosition() - backRightTicks < 1.2 * TICKSPERBLOCK) && (frontLeft.getCurrentPosition() - frontLeftTicks < 1.2 * TICKSPERBLOCK)) && getRuntime() - timeElap < 2) {
                drivingCorrectionStraight(startAngle, 0.3);
            }

            motorsStop();
            sleep(1000);
        }

    }

}

