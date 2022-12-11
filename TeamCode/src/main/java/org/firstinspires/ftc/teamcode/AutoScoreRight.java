package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous
public class AutoScoreRight extends StarterAuto {
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
        double backLeftTicks = 0;

        waitForStart();
        int tag = getAprilTag(5);

        telemetry.addData("got tag", tag);
        telemetry.update();

        scoreMiddlePole();


        // reposition to straight against wall
        frontLeft.setPower(-0.50);
        backLeft.setPower(-0.50);
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

        if (tag == 1){

            startAngle = imu.getAngularOrientation().firstAngle;
            frontRightTicks = frontRight.getCurrentPosition();
            //GET NEW VALUES OF COLOR BL
            startAngle = imu.getAngularOrientation().firstAngle;
           timeElap = getRuntime();
         while(opModeIsActive() && ((colorBR.red() < 400  && colorBR.blue() < 300) && frontRight.getCurrentPosition() - frontRightTicks < TICKSPERBLOCK * 1.2) && getRuntime() - timeElap < 2){
             drivingCorrectionLeft(startAngle, 0.3);
//
          }

            motorsStop();
            sleep(300);


             timeElap = getRuntime();
            backRightTicks = backRight.getCurrentPosition();
            while(opModeIsActive() && (getRuntime() - timeElap < 1) && backRight.getCurrentPosition() - backRightTicks < TICKSPERBLOCK * 0.3){
                backRight.setPower(-0.5);
                backLeft.setPower(-0.5);
                frontLeft.setPower(-0.5);
                frontRight.setPower(-0.5);
                telemetry.addData("time", getRuntime() - timeElap);
                telemetry.update();
            }



            motorsStop();
            sleep(500);


//            ticksbefore = frontRight.getCurrentPosition();
//           time2 = getRuntime();
//            while(opModeIsActive() && (frontRight.getCurrentPosition() - ticksbefore < TICKSPERBLOCK * 0.2) && getRuntime() - time2 < 1){
//                drivingCorrectionLeft(startAngle, -0.3);
//            }
//            motorsStop();
//            sleep(500);

            telemetry.addData("finised right", frontRight.getCurrentPosition());

            backRightTicks = backRight.getCurrentPosition();
            frontLeftTicks = frontLeft.getCurrentPosition();
            timeElap = getRuntime();
            while(opModeIsActive() && ((backRight.getCurrentPosition() - backRightTicks < 1.2 * TICKSPERBLOCK) && (frontLeft.getCurrentPosition() - frontLeftTicks < 1.2 * TICKSPERBLOCK)) && getRuntime() - timeElap < 2){
                drivingCorrectionStraight(startAngle, 0.3);
            }

            motorsStop();

            telemetry.addData("all done", backRight.getCurrentPosition());
            telemetry.update();
        }
        else if (tag == 2){

            startAngle = imu.getAngularOrientation().firstAngle;
            frontRightTicks = frontRight.getCurrentPosition();
            while(opModeIsActive() && frontRight.getCurrentPosition() - frontRightTicks < TICKSPERBLOCK * 1.25){
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                frontRight.setPower(0.5);
                frontLeft.setPower(0.5);
            }
            motorsStop();

        }
        else if (tag == 3){
            startAngle = imu.getAngularOrientation().firstAngle;
            timeElap = getRuntime();

            while(opModeIsActive() && getRuntime() - timeElap < 2) {
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
                frontRight.setPower(-0.5);
                frontLeft.setPower(0.5);
            }


            startAngle = imu.getAngularOrientation().firstAngle;
            frontRightTicks = frontRight.getCurrentPosition();
            backLeftTicks = backLeft.getCurrentPosition();
            while(opModeIsActive() && ((frontRight.getCurrentPosition() - frontRightTicks < TICKSPERBLOCK * 1.1) && (backLeft.getCurrentPosition() - backLeftTicks < TICKSPERBLOCK * 1.1))){
                drivingCorrectionStraight(startAngle, 0.5);
            }
            motorsStop();
        }

    }



}

