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


        double startAngle = imu.getAngularOrientation().firstAngle;

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
            double ticksbefore = frontRight.getCurrentPosition();
            //GET NEW VALUES OF COLOR BL
            startAngle = imu.getAngularOrientation().firstAngle;
         while(opModeIsActive() && ((colorBL.red() < 600  && colorBL.blue() < 500) && frontRight.getCurrentPosition() - ticksbefore < TICKSPERBLOCK * 1.2)){
             drivingCorrectionLeft(startAngle, 0.3);
//
          }

            motorsStop();
            sleep(1000);
            startAngle = imu.getAngularOrientation().firstAngle;
            ticksbefore = backRight.getCurrentPosition();
            double ticksbefore2 = frontLeft.getCurrentPosition();
            while(opModeIsActive() && ((backRight.getCurrentPosition() - ticksbefore < 1.2 * TICKSPERBLOCK) && (frontLeft.getCurrentPosition() - ticksbefore < 1.2 * TICKSPERBLOCK))){
                drivingCorrectionStraight(startAngle, 0.3);
            }

            motorsStop();
            sleep(1000);


        }
        else if (tag == 2){

            startAngle = imu.getAngularOrientation().firstAngle;
            double ticksBefore = frontRight.getCurrentPosition();
            while(opModeIsActive() && frontRight.getCurrentPosition() - ticksBefore < TICKSPERBLOCK * 1.25){
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                frontRight.setPower(0.5);
                frontLeft.setPower(0.5);
            }
            motorsStop();

        }
        else if (tag == 3){
            startAngle = imu.getAngularOrientation().firstAngle;
            double time = getRuntime();

            while(opModeIsActive() && getRuntime() - time < 2) {
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
                frontRight.setPower(-0.5);
                frontLeft.setPower(0.5);
            }


            startAngle = imu.getAngularOrientation().firstAngle;
            double ticksbefore = frontRight.getCurrentPosition();
            double ticksbefore2 = backLeft.getCurrentPosition();
            while(opModeIsActive() && ((frontRight.getCurrentPosition() - ticksbefore < TICKSPERBLOCK * 1.1) && (backLeft.getCurrentPosition() - ticksbefore2 < TICKSPERBLOCK * 1.1))){
                drivingCorrectionStraight(startAngle, 0.5);
            }
            motorsStop();
        }

    }



}

