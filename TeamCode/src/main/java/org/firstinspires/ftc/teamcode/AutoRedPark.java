package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous
public class AutoRedPark extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        initialize();
        initAprilTags();
        packet.addLine("id before");
        dashboard.sendTelemetryPacket(packet);

            // 400 per foot

        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;

        waitForStart();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        if (tag == 1){


            int frontpos = frontRight.getCurrentPosition();
            //Continue moving until robot is on tape
            while (opModeIsActive() && tapeSensor45(true) && frontRight.getCurrentPosition() - frontpos < 5500) {
                drivingCorrectionLeft(startAngle, 0.5);

            }

            motorsStop();
            sleep(1000);


            // loop until strafe you strafe a little right, want to make sure you dont hit stack of cups
            startAngle = imu.getAngularOrientation().firstAngle;
            int backleftpos = backLeft.getCurrentPosition();
            while(opModeIsActive() && backLeft.getCurrentPosition() - backleftpos < TICKSPERBLOCK * 0.05){
                drivingCorrectionLeft(startAngle, -0.5);
            }
            motorsStop();
            sleep(500);

            drivingCorrectionStraight(startAngle, 0.5);


            double startTime = getRuntime();
            //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
            // this is so robot is for sure off the tape
            int leftpos = frontLeft.getCurrentPosition();

            while (opModeIsActive() && ((tapeSensor90(true) && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
                drivingCorrectionStraight(startAngle, 0.5);
            }

            motorsStop();
            sleep(1000);

        }
        else if (tag == 2){
            telemetry.addLine("tag is 2!");


            int frontpos = frontRight.getCurrentPosition();
            //Continue moving until robot is on tape
            while (opModeIsActive() && tapeSensor45(true) && (frontRight.getCurrentPosition() - frontpos < 5500)) {
                drivingCorrectionLeft(startAngle, 0.5);

            }

            motorsStop();
            sleep(1000);


            drivingCorrectionStraight(startAngle, 0.5);


            double startTime = getRuntime();
            //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
            // this is so robot is for sure off the tape
            int leftpos = frontLeft.getCurrentPosition();

            while (opModeIsActive() && ((tapeSensor90(true) && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
                drivingCorrectionStraight(startAngle, 0.5);
            }

           motorsStop();
            sleep(1000);

            drivingCorrectionLeft(startAngle, -0.5);
            sleep(1000);

        }
        else if (tag == 3){

            while(opModeIsActive() && colorBR.red() < 1000){
                drivingCorrectionLeft(startAngle, -0.5);
            }
            motorsStop();
            sleep(1000);


            while(opModeIsActive() && backRight.getCurrentPosition() < 1.5 * TICKSPERBLOCK){
                drivingCorrectionStraight(startAngle, 0.5);
            }

            motorsStop();
            sleep(1000);
        }

    }

}
