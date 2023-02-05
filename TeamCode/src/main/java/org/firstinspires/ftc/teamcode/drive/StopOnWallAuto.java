package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.StarterAuto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class StopOnWallAuto extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        double armDrop = 0.761;
        double stringDrop = 1.719;

        double stringPickUp = 1.56;

        double armPicks[] = {2.02, 2.055, 2.13, 2.14, 2.202};


        double stringWallLength = 1.55;
        double armHeightWall = 1.88;


        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;

        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        grabbaClose();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 68;
        // 80

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-36, -18.75), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(73))
                .build();
        TrajectorySequence endingStraight = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-73))
                .strafeTo(new Vector2d(-36, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingLeft = drive.trajectoryBuilder(endingStraight.end())
                .strafeTo(new Vector2d(-12, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingRight = drive.trajectoryBuilder(endingStraight.end())
                .strafeTo(new Vector2d(-60, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();
        telemetry.addData("string pot", stringpot.getVoltage());
        telemetry.addData("armpot", armpot.getVoltage());
        telemetry.update();

        int tag = getAprilTag(5);
        packet.put("APRIL CONE", tag);
        dashboard.sendTelemetryPacket(packet);

        double timeout = 24.5;
        if (tag == 2) {
            timeout = 27.5;
        }

        double timeStart = getRuntime();

        drive.followTrajectorySequenceAsync(traj1);


        double timeElap = getRuntime();
        while (opModeIsActive() && (drive.isBusy() || !armDoneFirst || !stringDone0)) {
            if (getRuntime() - timeElap > 0.15) {
                armDoneFirst = armAsync(armDrop + 0.15, true, 1);
                stringDone0 = stringAsync(stringDrop);
            }
            drive.update();
        }
        stringMotor.setPower(0.1);

        wristDrop();

        boolean stringDoneFirst = false;
        while (opModeIsActive() && (!armDone0 || !stringDoneFirst)) {
            armDone0 = armAsync(armDrop, true, 1);
            stringDoneFirst = stringAsync(stringDrop);
        }
        grabbaOpen();
        sleep(200);

       //sleep(10000); // GRAB VALUES

        // Cycles with cones
        cycleloop:
        for (int cone = 0; cone < 5; cone++) {
            if ((getRuntime() - timeStart) >= timeout) {
                break cycleloop;
            }
            //Grab Align
            boolean armDone = false;
            boolean stringDone = false;
            wristPick();
            grabbaOpen();

            while (opModeIsActive() && (!armDone || !stringDone)) {
                //Go down to above cones, string inside wall
                armDone = armAsync(armHeightWall, false, 1, 0.4, 0.03, 0.02);
                stringDone = stringAsync(stringWallLength+0.35);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
                if (Math.abs(Math.toDegrees(imu.getAngularOrientation().secondAngle)) > 10) {
                    requestOpModeStop();
                }
            }
            packet.addLine("im up here");

            sleep(150);
            // Move string motor until pot values haven't changed - hitting the wall -
            double stringPotLastVal = stringpot.getVoltage();
            stringDone = false;
            while(opModeIsActive() && !stringDone ) {
                stringDone = stringAsync(stringPickUp);
            }
            packet.addLine("im down here");

            stringMotor.setPower(0);

            armDone = false;
            while(opModeIsActive() && !armDone){
                armDone = armAsync(armPicks[cone], true, 1);
            }

            // grab cone
            sleep(100);
            grabbaClose();
            sleep(300);

            packet.put("status",1);
            dashboard.sendTelemetryPacket(packet);
            // Move to midpoint and flip wrist
            boolean armDone3 = false;
            while (opModeIsActive() && !armDone3) {
                packet.put("status", 2);
                packet.put("stringpot", stringpot.getVoltage());
                packet.put("armpot", armpot.getVoltage());
                dashboard.sendTelemetryPacket(packet);
                armDone3 = armAsync(armDrop + 1, false, 1);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
                if (Math.abs(Math.toDegrees(imu.getAngularOrientation().secondAngle)) > 10) {
                    requestOpModeStop();
                }
            }
            packet.put("status", 3);
            dashboard.sendTelemetryPacket(packet);
            wristDrop();

            boolean armDone4 = false;
            boolean stringDone4 = false;

            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
                armDone4 = armAsync(armDrop, true, 1);
                stringDone4 = stringAsync(stringDrop);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
                if (Math.abs(Math.toDegrees(imu.getAngularOrientation().secondAngle)) > 10) {
                    requestOpModeStop();
                }
            }
            armMotor.setPower(0);
            stringMotor.setPower(0);
            grabbaOpen();
            sleep(200);
            dashboard.sendTelemetryPacket(packet);
        }

        if (armpot.getVoltage() < 0.766) {
            grabbaOpen();
        }

        wristDrop();
        returnMiddle();
//        drive.turn(Math.toRadians(-74));

        if (tag == 1) {
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingLeft);
            motorsStop();
            sleep(1000);
        } else if (tag == 2) {
            drive.turn(Math.toRadians(-74));
            motorsStop();
            sleep(1000);
        } else if (tag == 3) {
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingRight);
            motorsStop();
            sleep(1000);
        }
    }
}

