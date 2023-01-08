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
public class MainAutoRight extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        double armDrop = 0.749;
        double stringDrop = 0.974;

        double armPicks[] = {2.041, 2.105, 2.145, 2.152, 2.23};
        double stringPicks[] = {0.641, 0.644, 0.642, 0.649, 0.658};

        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;


        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        waitForStart();
        double timeStart = getRuntime();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 68;
        // 80

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(36, -24.25), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingStraight = drive.trajectoryBuilder(traj1.end(),Math.toRadians(164))
                .strafeTo(new Vector2d(36, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory endingLeft = drive.trajectoryBuilder(endingStraight.end(),Math.toRadians(164))
                .strafeTo(new Vector2d(12, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingRight = drive.trajectoryBuilder(endingStraight.end(),Math.toRadians(164))
                .strafeTo(new Vector2d(60, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        double timeout = 25;
        if (tag == 2) {
            timeout = 28;
        }

        waitForStart();

        grabbaClose();
        if (isStopRequested()) return;


        drive.followTrajectoryAsync(traj1);
        double timeElap = getRuntime();
        while(opModeIsActive() && (drive.isBusy() || !armDoneFirst || !stringDone0)){
            if (getRuntime() - timeElap > 0.5) {
                armDoneFirst = armAsync(armDrop + 0.15, true,1);
                stringDone0 = stringAsync(stringDrop);
            }
            drive.update();
        }
//        drive.turn(Math.toRadians(74));
//
//        wristServo.setPosition(0.94);
//        // Goes and drops pre-loaded cone
////        while (opModeIsActive() && (!armDoneFirst || !stringDone0)) {
////            armDoneFirst = armAsync(armDrop + 0.18, true, 1);
////            stringDone0 = stringAsync(stringDrop);
////        }
//
//        boolean stringDoneFirst = false;
//        while (opModeIsActive() && (!armDone0 || !stringDoneFirst)) {
//            armDone0 = armAsync(armDrop, false, .5);
//            stringDoneFirst = stringAsync(stringDrop);
//        }
//        grabbaOpen();
//        sleep(100);
//
//        // Cycles with cones
//        for (int cone = 0; cone < 5; cone++) {
//            if ((getRuntime() - timeStart) >= timeout) {
//                returnHome();
//                grabbaClose();
//                break;
//            }
//            //Grab Align
//            boolean armDone = false;
//            boolean stringDone = false;
//            wristServo.setPosition(0);
//            while (opModeIsActive() && (!armDone || !stringDone)) {
//                armDone = armAsync(armPicks[cone] - 0.5, true, 1);
//                stringDone = stringAsync(stringPicks[cone]);
//                if ((getRuntime() - timeStart) >= timeout) {
//                    returnHome();
//                    grabbaClose();
//                    break;
//                }
//            }
//
//            //Grab
//            boolean armDone2 = false;
//            grabbaOpen();
//            while (opModeIsActive() && !armDone2) {
//                armDone2 = armAsync(armPicks[cone], true, 0.7);
//                if ((getRuntime() - timeStart) >= timeout) {
//                    returnHome();
//                    grabbaClose();
//                    break;
//                }
//            }
////            sleep(100);
//            grabbaClose();
//            sleep(200);
//
//            boolean armDone3 = false;
//
//            // Move to midpoint and flip wrist
//            while (opModeIsActive() && !armDone3) {
//                armDone3 = armAsync(armDrop + 0.6, false, 0.8);
//                if ((getRuntime() - timeStart) >= timeout) {
//                    returnHome();
//                    grabbaClose();
//                    break;
//                }
//            }
//            wristServo.setPosition(0.94);
//
//            boolean armDone4 = false;
//            boolean stringDone4 = false;
//
//            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
//                armDone4 = armAsync(armDrop, true, 0.7);
//                stringDone4 = stringAsync(stringDrop);
//                if ((getRuntime() - timeStart) >= timeout) {
//                    returnHome();
//                    grabbaClose();
//                    break;
//                }
//            }
//            armMotor.setPower(0);
//            stringMotor.setPower(0);
//            grabbaOpen();
//            sleep(200);
//        }
//
////        drive.turn(Math.toRadians(-74));
//
//
//
//        if (tag == 1) {
//            drive.followTrajectory(endingStraight);
//            drive.followTrajectory(endingLeft);
//            packet.put("tag", tag);
//            dashboard.sendTelemetryPacket(packet);
//            motorsStop();
//            sleep(1000);
//        } else if (tag == 2) {
//            packet.put("tag", tag);
//            dashboard.sendTelemetryPacket(packet);
//
//            motorsStop();
//            sleep(1000);
//        } else if (tag == 3) {
//            drive.followTrajectory(endingStraight);
//            drive.followTrajectory(endingRight);
//            packet.put("tag", tag);
//            dashboard.sendTelemetryPacket(packet);
//            motorsStop();
//            sleep(1000);
//        }
    }
}
