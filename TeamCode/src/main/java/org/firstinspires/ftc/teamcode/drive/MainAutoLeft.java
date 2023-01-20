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
public class MainAutoLeft extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        double armDrop = 0.774;
        double stringDrop = 0.96;

        double armPicks[] = {2.01, 2.03, 2.1, 2.118, 2.178};
        double stringPicks[] = {0.787, 0.8, 0.791, 0.791, 0.793};


        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;


        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        grabbaClose();
        waitForStart();

        double timeStart = getRuntime();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 68;
        // 80


        //imu: -148.84170654373366


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-36, -18.75), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(73.75))
                .build();
        TrajectorySequence endingStraight = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-73.75))
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

        double timeout = 25;
        if (tag == 2) {
            timeout = 27.5;
        }

        waitForStart();

        grabbaClose();
        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(traj1);
        double timeElap = getRuntime();
        while(opModeIsActive() && (drive.isBusy() || !armDoneFirst || !stringDone0)){
            if (getRuntime() - timeElap > 0.15) {
                armDoneFirst = armAsync(armDrop + 0.15, true,1);
                stringDone0 = stringAsync(stringDrop);
            }
            drive.update();
        }
//        drive.turn(Math.toRadians(73));
        stringMotor.setPower(0.1);

        wristDrop();


        boolean stringDoneFirst = false;
        while (opModeIsActive() && (!armDone0 || !stringDoneFirst)) {
            armDone0 = armAsync(armDrop, true, 1);
            stringDoneFirst = stringAsync(stringDrop);
        }
        grabbaOpen();
        sleep(100);

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
                armDone = armAsync(armPicks[cone], true, 1);
                stringDone = stringAsync(stringPicks[cone]);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
            }

            sleep(100);
            grabbaClose();
            sleep(300);

            boolean armDone3 = false;

            // Move to midpoint and flip wrist

            while (opModeIsActive() && !armDone3) {
                armDone3 = armAsync(armDrop + 1, false, 1);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
            }
            wristDrop();

            boolean armDone4 = false;
            boolean stringDone4 = false;

            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
                armDone4 = armAsync(armDrop, true, 1);
                stringDone4 = stringAsync(stringDrop);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
            }
            armMotor.setPower(0);
            stringMotor.setPower(0);
            grabbaOpen();
            sleep(200);
        }
        returnMiddle();
//        drive.turn(Math.toRadians(-74));

        if (tag == 1) {
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingLeft);
            motorsStop();
            sleep(1000);
        } else if (tag == 2) {
            drive.turn(Math.toRadians(73));
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
