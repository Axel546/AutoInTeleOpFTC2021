package org.firstinspires.ftc.teamcode.autonomytest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Autonomie4Discuri", group = "autonomy")
public class Autonomie4Discuri extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Pose2d(30, 30, 0))
//                .build();

        Trajectory trajPowershot = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(63, 12), 0)
                .build();

        Trajectory putAwayWobble1 = new TrajectoryBuilder(trajPowershot.end(), drive.constraints)
                .splineTo(new Vector2d(110,-10),Math.toRadians(-90))
                .splineTo(new Vector2d(53,-20),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(50,-12),Math.toRadians(180))
                .splineTo(new Vector2d(48,0),Math.toRadians(100))
                .splineTo(new Vector2d(38,-11),Math.toRadians(180))
                .build();

        Trajectory takeWobble = new TrajectoryBuilder(putAwayWobble1.end(),drive.constraints)
                .strafeTo(new Vector2d(35,-11))
                .build();
        Trajectory trajShoot = new TrajectoryBuilder(takeWobble.end().plus(new Pose2d(0,0,Math.toRadians(180))), drive.constraints)
                .splineTo(new Vector2d(63,-8),0)
                .build();

        Trajectory putAwayWobble2 = new TrajectoryBuilder(trajShoot.end(), drive.constraints)
                .splineTo(new Vector2d(100,-20),0)
                .build();

        Trajectory parkRobot = new TrajectoryBuilder(putAwayWobble2.end(), drive.constraints)
                .strafeTo(new Vector2d(65,0))
                .build();

        /*Trajectory takeRings = new TrajectoryBuilder(putAwayWobble1.end().plus(new Pose2d(0,0,Math.toRadians(5))), drive.constraints)
                .splineTo(new Vector2d(48,0),Math.toRadians(100))
                .splineTo(new Vector2d(38,-11),Math.toRadians(180))
                .build();
         */

        /*Trajectory littleBack = new TrajectoryBuilder(putAwayWobble1.end(), drive.constraints)
                .strafeTo(new Vector2d(69,-8))
                .build();

        Trajectory returnToBase = new TrajectoryBuilder(littleBack.end(), drive.constraints)
                .splineTo(new Vector2d(45,-11),Math.toRadians(180))
                .build();

        Trajectory littleFront = new TrajectoryBuilder(returnToBase.end(), drive.constraints)
                .strafeTo(new Vector2d(30,-11))
                .build();

        Trajectory putAwayWobble2 = new TrajectoryBuilder(littleFront.end().plus(new Pose2d(0,0,Math.toRadians(-180))), drive.constraints)
                .splineTo(new Vector2d(63,-22.5),0)
                .build();

        Trajectory trajShoot = new TrajectoryBuilder(putAwayWobble2.end(), drive.constraints)
                .back(3)
                .splineToConstantHeading(new Vector2d(63,-8),0)
                .build();

        Trajectory parkRobot = new TrajectoryBuilder(trajShoot.end(), drive.constraints)
                .strafeTo(new Vector2d(65,-8))
                .build();

         */


        drive.followTrajectory(trajPowershot);
        sleep(500);
        drive.turn(Math.toRadians(-2));
        sleep(500);
        drive.turn(Math.toRadians(-2));
        sleep(500);
        drive.followTrajectory(putAwayWobble1);
        drive.followTrajectory(takeWobble);
        sleep(200);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(trajShoot);
        sleep(2000);
        drive.followTrajectory(putAwayWobble2);
        drive.followTrajectory(parkRobot);
        //drive.followTrajectory(takeRings);
        //drive.turn(Math.toRadians(-10));

        /*drive.followTrajectory(littleBack);
        drive.followTrajectory(returnToBase);
        drive.followTrajectory(littleFront);
        sleep(500);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(putAwayWobble2);
        sleep(500);
        drive.followTrajectory(trajShoot);
        sleep(500);
        drive.followTrajectory(parkRobot);

         */

//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(0)))
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );
    }
}
