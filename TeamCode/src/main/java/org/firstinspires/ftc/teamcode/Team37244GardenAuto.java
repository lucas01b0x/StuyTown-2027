package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// This is a simple auton where the bot shoots the preload and drives towards the loading zone

@Autonomous(name = "Team37244GardenAuto", group = "Testing")
public class Team37244GardenAuto extends LinearOpMode {
    Team37244Robot robot;

    // Autonomous timing
    private static final double AUTO_TIMEOUT_SECONDS = 29.0;
    private static final double FORWARD_TIME_SECONDS = 1;

    // Timers
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Autonomous states
    private enum AutoState {
        DRIVE_FORWARD,
        LOADING_ZONE_PARK,
        COMPLETE
    }

    private AutoState currentState = AutoState.DRIVE_FORWARD;

    @Override
    public void runOpMode() {
        robot = new Team37244Robot();

        robot.init(hardwareMap);

        // Ensure mechanisms start stopped
        robot.stopAllMotorsAndMechanisms();

        telemetry.addLine("Autonomous initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.stopAllMotorsAndMechanisms();
            return;
        }

        // Reset timers at the beginning of auton
        currentState = AutoState.DRIVE_FORWARD;
        autoTimer.reset();
        stateTimer.reset();

        while (opModeIsActive()) {
            // Stops before the 30-second limit
            if (autoTimer.seconds() >= AUTO_TIMEOUT_SECONDS) {
                currentState = AutoState.COMPLETE;
            }

            switch (currentState) {
                case DRIVE_FORWARD:
                    robot.fieldCentricDrive(1, 0, 0);
                    if (stateTimer.seconds() >= FORWARD_TIME_SECONDS) {
                        robot.stopDriveMotors();
                        stateTimer.reset();
                        currentState = AutoState.LOADING_ZONE_PARK;
                    }
                    break;

                case LOADING_ZONE_PARK:
                    robot.fieldCentricDrive(1, -0.3, 0);
                    if (stateTimer.seconds() >= FORWARD_TIME_SECONDS) {
                        robot.stopDriveMotors();
                        stateTimer.reset();
                        currentState = AutoState.COMPLETE;
                    }
                    break;

                case COMPLETE:
                    robot.stopAllMotorsAndMechanisms();
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData(
                    "Autonomous Time",
                    "%.2f", autoTimer.seconds());
            telemetry.update();

            // Exit once the autonomous routine is complete
            if (currentState == AutoState.COMPLETE) {
                robot.stopAllMotorsAndMechanisms();
                break;
            }
        }

        // Final safety stop
        robot.stopAllMotorsAndMechanisms();
    }
}