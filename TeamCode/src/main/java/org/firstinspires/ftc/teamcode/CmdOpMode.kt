package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum


// Personally, I chose to run my code using a command-based Op Mode since it works better for me
// In a regular LinearOpMode, processes are executed in a sequential workflow
// In an OpMode, on the other hand, code is executed through loops

/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 */
@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */

    // Declaring subsystems
    lateinit var mecanum: SolversMecanum

    // Declaring useful components
    lateinit var controller: GamepadEx
    lateinit var imu: IMU
    lateinit var otos: SparkFunOTOS
    lateinit var revHubOrientation: RevHubOrientationOnRobot

    // Initializing the robot
    override fun initialize() {
        // Here, declare code to be executed right after pressing the INIT button
        // Including, for example, declaration of subsystems, configuring the IMU, etc.

        // REV Hub IMU declaration
        otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        imu = hardwareMap.get(IMU::class.java, "imu")
        imu.resetYaw()

        revHubOrientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )

        imu.initialize(IMU.Parameters(revHubOrientation))

        /* Subsystem initialization */

        // Initializing the mecanum
        mecanum = SolversMecanum(hardwareMap, telemetry)
        mecanum.defaultCommand = JoystickCmd(
            { -controller.leftX },
            { controller.leftY },
            { -controller.rightX },
            { getRobotYaw(AngleUnit.DEGREES) },
            mecanum
        )

        // Initializing controller & button bindings
        controller = GamepadEx(gamepad1)
        configureButtonBindings()
    }

    /* ! FUNCTIONAL CODE ! */
    fun getRobotYaw(angleUnit: AngleUnit): Double = imu.robotYawPitchRollAngles.getYaw(angleUnit)

    fun configureButtonBindings() {
        GamepadButton(controller, GamepadKeys.Button.START)
            .whenPressed(InstantCommand({
                imu.resetYaw()
            }))
    }

    fun periodic() {
        /*telemetry.addData("frontRight", mecanum.frontRightMotor.getLinearVelocity().mps)
        telemetry.addData("frontLeft", mecanum.frontLeftMotor.getLinearVelocity().mps)
        telemetry.addData("backRight", mecanum.backRightMotor.getLinearVelocity().mps)
        telemetry.addData("backLeft", mecanum.backLeftMotor.getLinearVelocity().mps)*/

        // Telemetry to retrieve useful data
        telemetry.addData("RobotYaw", getRobotYaw(AngleUnit.DEGREES))
        telemetry.addData("X", otos.position.x)
        telemetry.addData("Y", otos.position.y)
        telemetry.addData("H", otos.position.h)
    }


    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()
            periodic()

            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}