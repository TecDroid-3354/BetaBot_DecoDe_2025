package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum
import kotlin.math.pow

@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {
    lateinit var mecanum: Mecanum
    lateinit var colorSensorEx: ColorSensorEx
    override fun initialize() {
        mecanum = Mecanum(hardwareMap, telemetry)
        colorSensorEx = ColorSensorEx(hardwareMap.get(ColorSensor::class.java, "colorSensor"), telemetry)

        configureButtonBindings()

    }

    fun configureButtonBindings() {

    }

    fun periodic() {

        val speeds = ChassisSpeeds( -gamepad1.left_stick_y.toDouble().pow(3.0) * 1.5,
            -gamepad1.left_stick_x.toDouble().pow(3.0) * 2.0,
            -gamepad1.right_stick_x.toDouble().pow(3.0) * 20.0)
        mecanum.setChassisSpeeds(speeds)

        telemetry.addData("Detected Color", colorSensorEx.colorFromSensor)

        telemetry.update()
    }


    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize()

        waitForStart()

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            periodic()

            run()
        }
        reset()
    }
}