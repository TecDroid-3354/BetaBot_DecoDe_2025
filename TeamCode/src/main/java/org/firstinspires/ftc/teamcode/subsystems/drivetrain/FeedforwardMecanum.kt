package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import LinearVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.ToggleButton
import org.firstinspires.ftc.teamcode.utils.velocityMotor.SVACoefficients
import org.firstinspires.ftc.teamcode.utils.velocityMotor.VelocityMotor
import org.firstinspires.ftc.teamcode.utils.velocityMotor.VelocityMotorConfig
import kotlin.math.abs
import kotlin.math.sign


class FeedforwardMecanum(val hw: HardwareMap, val telemetry: Telemetry, val gamepad: Gamepad) : SubsystemBase() {

    lateinit var frontRightMotor: VelocityMotor
    lateinit var frontLeftMotor: VelocityMotor
    lateinit var backRightMotor: VelocityMotor
    lateinit var backLeftMotor: VelocityMotor

    private val frPIDFCoefficients = SVACoefficients(0.05, 0.2, 0.0)
    private val flPIDFCoefficients = SVACoefficients(0.075, 0.2, 0.0)
    private val brPIDFCoefficients = SVACoefficients(0.075, 0.2, 0.0)
    private val blPIDFCoefficients = SVACoefficients(0.05, 0.2, 0.0)



    // Kinematic
    // Locations of the wheels relative to the robot center.
    var frontLeftLocation: Translation2d = Translation2d(0.381, 0.381)
    var frontRightLocation: Translation2d = Translation2d(0.381, -0.381)
    var backLeftLocation: Translation2d = Translation2d(-0.381, 0.381)
    var backRightLocation: Translation2d = Translation2d(-0.381, -0.381)

    // Creating my kinematics object using the wheel locations.
    var kinematics: MecanumDriveKinematics = MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation,
        backLeftLocation, backRightLocation
    )

    var prevVelLeft = 0.0
    var prevVelRight = 0.0

    var maxDelta = 0.05

    var frPower = 0.0
    var flPower = 0.0
    var brPower = 0.0
    var blPower = 0.0

    val yToggleButton = ToggleButton { gamepad.y }
    val bToggleButton = ToggleButton { gamepad.b }
    val xToggleButton = ToggleButton { gamepad.x }
    val aToggleButton = ToggleButton { gamepad.a }
    val startToggleButton = ToggleButton { gamepad.start }


    init {
        motorsConfig()

    }

    fun setMotorsVelocity(velocity: LinearVelocity) {
        frontRightMotor.setVelocity(velocity)
        frontLeftMotor.setVelocity(velocity)
        backRightMotor.setVelocity(velocity)
        backLeftMotor.setVelocity(velocity)
    }

    fun identification() {
        val powerConstant = 0.025

        if (yToggleButton.wasJustPressed()) {
            frPower += powerConstant
        }
        if (bToggleButton.wasJustPressed()) {
            flPower += powerConstant
        }
        if (xToggleButton.wasJustPressed()) {
            brPower += powerConstant
        }
        if (aToggleButton.wasJustPressed()) {
            blPower += powerConstant
        }
        if (startToggleButton.wasJustPressed()) {
            frPower += powerConstant
            flPower += powerConstant
            brPower += powerConstant
            blPower += powerConstant
        }

        frontRightMotor.setPower(frPower)
        frontLeftMotor.setPower(flPower)
        backRightMotor.setPower(brPower)
        backLeftMotor.setPower(blPower)

        telemetry.addData("fr", frPower)
        telemetry.addData("fl", flPower)
        telemetry.addData("br", brPower)
        telemetry.addData("bl", blPower)
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)

        val flVel = smoothVel(prevVelLeft, wheelSpeeds.frontLeftMetersPerSecond)
        val blVel = smoothVel(prevVelLeft, wheelSpeeds.rearLeftMetersPerSecond)
        val frVel = smoothVel(prevVelRight, wheelSpeeds.frontRightMetersPerSecond)
        val brVel = smoothVel(prevVelRight, wheelSpeeds.rearRightMetersPerSecond)

        /*val flVel = wheelSpeeds.frontLeftMetersPerSecond
        val blVel = wheelSpeeds.rearLeftMetersPerSecond
        val frVel = wheelSpeeds.frontRightMetersPerSecond
        val brVel = wheelSpeeds.rearRightMetersPerSecond*/

        prevVelLeft  = (flVel + blVel) / 2.0
        prevVelRight  = (frVel + brVel) / 2.0

        frontRightMotor.setVelocity(LinearVelocity.fromMps(frVel))
        frontLeftMotor.setVelocity(LinearVelocity.fromMps(flVel))
        backRightMotor.setVelocity(LinearVelocity.fromMps(brVel))
        backLeftMotor.setVelocity(LinearVelocity.fromMps(blVel))

        telemetry.addData("frSpeed", frVel)
        telemetry.addData("flSpeed", flVel)
        telemetry.addData("brSpeed", brVel)
        telemetry.addData("blSpeed", blVel)
    }

    // Función para limitar la aceleración
    private fun smoothVel(prev: Double, target: Double): Double {
        val delta = target - prev
        if (abs(delta) > maxDelta) {
            return prev + sign(delta) * maxDelta
        }
        return target
    }
    fun motorsConfig() {
        frontRightMotor = VelocityMotor(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.frId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
                MecanumConstants.Physics.ticksPerRevolution,
                frPIDFCoefficients
            ))

        frontLeftMotor = VelocityMotor(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.flId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
                MecanumConstants.Physics.ticksPerRevolution,
                flPIDFCoefficients
            ))

        backRightMotor = VelocityMotor(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.brId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.FORWARD,
                MecanumConstants.Physics.ticksPerRevolution,
                brPIDFCoefficients
            ))

        backLeftMotor = VelocityMotor(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.blId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
                MecanumConstants.Physics.ticksPerRevolution,
                blPIDFCoefficients
            ))

        frontRightMotor.setCircumference(MecanumConstants.Physics.circumference)
        frontLeftMotor.setCircumference(MecanumConstants.Physics.circumference)
        backRightMotor.setCircumference(MecanumConstants.Physics.circumference)
        backLeftMotor.setCircumference(MecanumConstants.Physics.circumference)

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
    }
}