package org.firstinspires.ftc.teamcode.utils.positionMotorEx

import Angle
import AngularVelocity
import Distance
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.hardware.motors.Motor

data class PositionMotorExConfig(
    val zeroPowerBehavior: Motor.ZeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT,
    val direction: Motor.Direction = Motor.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients,
    var gearRatio: Double = 1.0,
    val powerThreshold: Double = 0.01
)

interface IPositionMotorEx {
    /* ! CONFIG METHODS ! */
    var config: PositionMotorExConfig
    fun applyConfig()
    fun applyConfig(config: PositionMotorExConfig)

    /* ! FUNCTIONAL METHODS ! */
    fun setPower(power: Double)
    fun setPosition(setPoint: Angle)
    fun setPosition(distance: Distance, circumference: Distance)
    fun stopMotor()

    /* ! SETTER METHODS ! */
    fun setGearRatio(gearRatio: Double)
    fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients)
    fun setDirection(direction: Motor.Direction)
    fun setPidfTolerance(tolerance: Angle)
    fun setPidfTolerance(tolerance: Distance, circumference: Distance)
    fun setMode(mode: Motor.RunMode)

    /* ! GETTER METHODS ! */
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
}