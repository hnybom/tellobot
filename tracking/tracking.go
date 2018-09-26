package tracking

import (
	"fmt"
	"tellobot/drone"
	"tellobot/utils"
	"time"
)

const (
	maxPowerR = float32(60)
	maxPower = float32(40)
	minPower = float32(10)
)

type flyState struct {
	onTarget bool
	onTargetCounter int
	ringCounter int
}


var (
	state = flyState{false, 0, 0}
)

func FindNextRing(drone drone.Drone) {
	drone.Hover()
	drone.Clockwise(70)
}

func FlyTracking(xdiff float32, ydiff float32, distance float32, rotation float32, drone drone.Drone) {

	powerx := (xdiff / 0.2) * maxPowerR
	powery := (ydiff / 0.2) * maxPower
	powerz := rotation * 100

	powerx = utils.Abs(powerx)
	powery = utils.Abs(powery)
	powerz = utils.Abs(powerz)

	powerx = utils.CapPower(powerx, maxPowerR)
	powery = utils.CapPower(powery, maxPower)
	powerz = utils.CapPower(powerz, maxPower)

	fly(powerx, xdiff, powery, ydiff, distance, rotation, powerz, drone)

}

func fly(powerx float32, xdiff float32,
		 powery float32, ydiff float32,
		 distance float32, rotation float32,
		 powerz float32, drone drone.Drone) {

	if state.onTarget {
		fmt.Println("X: ", xdiff, "Y: ", ydiff, "Z: ", distance)
		return
	}

	if xdiff > 0 {
		drone.Clockwise(int(powerx))
		fmt.Println("Power to SPIN CLOCKWISE with ", powerx)
	} else {
		drone.CounterClockwise(int(powerx))
		fmt.Println("Power to SPIN COUNTERCLOCKWISE with ", powerx)
	}

	if powery > minPower || powery < -minPower {
		if ydiff > 0 {
			drone.Down(int(powery))
			fmt.Println("Power to DOWN with ", powery)
		} else {
			drone.Up(int(powery))
			fmt.Println("Power to UP with ", powery)
		}
	} else {
		drone.Down(0)
	}

	if rotation > 0 {
		drone.Right(int(powerz))
		fmt.Println("Power to RIGHT with ", powerz)
	} else {
		drone.Left(int(powerz))
		fmt.Println("Power to LEFT with ", powerz)
	}

	keepDistance(distance, drone)
	//flyThrough(xdiff, ydiff, drone, distance)
}

func flyThrough(xdiff float32, ydiff float32, distance float32, rotation float32, drone drone.Drone) {
	if xdiff < 0.10 && ydiff < 0.10 && rotation < 0.10 {
		state.onTargetCounter++
		if state.onTargetCounter > 25 {

			drone.Forward(50)
			drone.Up(0)
			drone.Right(0)
			drone.Clockwise(0)
			state.onTarget = true

			time.AfterFunc(time.Duration(distance*100)*4*time.Second/100, func() {
				drone.Hover()
				state.onTarget = false
				state.onTargetCounter = 0
				state.ringCounter++
			})
		}
	} else {
		state.onTargetCounter = 0
	}
}

func keepDistance(distance float32, drone drone.Drone) {
	if distance > 2 {
		powerz := (distance / 3) * maxPower
		powerz = utils.Abs(powerz)
		powerz = utils.CapPower(powerz, maxPower)
		drone.Forward(int(powerz))
		fmt.Println("Power to CLOSER with ", powerz)
	} else if distance < 1.5 {

		powerz := (1.5 - distance) * maxPower
		powerz = utils.Abs(powerz)
		powerz = utils.CapPower(powerz, maxPower)
		drone.Backward(int(powerz))
		fmt.Println("Power to FURTHER with ", powerz)
	} else if drone != nil {
		drone.Forward(0)
	}
}

