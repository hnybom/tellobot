package main

import (
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"time"
)

func main() {
	drone := tello.NewDriver("8888")

	work := func() {
		drone.TakeOff()
		gobot.After(5 * time.Second, func() {
			drone.BackFlip()
		})

		gobot.After(10 * time.Second, func() {
			drone.FrontFlip()
		})

		gobot.After(15 * time.Second, func() {
			drone.Land()
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work)

	robot.Start()

}
