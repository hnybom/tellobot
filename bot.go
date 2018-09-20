package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"time"
)

func main() {
	drone := tello.NewDriver("8888")
	autopilot := false

	work := func() {
		drone.TakeOff()

		gobot.After(5 * time.Second, func() {
			autopilot = true
		})


		gobot.After(20 * time.Second, func() {
			drone.Land()
		})
	}

	drone.On(tello.FlightDataEvent, func(data interface{}) {
		fd := data.(*tello.FlightData)
		fmt.Println(fd.Height)
		if(autopilot) {
			if(fd.Height < 0) {
				drone.Up(30)
			} else if(fd.Height > 2) {
				drone.Down(30)
			} else if(fd.Height > 5) {
				drone.Down(60)
			}
		}
	})

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work)

	robot.Start()

}
