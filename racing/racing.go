package main

import (
	"fmt"
	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot/platforms/keyboard"
	"gocv.io/x/gocv"
	"tellobot/drone"
	"tellobot/race"
	"tellobot/tracking"
)

const (
	frameX    = 400
	frameY    = 300
	frameSize = frameX * frameY * 3
)

var (
	takenoff = false
)

func main() {

	// create window
	window := gocv.NewWindow("Drone")

	// create race
	racex := race.NewRace()
	defer racex.Close()

	// create drone
	//dronex := drone.New(drone.DroneFake, mapKeys, "../camera-calibration.yaml")
	dronex := drone.New(drone.DroneReal, mapKeys, "../drone-camera-calibration-400.yaml")
	err := dronex.Init()
	if err != nil {
		fmt.Printf("error while initializing drone: %v\n", err)
		return
	}

	// create mat to hold the video frame
	frame := gocv.NewMat()

	//rings := make(map[int]*race.Ring)

	for {
		dronex.ReadVideoFrame(&frame)
		if frame.Empty() {
			continue
		}

		//rings := racex.DetectRings(&frame, nil)
		rings := racex.DetectRings(&frame, nil)
		zvecs := []mgl32.Vec3{}
		for id, ring := range rings {
			_ = id
			_, rot := ring.EstimatePose(dronex)
			zvec := rot.Mul3x1(mgl32.Vec3{ 0.0, 0.0, 1.0 })
			zvecs = append(zvecs, zvec)
			//fmt.Printf("%.2f, %.2f, %.2f\n", ring.Position[0], ring.Position[1], ring.Position[2])
			ring.Draw(&frame, dronex)
		}

		if (len(rings) > 0 && takenoff) {
			pos := dronex.CameraToDroneMatrix().Mul3x1(rings[0].Position)
			zrot := zvecs[0][0]
			tracking.FlyTracking(pos.X(), pos.Y(), pos.Z(), zrot, dronex)
		} else {
			dronex.Hover()
			dronex.CeaseRotation()
		}

		drone.DrawCrosshair(dronex, &frame)
		drone.DrawControls(dronex, &frame)

		window.IMShow(frame)
		window.WaitKey(1)
	}
}

func mapKeys(key keyboard.KeyEvent, drone drone.Drone) {
	switch key.Key {
	case keyboard.Spacebar:
		if (takenoff) {
			drone.Land()
		} else {
			drone.TakeOff()
		}
		takenoff = !takenoff
	}

}
