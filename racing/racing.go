package main

import (
	"fmt"
	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/keyboard"
	"gocv.io/x/gocv"
	"io"
	"os/exec"
	"strconv"
	"tellobot/race"
	"tellobot/tracking"
	"time"
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
	window := gocv.NewWindow("Tello")
	racing := race.NewRace("../drone-camera-calibration-400.yaml")

	drone := tello.NewDriver("8890")
	keys := keyboard.NewDriver()
	mapKeys(keys, drone)

	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()

	work := func() {
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			drone.StartVideo()
			drone.SetVideoEncoderRate(tello.VideoBitRate1M)
			drone.SetExposure(0)

			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})

		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone, keys},
		work,
	)

	robot.Start(false)

	worldRotationMat := mgl32.HomogRotate3DX(mgl32.DegToRad(13.0))

	for {
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}

		racing.DetectRings(&img)
		rings := racing.Rings

		for _, ring := range rings {
			racing.EstimateRingPosition(ring)
			ring.Draw(&img)
		}

		if(len(rings) > 0 && takenoff) {
			pos := rings[0].Position
			transformedPosition := mgl32.TransformCoordinate(pos, worldRotationMat)
			tracking.FlyTracking(transformedPosition.X(), transformedPosition.Y(), transformedPosition.Z(), drone)
		}

		window.IMShow(img)
		window.WaitKey(1)
	}


}


func mapKeys(keys *keyboard.Driver, drone *tello.Driver) {



	keys.On(keyboard.Key, func(data interface{}) {
		key := data.(keyboard.KeyEvent)
		switch key.Key {
		case keyboard.Spacebar:
			if(takenoff) {
				drone.Land()
			} else {
				drone.TakeOff()
			}
			takenoff = !takenoff
		}
	})
}
