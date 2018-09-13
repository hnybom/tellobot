package test

import (
	"fmt"
	"io"
	"os/exec"
	"strconv"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/opencv"
	"gocv.io/x/gocv"
)

const (
	frameX    = 400
	frameY    = 300
	frameSize = frameX * frameY * 3
	offset    = 32767.0
)

func main() {
	drone := tello.NewDriver("8890")
	window := opencv.NewWindowDriver()

	work := func() {
		ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
			"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
		ffmpegIn, _ := ffmpeg.StdinPipe()
		ffmpegOut, _ := ffmpeg.StdoutPipe()
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		go func() {
			for {
				buf := make([]byte, frameSize)
				if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
					fmt.Println(err)
					continue
				}

				img,_ := gocv.NewMatFromBytes(720, 960, gocv.MatTypeCV8UC3, buf)
				if img.Empty() {
					continue
				}
				window.ShowImage(img)
				window.WaitKey(1)
			}
		}()

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			drone.StartVideo()
			drone.SetExposure(1)
			drone.SetVideoEncoderRate(4)

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
		[]gobot.Device{drone, window},
		work,
	)

	robot.Start()
}