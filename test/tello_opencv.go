// +build example
//
// Do not build by default.

/*
You must have ffmpeg and OpenCV installed in order to run this code. It will connect to the Tello
and then open a window using OpenCV showing the streaming video.

How to run

	go run examples/tello_opencv.go
*/

package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"io"
	"os/exec"
	"strconv"
	"tellobot/tracking"
	"time"
)

const (
	frameX    = 480
	frameY    = 360
	frameSize = frameX * frameY * 3

)

var (
	hmaxTracking, hminTracking, sminTracking, smaxTracking, vminTracking,vmaxTracking *gocv.Trackbar
	hmax, hmin, smin, smax, vmin, vmax int
	)


func main() {

	drone := tello.NewDriver("8890")
	window := gocv.NewWindow("Tello")
	trackingBar := gocv.NewWindow("Tracking sliders")
	hmaxTracking = trackingBar.CreateTrackbar("H_MAX", 255)
	hminTracking = trackingBar.CreateTrackbar("H_MIN", 255)
	sminTracking = trackingBar.CreateTrackbar("S_MIN", 255)
	smaxTracking = trackingBar.CreateTrackbar("S_MAX", 255)
	vminTracking = trackingBar.CreateTrackbar("V_MIN", 255)
	vmaxTracking = trackingBar.CreateTrackbar("V_MAX", 255)
	trackingBar.ResizeWindow(800, 600)

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
			drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
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
		[]gobot.Device{drone},
		work,
	)

	// calling Start(false) lets the Start routine return immediately without an additional blocking goroutine
	robot.Start(false)

	// now handle video frames from ffmpeg stream in main thread, to be macOS/Windows friendly
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

		tracking.PrintIfChanged(&hmin, hminTracking, "H_MIN")
		tracking.PrintIfChanged(&hmax, hmaxTracking, "H_MAX")
		tracking.PrintIfChanged(&vmin, vminTracking, "V_MIN")
		tracking.PrintIfChanged(&vmax, vmaxTracking, "V_MAX")
		tracking.PrintIfChanged(&smin, sminTracking, "S_MIN")
		tracking.PrintIfChanged(&smax, smaxTracking, "S_MIN")
		/*tracking.FilterImage(img, &img,
			hminTracking.GetPos(),
			hmaxTracking.GetPos(),
			sminTracking.GetPos(),
			smaxTracking.GetPos(),
			vminTracking.GetPos(),
			vmaxTracking.GetPos())*/

		window.IMShow(img)

		if window.WaitKey(1) >= 0 {
			break
		}

		if trackingBar.WaitKey(1) >= 0 {
			break;
		}
	}
}

