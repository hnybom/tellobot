package main

import (
	"fmt"
	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot/platforms/keyboard"
	"gocv.io/x/gocv/contrib"
	"image"
	"image/color"
	"io"
	"os/exec"
	"strconv"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
)

const (
	frameX    = 400
	frameY    = 300
	frameSize = frameX * frameY * 3
	webCam 	  = false
	maxPower = float32(50)
	minPower = float32(10)
)

func main() {
	window := gocv.NewWindow("Tello")
	dict := contrib.NewArucoPredefinedDictionary(contrib.ArucoPredefinedDict_5x5_50)
	defer dict.Close()

	if(webCam) {
		webcam, err := gocv.VideoCaptureDevice(0)
		if err != nil {
			fmt.Println(err)
			return
		}
		defer webcam.Close()

		img := gocv.NewMat()
		defer img.Close()

		camMat, cDiffs := contrib.ReadCameraParameters("../camera-calibration.yaml")

		for {
			if ok := webcam.Read(&img); !ok {
				fmt.Printf("cannot read device %d\n", 0)
				return
			}
			if img.Empty() {
				continue
			}

			corners, ids := dict.DetectMarkers(&img)
			drawCoordinates(camMat, cDiffs, img)

			if len(corners) > 0  {

				rvecs, xdiff, ydiff, distance := getFlightParams(corners, camMat, cDiffs, dict, img, ids)
				_ = rvecs
				flyTracking(xdiff, ydiff, distance, nil)

				distStr := fmt.Sprintf("d: %.0fcm, x: %.0fcm, y: %.0fcm", 100*distance, 100*xdiff, 100*ydiff)
				textSize := gocv.GetTextSize(distStr, gocv.FontHersheySimplex, 0.5, 3)
				gocv.PutText(&img, distStr, image.Pt(200-textSize.X/2, 40+textSize.Y/2), gocv.FontHersheySimplex, 0.8, color.RGBA{255, 255, 255, 0}, 4)

			}

			window.IMShow(img)
			window.WaitKey(1)
		}

	} else {

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

		camMat, cDiffs := contrib.ReadCameraParameters("../drone-camera-calibration-400.yaml")

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

			corners, ids := dict.DetectMarkers(&img)

			if len(corners) > 0 {

				rvecs, xdiff, ydiff, distance := getFlightParams(corners, camMat, cDiffs, dict, img, ids)
				_ = rvecs
				flyTracking(xdiff, ydiff, distance, drone)

			} else {
				drone.Hover()
			}

			window.IMShow(img)
			window.WaitKey(1)
		}
	}

}

func drawCoordinates(camMat gocv.Mat, cDiffs gocv.Mat, img gocv.Mat) {
	x0 := mgl32.Vec3{-1.0, 0, 0}
	x1 := mgl32.Vec3{+1.0, 0, 0}
	y0 := mgl32.Vec3{0, -1.0, 0}
	y1 := mgl32.Vec3{0, +1.0, 0}
	axis := []mgl32.Vec3{x0, x1, y0, y1}
	axis2d := contrib.ProjectPoints(axis, mgl32.Vec3{0, 0, 0}, mgl32.Vec3{0, 0, 1}, &camMat, &cDiffs)
	gocv.Line(&img, image.Pt(int(axis2d[0][0]), int(axis2d[0][1])), image.Pt(int(axis2d[1][0]), int(axis2d[1][1])), color.RGBA{255, 0, 255, 0}, 1)
	gocv.Line(&img, image.Pt(int(axis2d[2][0]), int(axis2d[2][1])), image.Pt(int(axis2d[3][0]), int(axis2d[3][1])), color.RGBA{0, 255, 255, 0}, 1)
}

func getFlightParams(corners [][]mgl32.Vec2, camMat gocv.Mat, cDiffs gocv.Mat, dict contrib.ArucoDictionary, img gocv.Mat, ids []int) ([]mgl32.Vec3, float32, float32, float32) {
	rvecs, tvecs := contrib.EstimateMarkerPoses(corners, 0.08, &camMat, &cDiffs)
	dict.DrawDetectedMarkers(&img, corners, ids, color.RGBA{255, 0, 0, 0})
	return rvecs, tvecs[0][0], tvecs[0][1], tvecs[0][2]
}

func flyTracking(xdiff float32, ydiff float32, distance float32, drone *tello.Driver) {

	powerx := (xdiff / 0.5) * maxPower
	powery := (ydiff / 0.5) * maxPower

	powerx = abs(powerx)
	powery = abs(powery)

	powerx = capPower(powerx)
	powery = capPower(powery)

	fly(powerx, xdiff, powery, ydiff, distance, drone)

}

func capPower(powerx float32) float32 {
	if (powerx > maxPower) {
		powerx = maxPower
	}
	return powerx
}

func abs(powery float32) float32 {
	if (powery < 0) {
		powery = -1 * powery
	}
	return powery
}

func fly(powerx float32, xdiff float32, powery float32, ydiff float32, distance float32, drone *tello.Driver) {

	if powerx > minPower || powerx < -minPower {
		if xdiff > 0 {
			if (drone != nil) {
				drone.Right(int(powerx))
			}
			fmt.Println("Power to RIGHT with ", powerx)
		} else {
			if (drone != nil) {
				drone.Left(int(powerx))
			}
			fmt.Println("Power to LEFT with ", powerx)
		}
	} else if (drone != nil) {
		drone.Right(0)
	}
	if powery > minPower || powery < -minPower {
		if ydiff > 0 {
			if (drone != nil) {
				drone.Down(int(powery))
			}
			fmt.Println("Power to DOWN with ", powery)
		} else {
			if (drone != nil) {
				drone.Up(int(powery))
			}
			fmt.Println("Power to UP with ", -powery)
		}
	} else if (drone != nil) {
		drone.Down(0)
	}

	if distance > 2 {

		powerz := (distance / 3) * maxPower
		powerz = abs(powerz)
		powerz = capPower(powerz)

		if (drone != nil) {
			drone.Forward(int(powerz))
		}
		fmt.Println("Power to CLOSER with ", powerz)
	} else if (distance < 1.5) {

		powerz := (1.5 - distance) * maxPower
		powerz = abs(powerz)
		powerz = capPower(powerz)

		if (drone != nil) {
			drone.Backward(int(powerz))
		}
		fmt.Println("Power to FURTHER with ", powerz)
	} else if (drone != nil) {
		drone.Forward(0)
	}
}

func mapKeys(keys *keyboard.Driver, drone *tello.Driver) {

	takenoff := false

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