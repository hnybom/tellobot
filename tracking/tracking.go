package tracking

import (
	"fmt"
	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"image"
	"time"
)

const (
	DISTANCE_TO_KEEP = 50
)

const (
	maxPower = float32(30)
	minPower = float32(5)
)


type Avgcontainer struct {
	index int
	values []int
}

type AvgcontainerFloat struct {
	index int
	values []float32
}

var (
	xavg = Avgcontainer{ 0, make([]int, 10)}
	yavg = Avgcontainer{ 0, make([]int, 10)}

	xavgAruco = Avgcontainer{ 0, make([]int, 10)}
	yavgAruco = Avgcontainer{ 0, make([]int, 10)}

	onTarget = false
	onTargetCounter = 0
)

func PrintIfChanged(val *int, bar *gocv.Trackbar, title string) {
	if *val != bar.GetPos() {
		fmt.Println(title, " : ", bar.GetPos())
		*val = bar.GetPos()
	}
}

func FilterImage(img gocv.Mat, dest *gocv.Mat, hmin int, hmax int, smin int, smax int, vmin int, vmax int) {
	gocv.CvtColor(img, dest, gocv.ColorBGRToHSV)

	gocv.InRangeWithScalar(*dest,
		gocv.NewScalar(
			float64(hmin),
			float64(smin),
			float64(vmin),
			0),
		gocv.NewScalar(
			float64(hmax),
			float64(smax),
			float64(vmax),
			0),
		dest)

	MorphOps(dest)

}

func MorphOps(thresh *gocv.Mat) {

	erodeElement := gocv.GetStructuringElement(gocv.MorphRect, image.Point{3, 3 })
	dilateElement := gocv.GetStructuringElement(gocv.MorphRect, image.Point{8, 8 })

	gocv.Erode(*thresh, thresh, erodeElement);
	gocv.Erode(*thresh, thresh, erodeElement);
	gocv.Dilate(*thresh, thresh, dilateElement);
	gocv.Dilate(*thresh, thresh, dilateElement);

}

func TrackFilteredObject(x *int, y *int, distance *float32, filteredImage gocv.Mat) {
	*distance = DISTANCE_TO_KEEP

	contours := gocv.FindContours(filteredImage, gocv.RetrievalCComp, gocv.ChainApproxSimple)

	refArea := float64(0)

	tempx := 0
	tempy := 0

	for i:=0 ; i < len(contours); i++ {
		contour := contours[i]
		area := gocv.ContourArea(contour)
		if area > refArea {

			sumx := 0
			sumy := 0

			for j := 0; j < len(contour); j++ {
				sumx += contour[j].X
				sumy += contour[j].Y
			}

			tempx = sumx / len(contour)
			tempy = sumy / len(contour)
			refArea = area
		}
	}

	*x = populateAndGetAverage(tempx, &xavg)
	*y = populateAndGetAverage(tempy, &yavg)

}

func TrackAruco(corners [][]mgl32.Vec2) (int, int, float32){

	sumX := 0
	sumY := 0

	if len(corners) > 0 {

		index := 0
		for i := 0; i < len(corners[index]); i++ {

			point := corners[index][i]
			sumX += int(point.X())
			sumY += int(point.Y())
		}

		sumX = sumX / len(corners[index])
		sumY = sumY / len(corners[index])
	}

	return populateAndGetAverage(sumX, &xavgAruco), populateAndGetAverage(sumY, &yavgAruco), DISTANCE_TO_KEEP

}

func FlyTracking(xdiff float32, ydiff float32, distance float32, drone *tello.Driver) {

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

	if(onTarget) {
		fmt.Println("X: ", xdiff, "Y: ", ydiff, "Z: ", distance)
		return
	}

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
			fmt.Println("Power to UP with ", powery)
		}
	} else if (drone != nil) {
		drone.Down(0)
	}

	if xdiff < 0.10 && ydiff < 0.10 {
		onTargetCounter++
		if(onTargetCounter > 25) {

			drone.Forward(30)
			drone.Up(0)
			drone.Right(0)
			onTarget = true

			time.AfterFunc(time.Duration(distance*100) * 4 * time.Second / 100, func() {
				drone.Hover()
			})
		}
	} else {
		onTargetCounter = 0
	}
}

func populateAndGetAverage(val int, avgcontainer *Avgcontainer) int {
	avgcontainer.values[avgcontainer.index] = val

	for i := avgcontainer.index; i < len(avgcontainer.values); i++ {
		if avgcontainer.values[i] == 0 {
			avgcontainer.values[i] = val
		}
	}

	avgcontainer.index = (avgcontainer.index + 1) % len(avgcontainer.values)

	sum := 0

	for i := 0; i < len(avgcontainer.values); i++ {
		sum += avgcontainer.values[i]
	}

	return sum / len(avgcontainer.values)
}
