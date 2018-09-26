package tracking

import (
	"github.com/go-gl/mathgl/mgl32"
	"gocv.io/x/gocv"
	"image"
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
)

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

	gocv.Erode(*thresh, thresh, erodeElement)
	gocv.Erode(*thresh, thresh, erodeElement)
	gocv.Dilate(*thresh, thresh, dilateElement)
	gocv.Dilate(*thresh, thresh, dilateElement)

}

func TrackFilteredObject(x *int, y *int, distance *float32, filteredImage gocv.Mat) {

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

func TrackAruco(corners [][]mgl32.Vec2) (int, int){

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

	return populateAndGetAverage(sumX, &xavgAruco), populateAndGetAverage(sumY, &yavgAruco)

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


