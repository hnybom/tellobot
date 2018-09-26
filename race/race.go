package race

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"tellobot/drone"

	"github.com/go-gl/mathgl/mgl32"
	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
)

var markerPositions []mgl32.Vec3 = []mgl32.Vec3{
	mgl32.Vec3{0.0, 0.295, 0},
	mgl32.Vec3{0.295, 0.0, 0},
	mgl32.Vec3{0.0, -0.295, 0},
	mgl32.Vec3{-0.295, 0.0, 0},
}

var projectPoints []mgl32.Vec3 = []mgl32.Vec3{
	mgl32.Vec3{0.0, 0.0, 0.0},
	mgl32.Vec3{0.0, 0.0, 0.2},
	mgl32.Vec3{0.0, 0.0, 0.4},
	markerPositions[0],
	markerPositions[1],
	markerPositions[2],
	markerPositions[3],
}

func init() {
	for i := 0; i < 40; i++ {
		angle := (2.0 * math.Pi) * (float64(i) / 40.0)
		s := float32(math.Sin(angle))
		c := float32(math.Cos(angle))
		projectPoints = append(projectPoints, mgl32.Vec3{s * 0.22, c * 0.22, 0.0})
	}
}

type Race struct {
	dict contrib.ArucoDictionary
}

func NewRace() *Race {
	r := Race{}
	r.dict = contrib.NewArucoPredefinedDictionary(contrib.ArucoPredefinedDict_5x5_50)

	return &r
}
func (r *Race) Close() {
	r.dict.Close()
}

func findSize(corners []mgl32.Vec2) mgl32.Vec2 {
	var min, max mgl32.Vec2
	min[0] = corners[0][0]
	min[1] = corners[0][1]
	max[0] = corners[0][0]
	max[1] = corners[0][1]
	for _, p := range corners {
		if p[0] < min[0] {
			min[0] = p[0]
		}
		if p[1] < min[1] {
			min[1] = p[1]
		}
		if p[0] > max[0] {
			max[0] = p[0]
		}
		if p[1] > max[1] {
			max[1] = p[1]
		}
	}
	return max.Sub(min)
}

func (r *Race) DetectRings(img *gocv.Mat, rings map[int]*Ring) map[int]*Ring {
	tracking := false
	if rings != nil {
		tracking = true
	} else {
		rings = make(map[int]*Ring)
	}

	// remove too old markers & rings (if no rings were given, this will be skipped)
	for id, ring := range rings {
		markersVisible := 0
		for i, m := range ring.Markers {
			if m == nil {
				continue
			}
			m.DetectedAgo++
			if m.DetectedAgo > 30 {
				for j := 0; j < 4; j++ {
					if m.Trackers[j] != nil {
						m.Trackers[j].Close()
					}
				}
				ring.Markers[i] = nil
				continue
			}

			// we're still tracking this marker
			markersVisible++
		}

		if markersVisible == 0 {
			// this ring has no markers visible
			delete(rings, id)
		}
	}

	const trackerRectSize = 0.4

	// add / update newly detected
	corners, ids := r.dict.DetectMarkers(img)
	for i, id := range ids {
		ringId := (id - 1) / 4
		ring, ok := rings[ringId]
		if !ok {
			ring = &Ring{}
			rings[ringId] = ring
		}
		m := ring.Markers[(id-1)%4]
		if m == nil {
			m = &Marker{}
			ring.Markers[(id-1)%4] = m
		}
		m.Corners = corners[i]

		if tracking {
			size := findSize(m.Corners)
			m.DetectedAgo = 0

			// setup new trackers
			for j := 0; j < 4; j++ {
				if m.Trackers[j] != nil {
					m.Trackers[j].Close()
				}
				m.Trackers[j] = contrib.NewTrackerMedianFlow()

				rect := image.Rect(
					int(m.Corners[j][0]-size[0]*trackerRectSize),
					int(m.Corners[j][1]-size[1]*trackerRectSize),
					int(m.Corners[j][0]+size[0]*trackerRectSize),
					int(m.Corners[j][1]+size[1]*trackerRectSize))
				m.Trackers[j].Init(*img, rect)
			}
		}
	}

	if tracking {
		// update trackers for undetected corners
		for id, ring := range rings {
			markersActive := 0
			totalCorners := 0
			for i, m := range ring.Markers {
				if m == nil {
					continue
				}
				if m.DetectedAgo == 0 {
					markersActive++
					totalCorners += 4
					continue
				}

				cornersActive := 0
				for j := 0; j < 4; j++ {
					if m.Trackers[j] == nil {
						continue
					}
					rect, ok := m.Trackers[j].Update(*img)
					if !ok {
						m.Trackers[j].Close()
						m.Trackers[j] = nil
						continue
					}

					corner := mgl32.Vec2{float32(rect.Min.X+rect.Max.X) * 0.5, float32(rect.Min.Y+rect.Max.Y) * 0.5}

					remove := false
					if rect.Max.X-rect.Min.X > 100 {
						remove = true
					} else if rect.Max.Y-rect.Min.Y > 100 {
						remove = true
					} else if corner.Sub(m.Corners[j]).Len() > 50 {
						remove = true
					}
					if remove {
						m.Trackers[j].Close()
						m.Trackers[j] = nil
						continue
					}
					cornersActive++

					m.Corners[j] = corner
				}
				if cornersActive == 0 {
					ring.Markers[i] = nil
				} else {
					totalCorners += cornersActive
					markersActive++
				}
			}
			if markersActive == 0 || totalCorners < 4 {
				fmt.Println(markersActive, totalCorners)
				delete(rings, id)
			}
		}
	}

	return rings
}

var markerCorners []mgl32.Vec3 = []mgl32.Vec3{
	mgl32.Vec3{-0.04, +0.04, 0},
	mgl32.Vec3{+0.04, +0.04, 0},
	mgl32.Vec3{+0.04, -0.04, 0},
	mgl32.Vec3{-0.04, -0.04, 0},
}

func (r *Ring) EstimatePose(d drone.Drone) (pos mgl32.Vec3, rot mgl32.Mat3) {
	var objectPoints []mgl32.Vec3
	var imagePoints []mgl32.Vec2
	for i, m := range r.Markers {
		if m == nil {
			continue
		}
		for j := 0; j < 4; j++ {
			if m.DetectedAgo == 0 || m.Trackers[j] != nil {
				objectPoints = append(objectPoints, markerPositions[i].Add(markerCorners[j]))
				imagePoints = append(imagePoints, m.Corners[j])
			}
		}
	}

	r.RodriguesRotation, r.Position = contrib.SolvePnP(objectPoints, imagePoints, d.CameraMatrix(), d.DistortionCoefficients())

	rot = contrib.Rodrigues(r.RodriguesRotation)

	return r.Position, rot
}

type Ring struct {
	Markers [4]*Marker // N, E, S, W

	Position          mgl32.Vec3
	RodriguesRotation mgl32.Vec3
}

type Marker struct {
	Corners     []mgl32.Vec2       // NW, NE, SE, SW
	Trackers    [4]contrib.Tracker // one tracker per corner
	DetectedAgo int                // how many frames ago
}

func (r *Ring) Draw(img *gocv.Mat, d drone.Drone) {

	p := contrib.ProjectPoints(projectPoints, r.RodriguesRotation, r.Position, d.CameraMatrix(), d.DistortionCoefficients())
	center := image.Pt(int(p[0][0]), int(p[0][1]))
	z := image.Pt(int(p[1][0]), int(p[1][1]))
	z2 := image.Pt(int(p[2][0]), int(p[2][1]))

	p = p[3:] // remove center and z-axis
	for i := 0; i < 4; i++ {
		pt := image.Pt(int(p[i][0]), int(p[i][1]))
		if r.Markers[i] == nil {
			gocv.Ellipse(img, pt, image.Pt(4, 4), 0, 0, 360, color.RGBA{255, 0, 0, 0}, 1)
		} else {
			if r.Markers[i].DetectedAgo == 0 {
				gocv.Ellipse(img, pt, image.Pt(4, 4), 0, 0, 360, color.RGBA{0, 255, 0, 0}, 1)
			} else {
				gocv.Ellipse(img, pt, image.Pt(4, 4), 0, 0, 360, color.RGBA{0, 0, 255, 0}, 1)
			}
		}
	}

	p = p[4:] // remove marker positions
	for i := 0; i < len(p); i++ {
		p0 := image.Pt(int(p[i][0]), int(p[i][1]))
		p1 := image.Pt(int(p[(i+1)%len(p)][0]), int(p[(i+1)%len(p)][1]))
		gocv.Line(img, p0, p1, color.RGBA{255, 255, 0, 0}, 2)
	}

	distStr := fmt.Sprintf("%.0fcm", 100*r.Position[2])
	textSize := gocv.GetTextSize(distStr, gocv.FontHersheySimplex, 0.5, 3)
	gocv.PutText(img, distStr, image.Pt(center.X-textSize.X/2, center.Y+textSize.Y/2), gocv.FontHersheySimplex, 0.8, color.RGBA{255, 255, 255, 0}, 3)

	gocv.Line(img, z, z2, color.RGBA{0, 0, 255, 0}, 1)
	gocv.Line(img, center, z, color.RGBA{0, 255, 255, 0}, 2)
}
