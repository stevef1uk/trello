// Based off code from 'javaguy' on the Trello Drone Forum: https://tellopilots.com/threads/face-tracking-with-tello-and-gocv.374/
// I have removed the joystick controls and added some prints
// The drone takes the first face location it finds as the baseline against which to move forwards or backwards based on the latest face detected
package main

import (
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
	"os"
	"os/exec"
	"strconv"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
)

const frameSize = frameX * frameY * 3
const frameX = 400

const frameY = 350

//const frameY = 400

var drone = tello.NewDriver("8890")
var window = gocv.NewWindow("Tello")

var ffmpeg = exec.Command("ffmpeg", "-hwaccel", "auto", "-threads", "0", "-hwaccel_device", "opencl", "-i", "pipe:0",
	"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")

//var ffmpeg = exec.Command("mplayer", "-xy", "400", "-fps", "25", "-lavdopts", "threads=4", "-framedrop", "-")
var ffmpegIn, _ = ffmpeg.StdinPipe()
var ffmpegOut, _ = ffmpeg.StdoutPipe()

var flightData *tello.FlightData

var detectSize = true
var distTolerance = float64(0.02 * dist(0, 0, frameX, frameY))

func setup(delay int64) {
	go func() {
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.FlightDataEvent, func(data interface{}) {
			flightData = data.(*tello.FlightData)
			fmt.Println("battery:", flightData.BatteryPercentage)
		})

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			fmt.Println("Distance Tolerance = ", distTolerance)
			drone.StartVideo()
			drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
			drone.SetExposure(0)
			drone.TakeOff()
			gobot.After(time.Duration(delay)*time.Second, func() {
				drone.Land()
			})
			gobot.Every(250*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})

		robot := gobot.NewRobot("tello",
			[]gobot.Connection{},
			[]gobot.Device{drone},
		)

		robot.Start()
	}()
}

func main() {
	if len(os.Args) < 4 {
		fmt.Println("How to run:\ngo run facetracking.go [protofile] [modelfile] [landtime]")
		return
	}

	proto := os.Args[1]
	model := os.Args[2]
	landDelay, err := strconv.ParseInt(os.Args[3], 10, 64)
	if err != nil {
		fmt.Println("landtime must be an integer in seconds\n")
		return
	}
	fmt.Printf("landing in %d seconds\n", landDelay)

	setup(landDelay)

	net := gocv.ReadNetFromCaffe(proto, model)
	if net.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", proto, model)
		return
	}
	defer net.Close()

	green := color.RGBA{0, 255, 0, 0}

	if net.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", proto, model)
		return
	}
	defer net.Close()

	refDistance := float64(0)
	detected := false
	left := float32(0)
	top := float32(0)
	right := float32(0)
	bottom := float32(0)
	i_left := float32(0)
	i_top := float32(0)
	i_right := float32(0)
	i_bottom := float32(0)
	i_distance := float64(0)
	firstDetect := true

	for {
		detected = false
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}
		W := float32(img.Cols())
		H := float32(img.Rows())
		blob := gocv.BlobFromImage(img, 1.0, image.Pt(128, 96), gocv.NewScalar(104.0, 177.0, 123.0, 0), false, false)
		defer blob.Close()

		net.SetInput(blob, "data")

		detBlob := net.Forward("detection_out")
		defer detBlob.Close()

		detections := gocv.GetBlobChannel(detBlob, 0, 0)
		defer detections.Close()

		for r := 0; r < detections.Rows(); r++ {
			confidence := detections.GetFloatAt(r, 2)
			if confidence < 0.5 {
				continue
			}

			left = detections.GetFloatAt(r, 3) * W
			top = detections.GetFloatAt(r, 4) * H
			right = detections.GetFloatAt(r, 5) * W
			bottom = detections.GetFloatAt(r, 6) * H

			left = min(max(0, left), W-1)
			right = min(max(0, right), W-1)
			bottom = min(max(0, bottom), H-1)
			top = min(max(0, top), H-1)

			// The first time we have seen a face record the rectance details
			if firstDetect {
				firstDetect = false
				i_left = left
				i_right = right
				i_bottom = bottom
				i_top = top
				i_distance = dist(i_left, i_top, i_right, i_bottom)
				fmt.Printf("Face initial distance %d\n", i_distance)
				if i_distance < distTolerance*5 {
					fmt.Printf("Face found but too far away for baseline\n", i_distance)
					firstDetect = true
				} else {
					drone.Up(50)
					drone.Down(50)
				}
			}
			rect := image.Rect(int(left), int(top), int(right), int(bottom))
			gocv.Rectangle(&img, rect, green, 3)
			detected = true
			detectSize = true
		}

		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}

		//if !tracking || !detected
		if !detected {
			fmt.Println("Face not detected\n")
			drone.Clockwise(0)
			drone.Up(0)
			drone.Forward(0)
			continue
		}

		fmt.Println("In Loop\n")
		fmt.Printf("distTolerance %d\n", distTolerance)

		if detectSize {
			detectSize = false
			refDistance = dist(left, top, right, bottom)
			fmt.Printf("refDistance: %d\n", refDistance)
		}

		// Lets put image in center of image
		if right < W/2 {
			fmt.Printf("right : %q < W/2 %q\n", right, W/2)
			drone.CounterClockwise(15)
		} else if left > W/2 {
			fmt.Printf("left : %q > W/2 %q\n", left, W/2)
			drone.Clockwise(30)
		} else {
			fmt.Printf("not right or left\n")
			drone.Clockwise(0)
		}

		// Lets put image in center of image
		if top < H/10 {
			fmt.Printf("top : %q < H/2 %q\n", top, H/2)
			drone.Up(30)
		} else if bottom > H-H/10 {
			fmt.Printf("bottom : %q > H-H/10 %q\n", bottom, H-H/10)
			drone.Down(30)
		} else {
			fmt.Printf("not up or down\n")
			drone.Up(0)
		}

		if refDistance < (i_distance - distTolerance/1.5) {
			fmt.Printf("forward : %q  %q %q\n", refDistance, i_distance-distTolerance/1.5, i_distance)
			drone.Forward(30)
		} else if refDistance > (i_distance + distTolerance/1.5) {
			fmt.Printf("backwards : %q  %q %q\n", refDistance, i_distance+distTolerance/1.5, i_distance)
			drone.Backward(30)
		} else {
			fmt.Printf("not forward or back\n")
			drone.Forward(0)
		}
	}
}

// This is Pythagoras to return the hypotenuse of a rectangle
func dist(x1, y1, x2, y2 float32) float64 {
	return math.Sqrt(float64((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)))
}

func min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func max(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
