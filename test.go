package main

import (
        //"fmt"
        "time"

        "gobot.io/x/gobot"
        "gobot.io/x/gobot/platforms/dji/tello"
)

func main() {
        drone := tello.NewDriver("8888")

        work := func() {
                drone.TakeOff()

		gobot.After(3*time.Second, func() {
		      drone.Forward(25)
		  })

		gobot.After(10*time.Second, func() {
			      drone.FrontFlip()
			  })

		gobot.After(15*time.Second, func() {
		      drone.Backward(20)
		  })

	        gobot.After(18*time.Second, func() {
		      drone.BackFlip()
		  })



                gobot.After(24*time.Second, func() {
                        drone.Land()
                })
        }

        robot := gobot.NewRobot("tello",
                []gobot.Connection{},
                []gobot.Device{drone},
                work,
        )

        robot.Start()
}

