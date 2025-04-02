// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"flag"
	"math/rand"

	"github.com/pointlander/snow/vector"

	"github.com/alixaxel/pagerank"
)

// Item is an item in a circular buffer
type Item struct {
	Vector *[256]float32
	Next   byte
	Action byte
}

// Pixel is a pixel sensor
type Pixel struct {
	X, Y   int
	Mixer  Mixer
	Index  int
	Buffer [1024]Item
}

type (
	// JoystickState is the state of a joystick
	JoystickState uint
	// LightState is the states of the lights
	LightState uint
	// Mode is the operating mode of the robot
	Mode uint
	// Camera is a camera
	TypeCamera uint
	// Action is an action to take
	TypeAction uint
)

const (
	// JoystickStateNone is the default state of a joystick
	JoystickStateNone JoystickState = iota
	// JoystickStateUp is the state of a joystick when it is pushed up
	JoystickStateUp
	// JoystickStateDown is the state of a joystick when it is pushed down
	JoystickStateDown
)

const (
	// LightStateOn the light is on
	LightStateOn LightState = iota
	// LightStateOff the light is off
	LightStateOff
)

const (
	// ModeManual
	ModeManual Mode = iota
	// ModeAuto
	ModeAuto
)

const (
	// ActionLeft
	ActionLeft TypeAction = iota
	// ActionRight
	ActionRight
	// ActionForward
	ActionForward
	// ActionBackward
	ActionBackward
	// ActionNone
	ActionNone
	// ActionLight
	ActionLight
	// ActionCount
	ActionCount
)

// Robot is a robot
type Robot interface {
	Init()
	Do(action TypeAction)
}

// String returns a string representation of the JoystickState
func (j JoystickState) String() string {
	switch j {
	case JoystickStateUp:
		return "up"
	case JoystickStateDown:
		return "down"
	default:
		return "none"
	}
}

var (
	// FlagRobot is the robot mode
	FlagRobot = flag.Bool("robot", false, "is the robot mode")
)

// Mind is a minde
func Mind(do func(action TypeAction)) {
	camera := NewV4LCamera()

	go camera.Start("/dev/video0")

	indexes, left, right := make(chan [6]float32, 8), make(chan Frame, 8), make(chan Frame, 8)
	go func() {
		for img := range camera.Images {
			left <- img
			right <- img
		}
	}()
	hemisphere := func(seed int64, images chan Frame) {
		rng := rand.New(rand.NewSource(seed))
		var pixels []Pixel
		for img := range images {
			width := img.Frame.Bounds().Max.X
			height := img.Frame.Bounds().Max.Y
			if pixels == nil {
				for i := 0; i < 256; i++ {
					mixer := NewMixer()
					x := rng.Intn(width)
					y := rng.Intn(height)
					pixel := Pixel{
						X:     x,
						Y:     y,
						Mixer: mixer,
					}
					for j := range pixel.Buffer {
						var vec [256]float32
						for k := range vec {
							vec[k] = rng.Float32()
						}
						scale := sqrt(vector.Dot(vec[:], vec[:]))
						for k := range vec {
							vec[k] /= scale
						}
						pixel.Buffer[j].Vector = &vec
						pixel.Buffer[j].Next = byte(rng.Intn(256))
						pixel.Buffer[j].Action = byte(rng.Intn(6))
					}
					pixel.Mixer.Add(0)
					pixels = append(pixels, pixel)
				}
			}
			inputs, indxs := []*[256]float32{}, []int{}
			for i := range pixels {
				pixel := img.GrayAt(pixels[i].X, pixels[i].Y)
				query, max, index := pixels[i].Mixer.Mix(), float32(0.0), 0
				for j := range pixels[i].Buffer {
					cs := CS(query[:], pixels[i].Buffer[j].Vector[:])
					if cs > max {
						max, index = cs, j
					}
				}
				pixels[i].Index = (pixels[i].Index + 1) % len(pixels[i].Buffer)
				pixels[i].Buffer[pixels[i].Index].Vector = query
				pixels[i].Buffer[pixels[i].Index].Next = pixel.Y
				inputs = append(inputs, query)
				indxs = append(indxs, index)
				pixels[i].Mixer.Add(pixel.Y)
			}
			embedding := make([]float32, len(inputs))
			{
				graph := pagerank.NewGraph()
				for i := 0; i < len(inputs); i++ {
					for j := 0; j < len(inputs); j++ {
						p := CS(inputs[i][:], inputs[j][:])
						graph.Link(uint32(i), uint32(j), float64(p))
					}
				}
				graph.Rank(1.0, 1e-3, func(node uint32, rank float64) {
					embedding[node] = float32(rank)
				})
			}
			distro := [6]float32{}
			for i := range pixels {
				distro[pixels[i].Buffer[indxs[i]].Action] += embedding[i]
			}
			for i := range pixels {
				sum, selected, action := float32(0.0), rng.Float32(), 0
				for i, v := range distro {
					sum += v
					if selected < sum {
						action = i
						break
					}
				}
				pixels[i].Buffer[pixels[i].Index].Action = byte(action)
			}

			indexes <- distro
		}
	}

	go hemisphere(1, left)
	go hemisphere(2, right)

	rng := rand.New(rand.NewSource(1))
	actions := make([]float32, 6)
	count := 0
	for index := range indexes {
		if count%60 == 0 {
			sum, selected, index := float32(0.0), 60*rng.Float32(), 0
			for i, v := range actions {
				sum += v
				actions[i] = 0
				if selected < sum {
					index = i
					break
				}
			}
			switch index {
			case 0:
				do(ActionForward)
			case 1:
				do(ActionBackward)
			case 2:
				do(ActionLeft)
			case 3:
				do(ActionRight)
			case 4:
				do(ActionLight)
			case 5:
				do(ActionNone)
			}
		} else {
			for i := range index {
				actions[i] += index[i]
			}
		}
		count++
	}
}

func main() {
	flag.Parse()

	var robot Robot
	if *FlagRobot {
		robot = &Waveshare{}
	} else {
		robot = &Urobot{}
	}

	robot.Init()
	Mind(robot.Do)
}
