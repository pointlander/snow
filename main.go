// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math/rand"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/pointlander/snow/vector"

	"github.com/alixaxel/pagerank"
	htgotts "github.com/hegedustibor/htgo-tts"
	handlers "github.com/hegedustibor/htgo-tts/handlers"
	voices "github.com/hegedustibor/htgo-tts/voices"
	"github.com/veandco/go-sdl2/sdl"
	"go.bug.st/serial"
)

var joysticks = make(map[int]*sdl.Joystick)

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

func main() {
	flag.Parse()

	speech := htgotts.Speech{Folder: "audio", Language: voices.English, Handler: &handlers.MPlayer{}}
	speech.Speak("Starting...")

	camera := NewV4LCamera()
	say := make(chan string, 8)
	go func() {
		for s := range say {
			speech.Speak(s)
		}
	}()

	a := ActionNone

	go camera.Start("/dev/video0")

	mind := func() {
		indexes, left, right := make(chan int, 8), make(chan Frame, 8), make(chan Frame, 8)
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

				sum, selected, action := float32(0.0), rng.Float32(), 0
				for i, v := range distro {
					sum += v
					if selected < sum {
						action = i
						break
					}
				}
				indexes <- int(action)
			}
		}

		go hemisphere(1, left)
		go hemisphere(2, right)

		actions := make([]int, 6)
		count := 0
		for index := range indexes {
			if !*FlagRobot {
				if count%60 == 0 {
					max, index := 0, 0
					for i, v := range actions {
						actions[i] = 0
						if v > max {
							max, index = v, i
						}
					}
					switch index {
					case 0:
						say <- "forward"
					case 1:
						say <- "backward"
					case 2:
						say <- "left"
					case 3:
						say <- "right"
					case 4:
						say <- "light"
					case 5:
						say <- "none"
					}
				} else {
					actions[index%6]++
				}
			} else {
				if count%60 == 0 {
					max, index := 0, 0
					for i, v := range actions {
						actions[i] = 0
						if v > max {
							max, index = v, i
						}
					}
					switch index {
					case 0:
						a = ActionForward
					case 1:
						a = ActionBackward
					case 2:
						a = ActionLeft
					case 3:
						a = ActionRight
					case 4:
						a = ActionLight
					case 5:
						a = ActionNone
					}
				} else {
					actions[index%6]++
				}
			}
			count++
		}
	}

	if *FlagRobot {
		options := &serial.Mode{
			BaudRate: 115200,
		}
		port, err := serial.Open("/dev/ttyAMA0", options)
		if err != nil {
			panic(err)
		}

		var running bool

		c := make(chan os.Signal, 1)
		signal.Notify(c, os.Interrupt, syscall.SIGTERM)
		go func() {
			<-c
			err := port.Close()
			if err != nil {
				panic(err)
			}
			running = false
			os.Exit(1)
		}()

		var event sdl.Event
		sdl.Init(sdl.INIT_JOYSTICK)
		defer sdl.Quit()
		sdl.JoystickEventState(sdl.ENABLE)
		running = true
		var axis [5]int16
		joystickLeft := JoystickStateNone
		joystickRight := JoystickStateNone
		lightState := LightStateOff
		speed := 0.1
		var mode Mode

		go func() {
			message := map[string]interface{}{
				"T":      900,
				"main":   2,
				"module": 0,
			}
			data, err := json.Marshal(message)
			if err != nil {
				panic(err)
			}
			data = append(data, '\n')
			_, err = port.Write(data)
			if err != nil {
				panic(err)
			}
			leftSpeed, rightSpeed := 0.0, 0.0
			for running {
				time.Sleep(300 * time.Millisecond)
				if mode == ModeAuto {
					switch a {
					case ActionForward:
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateUp
					case ActionBackward:
						joystickLeft = JoystickStateDown
						joystickRight = JoystickStateDown
					case ActionLeft:
						joystickLeft = JoystickStateDown
						joystickRight = JoystickStateUp
					case ActionRight:
						joystickLeft = JoystickStateUp
						joystickRight = JoystickStateDown
					case ActionLight:
						pwm := 0
						if lightState == LightStateOn {
							pwm, lightState = 0, LightStateOff
						} else if lightState == LightStateOff {
							pwm, lightState = 128, LightStateOn
						}
						message := map[string]interface{}{
							"T":   132,
							"IO4": pwm,
							"IO5": pwm,
						}
						data, err := json.Marshal(message)
						if err != nil {
							panic(err)
						}
						data = append(data, '\n')
						_, err = port.Write(data)
						if err != nil {
							panic(err)
						}
					case ActionNone:
						joystickLeft = JoystickStateNone
						joystickRight = JoystickStateNone
					}
				}

				switch joystickLeft {
				case JoystickStateUp:
					leftSpeed = speed
				case JoystickStateDown:
					leftSpeed = -speed
				case JoystickStateNone:
					leftSpeed = 0.0
				}
				switch joystickRight {
				case JoystickStateUp:
					rightSpeed = speed
				case JoystickStateDown:
					rightSpeed = -speed
				case JoystickStateNone:
					rightSpeed = 0.0
				}

				message := map[string]interface{}{
					"T": 1,
					"L": leftSpeed,
					"R": rightSpeed,
				}
				data, err := json.Marshal(message)
				if err != nil {
					panic(err)
				}
				data = append(data, '\n')
				_, err = port.Write(data)
				if err != nil {
					panic(err)
				}
			}
		}()

		go mind()

		_, _ = joystickLeft, joystickRight
		for running {
			for event = sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
				switch t := event.(type) {
				case *sdl.QuitEvent:
					running = false
				case *sdl.JoyAxisEvent:
					value := int16(t.Value)
					axis[t.Axis] = value
					if t.Axis == 3 || t.Axis == 4 {
						if mode == ModeManual {
							if axis[3] < 20000 && axis[3] > -20000 {
								if axis[4] < -32000 {
									joystickRight = JoystickStateUp
								} else if axis[4] > 32000 {
									joystickRight = JoystickStateDown
								} else {
									joystickRight = JoystickStateNone
								}
							} else {
								joystickRight = JoystickStateNone
							}
						}
						//fmt.Printf("right [%d ms] Which: %v \t%d %d\n",
						//              t.Timestamp, t.Which, axis[3], axis[4])
					} else if t.Axis == 0 || t.Axis == 1 {
						if mode == ModeManual {
							if axis[0] < 20000 && axis[0] > -20000 {
								if axis[1] < -32000 {
									joystickLeft = JoystickStateUp
								} else if axis[1] > 32000 {
									joystickLeft = JoystickStateDown
								} else {
									joystickLeft = JoystickStateNone
								}
							} else {
								joystickLeft = JoystickStateNone
							}
						}
						//fmt.Printf("left [%d ms] Which: %v \t%d %d\n",
						//t.Timestamp, t.Which, axis[0], axis[1])
					} else if t.Axis == 2 {
						//fmt.Printf("2 axis [%d ms] Which: %v \t%x\n",
						//      t.Timestamp, t.Which, value)
						//speed = axis[2]
						//pwm = int(100 * (float64(speed) + 32768) / 65535)
						//fmt.Printf("speed %d pwm %d\n", speed, pwm)
					}
				case *sdl.JoyBallEvent:
					fmt.Printf("[%d ms] Ball:%d\txrel:%d\tyrel:%d\n",
						t.Timestamp, t.Ball, t.XRel, t.YRel)
				case *sdl.JoyButtonEvent:
					fmt.Printf("[%d ms] Button:%d\tstate:%d\n",
						t.Timestamp, t.Button, t.State)
					if t.Button == 0 && t.State == 1 {
						switch mode {
						case ModeManual:
							mode = ModeAuto
						case ModeAuto:
							mode = ModeManual
							joystickLeft = JoystickStateNone
							joystickRight = JoystickStateNone
						}
					} else if t.Button == 1 && t.State == 1 {
						speed += .1
						if speed > .3 {
							speed = 0.1
						}
					} else if t.Button == 2 && t.State == 1 {
						pwm := 0
						if lightState == LightStateOn {
							pwm, lightState = 0, LightStateOff
						} else if lightState == LightStateOff {
							pwm, lightState = 128, LightStateOn
						}
						message := map[string]interface{}{
							"T":   132,
							"IO4": pwm,
							"IO5": pwm,
						}
						data, err := json.Marshal(message)
						if err != nil {
							panic(err)
						}
						data = append(data, '\n')
						_, err = port.Write(data)
						if err != nil {
							panic(err)
						}
					}
				case *sdl.JoyHatEvent:
					fmt.Printf("[%d ms] Hat:%d\tvalue:%d\n",
						t.Timestamp, t.Hat, t.Value)
					if t.Value == 1 {
						// up
					} else if t.Value == 4 {
						// down
					} else if t.Value == 8 {
						// left
					} else if t.Value == 2 {
						// right
					}
				case *sdl.JoyDeviceAddedEvent:
					fmt.Println(t.Which)
					joysticks[int(t.Which)] = sdl.JoystickOpen(int(t.Which))
					if joysticks[int(t.Which)] != nil {
						fmt.Printf("Joystick %d connected\n", t.Which)
					}
				case *sdl.JoyDeviceRemovedEvent:
					if joystick := joysticks[int(t.Which)]; joystick != nil {
						joystick.Close()
					}
					fmt.Printf("Joystick %d disconnected\n", t.Which)
				default:
					fmt.Printf("Unknown event\n")
				}
			}

			sdl.Delay(16)
		}
	} else {
		mind()
	}
}
