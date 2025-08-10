// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"flag"
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/signal"
	"runtime"
	"strings"
	"syscall"
	"time"

	"github.com/pointlander/gradient/tf32"
	"github.com/pointlander/snow/vector"

	"github.com/AndreRenaud/gore"
	"github.com/alixaxel/pagerank"
	"github.com/hajimehoshi/ebiten/v2"
)

const (
	// B1 exponential decay of the rate for the first moment estimates
	B1 = 0.8
	// B2 exponential decay rate for the second-moment estimates
	B2 = 0.89
	// Eta is the learning rate
	Eta = 1.0e-3
)

const (
	// StateM is the state for the mean
	StateM = iota
	// StateV is the state for the variance
	StateV
	// StateTotal is the total number of states
	StateTotal
)

// Item is an item in a circular buffer
type Item struct {
	Vector  *[256]float32
	Actions [ActionCount]int
}

// LongTerm is a long term memory bank
type LongTerm struct {
	RegisterSet bool
	Register    Item
	Index       int
	Bank        [8]Item
}

// Pixel is a pixel sensor
type Pixel struct {
	X, Y     int
	Mixer    Mixer
	Index    int
	Buffer   [1024]Item
	LongTerm [8]LongTerm
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
	// ActionLightOn
	ActionLightOn
	// ActionLightOff
	ActionLightOff
	// ActionLookUp
	ActionLookUp
	// ActionLookDown
	ActionLookDown
	// ActionLookLeft
	ActionLookLeft
	// ActionLookRight
	ActionLookRight
	// ActionCount
	ActionCount
)

// Robot is a robot
type Robot interface {
	Init()
	Do(action TypeAction)
	Done()
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
	FlagRobot = flag.String("robot", "urobot", "is the robot mode")
	// FlagDoom doom mode
	FlagDoom = flag.Bool("doom", false, "doom mode")
	// FlagIwad iwad
	FlagIwad = flag.String("iwad", "", "iwad")
	// FlagMind which mind to use
	FlagMind = flag.String("mind", "three", "which mind to use")
)

// Mind is a minde
func Mind(do func(action TypeAction)) {
	camera := NewV4LCamera()

	go camera.Start("/dev/video0")

	indexes, left, right, dleft, dright :=
		make(chan [ActionCount]float32, 8), make(chan Frame, 8), make(chan Frame, 8), make(chan Frame, 8), make(chan Frame, 8)
	go func() {
		for img := range camera.Images {
			left <- img
			right <- img
			dleft <- img
			dright <- img
		}
	}()
	hemisphere := func(derivative bool, seed int64, images chan Frame) {
		rng := rand.New(rand.NewSource(seed))
		var pixels []Pixel
		var last Frame
		for img := range images {
			if last.Frame == nil {
				last = img
				continue
			}
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
						pixel.Buffer[j].Actions[rng.Intn(int(ActionCount))] = 1
					}
					for j := range pixel.LongTerm {
						for k := range pixel.LongTerm[j].Bank {
							var vec [256]float32
							for k := range vec {
								vec[k] = rng.Float32()
							}
							scale := sqrt(vector.Dot(vec[:], vec[:]))
							for k := range vec {
								vec[k] /= scale
							}
							pixel.LongTerm[j].Bank[k].Vector = &vec
							pixel.LongTerm[j].Bank[k].Actions[rng.Intn(int(ActionCount))] = 1
						}
					}
					pixel.Mixer.Add(0)
					pixels = append(pixels, pixel)
				}
			}
			inputs, items := []*[256]float32{}, []*Item{}
			for i := range pixels {
				pixel := img.GrayAt(pixels[i].X, pixels[i].Y)
				if derivative {
					lpixel := last.GrayAt(pixels[i].X, pixels[i].Y)
					diff := int(pixel.Y) - int(lpixel.Y)
					if diff < 0 {
						diff = -diff
					}
					pixel.Y = uint8(diff)
				}
				var found *Item
				query, max := pixels[i].Mixer.Mix(), float32(0.0)
				for j := range pixels[i].Buffer {
					cs := CS(query[:], pixels[i].Buffer[j].Vector[:])
					if cs > max {
						max, found = cs, &pixels[i].Buffer[j]
					}
				}
				for j := range pixels[i].LongTerm {
					for k := range pixels[i].LongTerm[j].Bank {
						cs := CS(query[:], pixels[i].LongTerm[j].Bank[k].Vector[:])
						if cs > max {
							max, found = cs, &pixels[i].LongTerm[j].Bank[k]
						}
					}
				}
				pixels[i].Index = (pixels[i].Index + 1) % len(pixels[i].Buffer)
				next := pixels[i].Buffer[pixels[i].Index]
				for j := range pixels[i].LongTerm {
					if pixels[i].LongTerm[j].RegisterSet {
						for k := range pixels[i].LongTerm[j].Register.Vector {
							pixels[i].LongTerm[j].Register.Vector[k] = (pixels[i].LongTerm[j].Register.Vector[k] + next.Vector[k])
						}
						for k := range pixels[i].LongTerm[j].Register.Actions {
							pixels[i].LongTerm[j].Register.Actions[k] += next.Actions[k]
						}
						pixels[i].LongTerm[j].Index = (pixels[i].LongTerm[j].Index + 1) % len(pixels[i].LongTerm[j].Bank)
						next = pixels[i].LongTerm[j].Bank[pixels[i].LongTerm[j].Index]
						pixels[i].LongTerm[j].Bank[pixels[i].LongTerm[j].Index] = pixels[i].LongTerm[j].Register
						pixels[i].LongTerm[j].Register = Item{}
						pixels[i].LongTerm[j].RegisterSet = false
					} else {
						pixels[i].LongTerm[j].Register = next
						pixels[i].LongTerm[j].RegisterSet = true
						break
					}
				}
				pixels[i].Buffer[pixels[i].Index].Vector = query
				inputs = append(inputs, query)
				items = append(items, found)
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
			distro := [ActionCount]float32{}
			for i := range pixels {
				actions, sum := items[i].Actions, 0
				for _, v := range actions {
					sum += v
				}
				for j, v := range actions {
					distro[j] += float32(v) * embedding[i] / float32(sum)
				}
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
				pixels[i].Buffer[pixels[i].Index].Actions = [ActionCount]int{}
				pixels[i].Buffer[pixels[i].Index].Actions[action] = 1
			}

			indexes <- distro
		}
	}

	go hemisphere(false, 1, left)
	go hemisphere(false, 2, right)
	go hemisphere(true, 1, dleft)
	go hemisphere(true, 2, dright)

	rng := rand.New(rand.NewSource(1))
	actions := make([]float32, ActionCount)
	mark := time.Now()
	for index := range indexes {
		if time.Since(mark) > time.Second {
			s := float32(0.0)
			for _, v := range actions {
				s += v
			}
			sum, selected, index := float32(0.0), rng.Float32(), 0
			for i, v := range actions {
				sum += v / s
				if selected < sum {
					index = i
					break
				}
			}
			do(TypeAction(index))
			for i := range actions {
				actions[i] = 0
			}
			mark = time.Now()
		} else {
			for i := range index {
				actions[i] += index[i]
			}
		}
	}
}

// AutoEncoder is an autoencoder
type AutoEncoder struct {
	Set       tf32.Set
	Rng       *rand.Rand
	Iteration int
}

// NewAutoEncoder creates a new autoencoder
func NewAutoEncoder() *AutoEncoder {
	a := AutoEncoder{
		Rng: rand.New(rand.NewSource(1)),
	}
	set := tf32.NewSet()
	set.Add("l1", 8*8, 8*8)
	set.Add("b1", 8*8, 1)
	set.Add("l2", 8*8, 8*8)
	set.Add("b2", 8*8, 1)

	for i := range set.Weights {
		w := set.Weights[i]
		if strings.HasPrefix(w.N, "b") {
			w.X = w.X[:cap(w.X)]
			w.States = make([][]float32, StateTotal)
			for ii := range w.States {
				w.States[ii] = make([]float32, len(w.X))
			}
			continue
		}
		factor := math.Sqrt(2.0 / float64(w.S[0]))
		for range cap(w.X) {
			w.X = append(w.X, float32(a.Rng.NormFloat64()*factor))
		}
		w.States = make([][]float32, StateTotal)
		for ii := range w.States {
			w.States[ii] = make([]float32, len(w.X))
		}
	}

	a.Set = set
	return &a
}

// Measure measures the loss
func (a *AutoEncoder) Measure(input *[128][]float32) float32 {
	others := tf32.NewSet()
	others.Add("input", 8*8, len(input))
	in := others.ByName["input"]
	for _, row := range input {
		for _, value := range row {
			in.X = append(in.X, value/255.0)
		}
	}

	l1 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l1"), others.Get("input")), a.Set.Get("b1")))
	l2 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l2"), l1), a.Set.Get("b2")))
	loss := tf32.Sum(tf32.Quadratic(l2, others.Get("input")))

	l := float32(0.0)
	loss(func(a *tf32.V) bool {
		l = a.X[0]
		return true
	})
	return l
}

// Measure measures the loss of a single input
func (a *AutoEncoder) MeasureSingle(input, output []float32) float32 {
	others := tf32.NewSet()
	others.Add("input", 8*8, 1)
	others.Add("output", 8*8, 1)
	in := others.ByName["input"]
	for _, value := range input {
		in.X = append(in.X, value/255.0)
	}
	out := others.ByName["output"]
	for _, value := range output {
		out.X = append(out.X, value)
	}

	l1 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l1"), others.Get("input")), a.Set.Get("b1")))
	l2 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l2"), l1), a.Set.Get("b2")))
	loss := tf32.Sum(tf32.Quadratic(l2, others.Get("output")))

	l := float32(0.0)
	loss(func(a *tf32.V) bool {
		l = a.X[0]
		return true
	})
	return l
}

func (a *AutoEncoder) pow(x float64) float64 {
	y := math.Pow(x, float64(a.Iteration+1))
	if math.IsNaN(y) || math.IsInf(y, 0) {
		return 0
	}
	return y
}

// Encode encodes
func (a *AutoEncoder) Encode(input *[128][]float32) float32 {
	others := tf32.NewSet()
	others.Add("input", 8*8, len(input))
	in := others.ByName["input"]
	for _, row := range input {
		for _, value := range row {
			in.X = append(in.X, value/255.0)
		}
	}

	l1 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l1"), others.Get("input")), a.Set.Get("b1")))
	l2 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l2"), l1), a.Set.Get("b2")))
	loss := tf32.Avg(tf32.Quadratic(l2, others.Get("input")))

	l := float32(0.0)
	a.Set.Zero()
	others.Zero()
	l = tf32.Gradient(loss).X[0]
	if math.IsNaN(float64(l)) || math.IsInf(float64(l), 0) {
		fmt.Println(a.Iteration, l)
		return 0
	}

	norm := 0.0
	for _, p := range a.Set.Weights {
		for _, d := range p.D {
			norm += float64(d * d)
		}
	}
	norm = math.Sqrt(norm)
	b1, b2 := a.pow(B1), a.pow(B2)
	scaling := 1.0
	if norm > 1 {
		scaling = 1 / norm
	}
	for _, w := range a.Set.Weights {
		for ii, d := range w.D {
			g := d * float32(scaling)
			m := B1*w.States[StateM][ii] + (1-B1)*g
			v := B2*w.States[StateV][ii] + (1-B2)*g*g
			w.States[StateM][ii] = m
			w.States[StateV][ii] = v
			mhat := m / (1 - float32(b1))
			vhat := v / (1 - float32(b2))
			if vhat < 0 {
				vhat = 0
			}
			w.X[ii] -= Eta * mhat / (float32(math.Sqrt(float64(vhat))) + 1e-8)
		}
	}
	a.Iteration++
	return l
}

// Encode encodes a single input
func (a *AutoEncoder) EncodeSingle(input, output []float32, rng *rand.Rand) float32 {
	others := tf32.NewSet()
	others.Add("input", 8*8, 1)
	others.Add("output", 8*8, 1)
	in := others.ByName["input"]
	for _, value := range input {
		in.X = append(in.X, value)
	}
	out := others.ByName["output"]
	for _, value := range output {
		out.X = append(out.X, value)
	}

	dropout := map[string]interface{}{
		"rng": rng,
	}

	l1 := tf32.Dropout(tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l1"), others.Get("input")), a.Set.Get("b1"))), dropout)
	l2 := tf32.Sigmoid(tf32.Add(tf32.Mul(a.Set.Get("l2"), l1), a.Set.Get("b2")))
	loss := tf32.Avg(tf32.Quadratic(l2, others.Get("output")))

	l := float32(0.0)
	a.Set.Zero()
	others.Zero()
	l = tf32.Gradient(loss).X[0]
	if math.IsNaN(float64(l)) || math.IsInf(float64(l), 0) {
		fmt.Println(a.Iteration, l)
		return 0
	}

	norm := 0.0
	for _, p := range a.Set.Weights {
		for _, d := range p.D {
			norm += float64(d * d)
		}
	}
	norm = math.Sqrt(norm)
	b1, b2 := a.pow(B1), a.pow(B2)
	scaling := 1.0
	if norm > 1 {
		scaling = 1 / norm
	}
	for _, w := range a.Set.Weights {
		for ii, d := range w.D {
			g := d * float32(scaling)
			m := B1*w.States[StateM][ii] + (1-B1)*g
			v := B2*w.States[StateV][ii] + (1-B2)*g*g
			w.States[StateM][ii] = m
			w.States[StateV][ii] = v
			mhat := m / (1 - float32(b1))
			vhat := v / (1 - float32(b2))
			if vhat < 0 {
				vhat = 0
			}
			w.X[ii] -= Eta * mhat / (float32(math.Sqrt(float64(vhat))) + 1e-8)
		}
	}
	a.Iteration++
	return l
}

// AutoEncoderMind is a mind
func AutoEncoderMind(frames chan Frame, do func(action TypeAction)) {
	rng := rand.New(rand.NewSource(1))
	var auto [actions]Auto
	for i := range auto {
		auto[i].Auto = NewAutoEncoder()
		auto[i].Action = TypeAction(i)
	}

	var votes [actions]uint

	img := <-frames
	width := img.Frame.Bounds().Max.X
	height := img.Frame.Bounds().Max.Y
	w, h := width/8, height/8
	fmt.Println(width, height, w, h, w*h)

	iteration := 0
	for img := range frames {
		width := img.Frame.Bounds().Max.X
		height := img.Frame.Bounds().Max.Y
		var l [actions]float32
		var p [128][]float32
		for i := range p {
			x := rng.Intn(width - 8)
			y := rng.Intn(height - 8)
			pixels := make([]float32, 8*8)
			for yy := 0; yy < 8; yy++ {
				for xx := 0; xx < 8; xx++ {
					pixel := img.GrayAt(x+xx, y+yy)
					pixels[yy*8+xx] = float32(pixel.Y)
				}
			}
			p[i] = pixels
		}
		for i := range auto {
			l[i] = auto[i].Auto.Measure(&p)
		}
		min, max, index, index1 := float32(math.MaxFloat32), float32(0.0), 0, 0
		for ii, value := range l {
			if value > max {
				max, index = value, ii
			}
			if value < min {
				min, index1 = value, ii
			}
		}
		votes[index1]++
		if iteration%30 == 0 {
			max, index := uint(0), 0
			for ii, value := range votes {
				if value > max {
					max, index = value, ii
				}
				votes[ii] = 0
			}
			do(auto[index].Action)
		}
		auto[index].Auto.Encode(&p)
		iteration++
	}
}

// AutoEncoderMind is a mind mach 2
func AutoEncoderMindMach2(frames chan Frame, do func(action TypeAction)) {
	rng := rand.New(rand.NewSource(1))
	img := <-frames
	width := img.Frame.Bounds().Max.X
	height := img.Frame.Bounds().Max.Y
	w, h := width/8, height/8
	fmt.Println(width, height, w, h, w*h)

	auto := make([][actions]Auto, w*h)
	for i := range auto {
		for ii := range auto[i] {
			auto[i][ii].Auto = NewAutoEncoder()
			auto[i][ii].Action = TypeAction(i)
		}
	}

	var votes [actions]uint

	iteration := 0
	for img := range frames {
		width := img.Frame.Bounds().Max.X
		height := img.Frame.Bounds().Max.Y
		type Patch struct {
			Input  []float32
			Output []float32
		}
		pixels := make([]Patch, 0, 8)
		mask, s := make(map[int]bool), 0
		for y := 0; y < height-8; y += 8 {
			for x := 0; x < width-8; x += 8 {
				input, output := make([]float32, 8*8), make([]float32, 8*8)
				for yy := 0; yy < 8; yy++ {
					for xx := 0; xx < 8; xx++ {
						pixel := float32(img.GrayAt(x+xx, y+yy).Y) / 255
						output[yy*8+xx] = pixel
						pixel += float32(rng.NormFloat64() / 8)
						if pixel < 0 {
							pixel = 0
						}
						if pixel > 1 {
							pixel = 1
						}
						input[yy*8+xx] = pixel
					}
				}
				pixels = append(pixels, Patch{
					Input:  input,
					Output: output,
				})
				if rng.Intn(8) == 0 {
					mask[s] = true
				}
				s++
			}
		}

		done := make(chan int, 8)
		measure := func(i int, seed int64) {
			rng := rand.New(rand.NewSource(seed))
			if !mask[i] {
				done <- -1
				return
			}
			min, max, minIndex, maxIndex := float32(math.MaxFloat32), float32(0), 0, 0
			for ii := range auto[i] {
				value := auto[i][ii].Auto.MeasureSingle(pixels[i].Input, pixels[i].Output)
				if value < min {
					min, minIndex = value, ii
				}
				if value > max {
					max, maxIndex = value, ii
				}
			}
			auto[i][maxIndex].Auto.EncodeSingle(pixels[i].Input, pixels[i].Output, rng)
			done <- minIndex
		}
		index, flight, cpus := 0, 0, runtime.NumCPU()
		for index < len(pixels) && flight < cpus {
			go measure(index, rng.Int63())
			flight++
			index++
		}
		for index < len(pixels) {
			act := <-done
			if act >= 0 {
				votes[act]++
			}
			flight--

			go measure(index, rng.Int63())
			flight++
			index++
		}
		for range flight {
			act := <-done
			if act >= 0 {
				votes[act]++
			}
		}
		if iteration%15 == 0 {
			max, action := uint(0), 0
			for ii, value := range votes {
				if value > max {
					max, action = value, ii
				}
				votes[ii] = 0
			}
			go do(TypeAction(action))
		}

		iteration++
	}
}

// AutoEncoderMind is a mind mach 3
func AutoEncoderMindMach3(frames chan Frame, do func(action TypeAction)) {
	rng := rand.New(rand.NewSource(1))
	img := <-frames
	width := img.Frame.Bounds().Max.X
	height := img.Frame.Bounds().Max.Y
	w, h := width/8, height/8
	fmt.Println(width, height, w, h, w*h)

	auto := make([][actions][actions]Auto, w*h)
	for i := range auto {
		for ii := range auto[i] {
			for iii := range auto[i][ii] {
				auto[i][ii][iii].Auto = NewAutoEncoder()
				auto[i][ii][iii].Action = TypeAction(i)
			}
		}
	}

	var votes [actions]float32

	iteration := 0
	last := TypeAction(0)
	for img := range frames {
		width := img.Frame.Bounds().Max.X
		height := img.Frame.Bounds().Max.Y
		type Patch struct {
			Input   []float32
			Output  []float32
			Entropy float32
		}
		pixels := make([]Patch, 0, 8)
		mask, s := make(map[int]bool), 0
		for y := 0; y < height-8; y += 8 {
			for x := 0; x < width-8; x += 8 {
				input, output := make([]float32, 8*8), make([]float32, 8*8)
				var histogram [256]float32
				for yy := 0; yy < 8; yy++ {
					for xx := 0; xx < 8; xx++ {
						g := img.GrayAt(x+xx, y+yy)
						pixel := float32(g.Y) / 255
						output[yy*8+xx] = pixel
						pixel += float32(rng.NormFloat64() / 16)
						if pixel < 0 {
							pixel = 0
						}
						if pixel > 1 {
							pixel = 1
						}
						input[yy*8+xx] = pixel
						histogram[g.Y]++
					}
				}
				entropy := float32(0.0)
				for _, value := range histogram {
					if value == 0 {
						continue
					}
					entropy += (value / (float32(8 * 8))) * float32(math.Log2(float64(value)/float64(8*8)))
				}
				pixels = append(pixels, Patch{
					Input:   input,
					Output:  output,
					Entropy: -entropy,
				})
				if rng.Intn(4) == 0 {
					mask[s] = true
				}
				s++
			}
		}

		type Vote struct {
			Min     int
			Max     int
			Entropy float32
		}
		done := make(chan Vote, 8)
		measure := func(i int, seed int64) {
			rng := rand.New(rand.NewSource(seed))
			if !mask[i] {
				done <- Vote{
					Min: -1,
					Max: -1,
				}
				return
			}
			min, max, minIndex, maxIndex := float32(math.MaxFloat32), float32(0), 0, 0
			for ii := range auto[i] {
				value := auto[i][last][ii].Auto.MeasureSingle(pixels[i].Input, pixels[i].Output)
				if value < min {
					min, minIndex = value, ii
				}
				if value > max {
					max, maxIndex = value, ii
				}
			}
			auto[i][last][maxIndex].Auto.EncodeSingle(pixels[i].Input, pixels[i].Output, rng)
			done <- Vote{
				Min:     minIndex,
				Max:     maxIndex,
				Entropy: pixels[i].Entropy,
			}
		}
		index, flight, cpus := 0, 0, runtime.NumCPU()
		for index < len(pixels) && flight < cpus {
			go measure(index, rng.Int63())
			flight++
			index++
		}
		for index < len(pixels) {
			act := <-done
			if act.Min >= 0 {
				votes[act.Min] += act.Entropy
			}
			if act.Max >= 0 {
				votes[act.Max] += act.Entropy
			}
			flight--

			go measure(index, rng.Int63())
			flight++
			index++
		}
		for range flight {
			act := <-done
			if act.Min >= 0 {
				votes[act.Min]++
			}
			if act.Max >= 0 {
				votes[act.Max]++
			}
		}
		if iteration%15 == 0 {
			max, action := float32(0.0), 0
			for ii, value := range votes {
				if value > max {
					max, action = value, ii
				}
				votes[ii] = 0
			}
			go do(TypeAction(action))
			last = TypeAction(action)
		}

		iteration++
	}
}

func main() {
	flag.Parse()

	var robot Robot
	if *FlagRobot == "turbopi" {
		robot = &Turbopi{}
	} else if *FlagRobot == "waveshare" {
		robot = &Waveshare{}
	} else {
		robot = &Urobot{}
	}

	robot.Init()

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-c
		robot.Done()
		os.Exit(1)
	}()

	if *FlagDoom {
		game := &DoomGame{}
		last, autoMode := ActionCount, false
		game.frames = make(chan Frame, 8)
		game.autoMode = func() {
			autoMode = !autoMode
			if !autoMode && last != ActionCount {
				var event gore.DoomEvent
				event.Type = gore.Ev_keyup
				switch last {
				case ActionLeft:
					event.Key = gore.KEY_LEFTARROW1
				case ActionRight:
					event.Key = gore.KEY_RIGHTARROW1
				case ActionForward:
					event.Key = gore.KEY_UPARROW1
				case ActionBackward:
					event.Key = gore.KEY_DOWNARROW1
				case ActionNone:
				}
				game.events = append(game.events, event)
				last = ActionCount
			}

		}
		ebiten.SetWindowSize(screenWidth, screenHeight)
		ebiten.SetWindowTitle("Gamepad (Ebitengine Demo)")
		ebiten.SetFullscreen(true)
		do := func(action TypeAction) {
			game.lock.Lock()
			defer game.lock.Unlock()
			if autoMode && last != ActionCount {
				var event gore.DoomEvent
				event.Type = gore.Ev_keyup
				switch last {
				case ActionLeft:
					event.Key = gore.KEY_LEFTARROW1
				case ActionRight:
					event.Key = gore.KEY_RIGHTARROW1
				case ActionForward:
					event.Key = gore.KEY_UPARROW1
				case ActionBackward:
					event.Key = gore.KEY_DOWNARROW1
				case ActionNone:
				}
				game.events = append(game.events, event)
			}
			if autoMode {
				var event gore.DoomEvent
				event.Type = gore.Ev_keydown
				switch action {
				case ActionLeft:
					event.Key = gore.KEY_LEFTARROW1
				case ActionRight:
					event.Key = gore.KEY_RIGHTARROW1
				case ActionForward:
					event.Key = gore.KEY_UPARROW1
				case ActionBackward:
					event.Key = gore.KEY_DOWNARROW1
				case ActionNone:
				}
				game.events = append(game.events, event)
			}
			last = action
		}
		go func() {
			gore.Run(game, []string{"-iwad", *FlagIwad})
			game.terminating = true
		}()
		if *FlagMind == "one" {
			go AutoEncoderMind(game.frames, do)
		} else if *FlagMind == "two" {
			go AutoEncoderMindMach2(game.frames, do)
		} else if *FlagMind == "three" {
			go AutoEncoderMindMach3(game.frames, do)
		}
		if err := ebiten.RunGame(game); err != nil {
			panic(err)
		}

		return
	}

	//Mind(robot.Do)
	camera := NewV4LCamera()
	go camera.Start("/dev/video0")
	AutoEncoderMind(camera.Images, robot.Do)
}
