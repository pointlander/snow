// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"math"
	"math/rand"

	"github.com/alixaxel/pagerank"

	htgotts "github.com/hegedustibor/htgo-tts"
	handlers "github.com/hegedustibor/htgo-tts/handlers"
	voices "github.com/hegedustibor/htgo-tts/voices"
)

// Pixel is a pixel sensor
type Pixel struct {
	X, Y  int
	Mixer Mixer
}

func main() {
	speech := htgotts.Speech{Folder: "audio", Language: voices.English, Handler: &handlers.MPlayer{}}
	speech.Speak("Starting...")

	camera := NewV4LCamera()
	say := make(chan string, 8)
	go func() {
		for s := range say {
			speech.Speak(s)
		}
	}()

	rng := rand.New(rand.NewSource(1))
	u := NewMatrix(256, 8)
	for i := 0; i < u.Cols*u.Rows; i++ {
		u.Data = append(u.Data, rng.Float32())
	}

	var pixels []Pixel
	go camera.Start("/dev/video0")
	count := 0
	for img := range camera.Images {
		width := img.Gray.Bounds().Max.X
		height := img.Gray.Bounds().Max.Y
		if pixels == nil {
			for i := 0; i < 256; i++ {
				mixer := NewMixer()
				x := rng.Intn(width)
				y := rng.Intn(height)
				pixels = append(pixels, Pixel{
					X:     x,
					Y:     y,
					Mixer: mixer,
				})
			}
		}
		inputs := []*[Size]float32{}
		for i := range pixels {
			pixel := img.Gray.GrayAt(pixels[i].X, pixels[i].Y)
			pixels[i].Mixer.Add(pixel.Y)
			inputs = append(inputs, pixels[i].Mixer.Mix())
		}
		{
			graph := pagerank.NewGraph()
			for i := 0; i < len(inputs); i++ {
				for j := 0; j < len(inputs); j++ {
					p := CS(inputs[i][:], inputs[j][:])
					graph.Link(uint32(i), uint32(j), float64(p))
				}
			}
			graph.Rank(1.0, 1e-3, func(node uint32, rank float64) {
				u.Data[node] = float32(rank)
			})
		}

		graph := pagerank.NewGraph()
		for i := 0; i < u.Rows; i++ {
			for j := 0; j < u.Rows; j++ {
				p := CS(u.Data[i*u.Cols:(i+1)*u.Cols], u.Data[j*u.Cols:(j+1)*u.Cols])
				graph.Link(uint32(i), uint32(j), float64(p))
			}
		}
		ranks := make([]float64, u.Rows)
		graph.Rank(1.0, 1e-3, func(node uint32, rank float64) {
			ranks[node] = rank
		})
		min, index := math.MaxFloat64, 0
		for i, v := range ranks {
			if v < min {
				min, index = v, i
			}
		}
		for i, v := range ranks {
			u.Data[index*u.Cols+i] = float32(v)
		}
		if count%(30) == 0 {
			switch index {
			case 5:
				say <- "left"
			case 6:
				say <- "right"
			case 7:
				say <- "straight"
			}
		}
		count++
	}
}
