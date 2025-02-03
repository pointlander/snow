// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
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
	u := NewMatrix(256, 256)
	for i := 0; i < u.Cols*u.Rows; i++ {
		u.Data = append(u.Data, rng.Float32())
	}

	var pixels []Pixel
	go camera.Start("/dev/video0")
	count := 0
	for img := range camera.Images {
		width := img.Frame.Bounds().Max.X
		height := img.Frame.Bounds().Max.Y
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
			pixel := img.GrayAt(pixels[i].X, pixels[i].Y)
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
		index, sum, selected := 0, 0.0, rng.Float64()
		for i, v := range ranks {
			sum += v
			if selected < sum {
				index = i
				break
			}
		}
		for i, v := range ranks {
			u.Data[index*u.Cols+i] = float32(v)
		}
		switch {
		case index%3 == 0:
			say <- "left"
		case index%3 == 1:
			say <- "right"
		case index%3 == 2:
			say <- "straight"
		}
		count++
	}
}
