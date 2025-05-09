// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
// "github.com/alixaxel/pagerank"
)

const (
	// Size is the number of histograms
	Size = 13
)

// Histogram is a buffered histogram
type Histogram struct {
	Vector [256]uint16
	Buffer [4096]byte
	Index  int
	Size   int
}

// NewHistogram make a new histogram
func NewHistogram(size int) Histogram {
	h := Histogram{
		Size: size,
	}
	return h
}

// Add adds a symbol to the histogram
func (h *Histogram) Add(s byte) {
	index := (h.Index + 1) % h.Size
	if symbol := h.Buffer[index]; h.Vector[symbol] > 0 {
		h.Vector[symbol]--
	}
	h.Buffer[index] = s
	h.Vector[s]++
	h.Index = index
}

// Mixer mixes several histograms together
type Mixer struct {
	Histograms []Histogram
}

// NewMixer makes a new mixer
func NewMixer() Mixer {
	histograms := make([]Histogram, Size)
	histograms[0] = NewHistogram(1)
	histograms[1] = NewHistogram(2)
	histograms[2] = NewHistogram(4)
	histograms[3] = NewHistogram(8)
	histograms[4] = NewHistogram(16)
	histograms[5] = NewHistogram(32)
	histograms[6] = NewHistogram(64)
	histograms[7] = NewHistogram(128)
	histograms[8] = NewHistogram(256)
	histograms[9] = NewHistogram(512)
	histograms[10] = NewHistogram(1024)
	histograms[11] = NewHistogram(2048)
	histograms[12] = NewHistogram(4096)
	return Mixer{
		Histograms: histograms,
	}
}

func (m Mixer) Copy() Mixer {
	histograms := make([]Histogram, Size)
	for i := range m.Histograms {
		histograms[i] = m.Histograms[i]
	}
	return Mixer{
		Histograms: histograms,
	}
}

// Add adds a symbol to a mixer
func (m *Mixer) Add(s byte) {
	for i := range m.Histograms {
		m.Histograms[i].Add(s)
	}
}

/*func (m Mixer) Mix() *[Size]float32 {
	output := [Size]float32{}
	x := NewMatrix(256, Size)
	for i := range m.Histograms {
		sum := float32(0.0)
		for _, v := range m.Histograms[i].Vector {
			sum += float32(v)
		}
		for _, v := range m.Histograms[i].Vector {
			x.Data = append(x.Data, float32(v)/sum)
		}
	}
	graph := pagerank.NewGraph()
	for i := 0; i < Size; i++ {
		a := x.Data[i*256 : i*256+256]
		for j := 0; j < Size; j++ {
			b := x.Data[j*256 : j*256+256]
			cs := CS(a, b)
			graph.Link(uint32(i), uint32(j), float64(cs))
		}
	}
	graph.Rank(1.0, 1e-3, func(node uint32, rank float64) {
		output[node] = float32(rank)
	})
	return &output
}*/

// Mix mixes the histograms outputting a matrix
func (m Mixer) Mix() *[256]float32 {
	output := [256]float32{}
	x := NewMatrix(256, Size)
	for i := range m.Histograms {
		sum := float32(0.0)
		for _, v := range m.Histograms[i].Vector {
			sum += float32(v)
		}
		for _, v := range m.Histograms[i].Vector {
			x.Data = append(x.Data, float32(v)/sum)
		}
	}
	SelfAttention(x, &output)
	return &output
}
