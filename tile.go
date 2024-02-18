package main

import (
	"sync"
)

const (
	R = 2
	G = 1
	B = 0
)

const (
	BarycentricAlgorithm = 0

	EdgeTestAlgorithm = 1

	SweepLineAlgorithm = 2
)

var AlgorithmUsed int = SweepLineAlgorithm

var Pitch int
var BytesPerPixel int
var WaitGroup sync.WaitGroup

type Shader func(s, t, w float32) (float32, float32, float32)

type Tile struct {
	Frame []byte

	Triangles []ProcessedTriangle

	X, Y int
}

func (tile *Tile) Barycentric(triangle *ProcessedTriangle) {
	for y := tile.Y; y < tile.Y+TileYSize-1; y++ {
		for x := tile.X; x < tile.X+TileXSize-1; x++ {
			if inside, s, t, w := triangle.Inside(x, y); inside {
				//var r, g, b float32 = shader(s, t, w)
				tile.Set(x, y, byte(s*255), byte(t*255), byte(w*255))
			}
		}
	}
}

func (tile *Tile) RasterizeChunk(triangle *ProcessedTriangle) {
	switch AlgorithmUsed {
	//case SweepLineAlgorithm:
	//	tile.SweepLine(triangle)
	//	break
	case BarycentricAlgorithm:
		tile.Barycentric(triangle)
		break

		//case EdgeTestAlgorithm:
		//	tile.EdgeTest(triangle)
		//	break
	}
}

func (tile *Tile) Rasterize(shader Shader) {
	for index := range tile.Triangles {
		tile.RasterizeChunk(&tile.Triangles[index])
	}

	tile.Triangles = nil

	WaitGroup.Done()
}

func (tile *Tile) Add(triangle *Triangle) {
	tile.Triangles = append(tile.Triangles, Process(triangle))
}

func (tile *Tile) Set(x, y int, r, g, b byte) {
	var position int = y*Pitch + x*BytesPerPixel

	tile.Frame[position+B] = b
	tile.Frame[position+G] = g
	tile.Frame[position+R] = r
}

func (tile *Tile) Clear(r, g, b byte) {
	for y := tile.Y; y < tile.Y+TileYSize; y++ {
		for x := tile.X; x < tile.X+TileXSize; x++ {
			tile.Set(x, y, r, g, b)
		}
	}

	WaitGroup.Done()
}
