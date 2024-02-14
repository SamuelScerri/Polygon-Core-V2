package main

import (
	"sync"
)

const (
	R = 2
	G = 1
	B = 0
)

type Buffer struct {
	Frame []byte
	Depth []float32

	Pitch         int
	BytesPerPixel int
	WaitGroup     sync.WaitGroup
}

func (buffer *Buffer) Rasterize(triangle *Triangle) {
	var sortedTriangle Triangle = triangle.Sort().ScreenSpace()

	if sortedTriangle.vertices[1][Y] == sortedTriangle.vertices[2][Y] {
		var invSlope1 float32 = (sortedTriangle.vertices[1][X] - sortedTriangle.vertices[0][X]) / (sortedTriangle.vertices[1][Y] - sortedTriangle.vertices[0][Y])
		var invSlope2 float32 = (sortedTriangle.vertices[2][X] - sortedTriangle.vertices[0][X]) / (sortedTriangle.vertices[2][Y] - sortedTriangle.vertices[0][Y])

		var curX1, curX2 float32 = sortedTriangle.vertices[0][X], sortedTriangle.vertices[0][X]

		for y := int(sortedTriangle.vertices[0][Y]); y <= int(sortedTriangle.vertices[1][Y]); y++ {
			for x := int(curX1); x < int(curX2); x++ {
				buffer.Set(x, y, 255, 255, 255)
			}

			curX1 += invSlope1
			curX2 += invSlope2
		}
	}
}

func (buffer *Buffer) Set(x, y int, r, g, b byte) {
	var position int = y*buffer.Pitch + x*buffer.BytesPerPixel

	buffer.Frame[position+B] = b
	buffer.Frame[position+G] = g
	buffer.Frame[position+R] = r
}

func (buffer *Buffer) ClearChunk(tx, ty int, r, g, b byte) {
	for y := ty; y < ty+TileYSize; y++ {
		for x := tx; x < tx+TileXSize; x++ {
			buffer.Set(x, y, r, g, b)
		}
	}

	buffer.WaitGroup.Done()
}

func (buffer *Buffer) Clear(r, g, b byte) {
	buffer.WaitGroup.Add(Cores)

	for ty := 0; ty < Height; ty += TileYSize {
		for tx := 0; tx < Width; tx += TileXSize {
			go buffer.ClearChunk(tx, ty, r, g, b)
		}
	}

	buffer.WaitGroup.Wait()
}
