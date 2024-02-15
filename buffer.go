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

func Clamp(value float32, min, max int) float32 {
	if value < float32(min) {
		return float32(min)
	} else if value > float32(max) {
		return float32(max)
	}

	return value
}

func (buffer *Buffer) Fragment(triangle *Triangle, vs1, vs2 *Vertex, span float32, x, y int) {
	s, t, w := triangle.Barycentric(vs1, vs2, span, x, y)

	var r float32 = s
	var g float32 = t
	var b float32 = w

	buffer.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
}

func (buffer *Buffer) RasterizeBottomFlat(triangle *Triangle, split, vs1, vs2 *Vertex, span float32, tx, ty int) {
	var invSlope1 float32 = (triangle.Vertices[1][X] - triangle.Vertices[0][X]) / (triangle.Vertices[1][Y] - triangle.Vertices[0][Y])
	var invSlope2 float32 = ((*split)[X] - triangle.Vertices[0][X]) / ((*split)[Y] - triangle.Vertices[0][Y])

	var clampedUp float32 = Clamp(triangle.Vertices[0][Y], ty, ty+TileYSize)
	var difference float32 = clampedUp - triangle.Vertices[0][Y]

	var curX1, curX2 float32 = triangle.Vertices[0][X] + invSlope1*difference,
		triangle.Vertices[0][X] + invSlope2*difference

	for y := int(clampedUp); y <= int(Clamp(triangle.Vertices[1][Y], ty, ty+TileYSize)); y++ {
		for x := int(Clamp(curX1, tx, tx+TileXSize)); x < int(Clamp(curX2, tx, tx+TileXSize)); x++ {
			buffer.Fragment(triangle, vs1, vs2, span, x, y)
		}

		curX1 += invSlope1
		curX2 += invSlope2
	}
}

func (buffer *Buffer) RasterizeTopFlat(triangle *Triangle, split, vs1, vs2 *Vertex, span float32, tx, ty int) {
	var invSlope1 float32 = (triangle.Vertices[2][X] - triangle.Vertices[1][X]) / (triangle.Vertices[2][Y] - triangle.Vertices[1][Y])
	var invSlope2 float32 = (triangle.Vertices[2][X] - (*split)[X]) / (triangle.Vertices[2][Y] - (*split)[Y])

	var clampedDown float32 = Clamp(triangle.Vertices[2][Y], ty, ty+TileYSize)
	var difference float32 = triangle.Vertices[2][Y] - clampedDown

	var curX1, curX2 float32 = triangle.Vertices[2][X] - invSlope1*difference,
		triangle.Vertices[2][X] - invSlope2*difference

	for y := int(clampedDown); y > int(Clamp(triangle.Vertices[1][Y], ty, ty+TileYSize)); y-- {
		for x := int(Clamp(curX1, tx, tx+TileXSize)); x < int(Clamp(curX2, tx, tx+TileXSize)); x++ {
			buffer.Fragment(triangle, vs1, vs2, span, x, y)
		}

		curX1 -= invSlope1
		curX2 -= invSlope2
	}
}

func (buffer *Buffer) RasterizeChunk(triangle *Triangle, split, vs1, vs2 *Vertex, span float32, tx, ty int) {
	buffer.RasterizeBottomFlat(triangle, split, vs1, vs2, span, tx, ty)
	buffer.RasterizeTopFlat(triangle, split, vs1, vs2, span, tx, ty)

	buffer.WaitGroup.Done()
}

func (buffer *Buffer) Rasterize(triangle *Triangle) {
	triangle.Sort()

	var split Vertex = Vertex{
		triangle.Vertices[0][X] + ((triangle.Vertices[1][Y]-triangle.Vertices[0][Y])/(triangle.Vertices[2][Y]-triangle.Vertices[0][Y]))*(triangle.Vertices[2][X]-triangle.Vertices[0][X]),
		triangle.Vertices[1][Y],
	}

	split.ScreenSpace()
	triangle.ScreenSpace()

	vs1, vs2 := triangle.Span()
	var span float32 = 1 / vs1.CrossProduct(&vs2)

	buffer.WaitGroup.Add(12)

	for ty := 0; ty < Height; ty += TileYSize {
		for tx := 0; tx < Width; tx += TileXSize {
			go buffer.RasterizeChunk(triangle, &split, &vs1, &vs2, span, tx, ty)
		}
	}

	buffer.WaitGroup.Wait()
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