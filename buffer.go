package main

import (
	"sync"
)

const (
	R = 2
	G = 1
	B = 0
)

type Shader func(s, t, w float32) (float32, float32, float32)

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

func (buffer *Buffer) Barycentric(triangle *Triangle, vs1, vs2 *Vertex, span float32, tx, ty int, shader Shader) {
	for y := ty; y < ty+TileYSize; y++ {
		for x := tx; x < tx+TileXSize; x++ {
			var s, t, w float32 = triangle.Barycentric(vs1, vs2, span, x, y)

			if s >= 0 && t >= 0 && s+t <= 1 {
				var r, g, b float32 = shader(s, t, w)
				buffer.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
			}
		}
	}
}

func (buffer *Buffer) SweepLine(triangle *Triangle, split, vs1, vs2 *Vertex, span float32, tx, ty int, shader Shader) {
	var invSlope1 float32 = (triangle.Vertices[1][X] - triangle.Vertices[0][X]) / (triangle.Vertices[1][Y] - triangle.Vertices[0][Y])
	var invSlope2 float32 = ((*split)[X] - triangle.Vertices[0][X]) / (triangle.Vertices[1][Y] - triangle.Vertices[0][Y])

	var clampedUp float32 = Clamp(triangle.Vertices[0][Y], ty, ty+TileYSize)
	var clampedMiddle float32 = Clamp(triangle.Vertices[1][Y], ty, ty+TileYSize)
	var clampedDown float32 = Clamp(triangle.Vertices[2][Y], ty, ty+TileYSize)

	var difference float32 = clampedUp - triangle.Vertices[0][Y]
	var curX1, curX2 float32 = triangle.Vertices[0][X] + invSlope1*difference, triangle.Vertices[0][X] + invSlope2*difference

	for y := int(clampedUp); y < int(clampedMiddle); y++ {
		for x := int(Clamp(curX1, tx, tx+TileXSize)); x < int(Clamp(curX2, tx, tx+TileXSize)); x++ {
			var s, t, w float32 = triangle.Barycentric(vs1, vs2, span, x, y)
			var r, g, b float32 = shader(s, t, w)

			buffer.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
		}

		curX1 += invSlope1
		curX2 += invSlope2
	}

	invSlope1 = (triangle.Vertices[2][X] - triangle.Vertices[1][X]) / (triangle.Vertices[2][Y] - triangle.Vertices[1][Y])
	invSlope2 = (triangle.Vertices[2][X] - (*split)[X]) / (triangle.Vertices[2][Y] - triangle.Vertices[1][Y])

	difference = triangle.Vertices[2][Y] - clampedDown
	curX1, curX2 = triangle.Vertices[2][X]-invSlope1*difference, triangle.Vertices[2][X]-invSlope2*difference

	for y := int(clampedDown) - 1; y > int(clampedMiddle)-1; y-- {
		for x := int(Clamp(curX1, tx, tx+TileXSize)); x < int(Clamp(curX2, tx, tx+TileXSize)); x++ {
			var s, t, w float32 = triangle.Barycentric(vs1, vs2, span, x, y)
			var r, g, b float32 = shader(s, t, w)

			buffer.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
		}

		curX1 -= invSlope1
		curX2 -= invSlope2
	}
}

func (buffer *Buffer) RasterizeChunk(triangle *Triangle, split, vs1, vs2 *Vertex, span float32, tx, ty int, shader Shader) {
	//buffer.SweepLine(triangle, split, vs1, vs2, span, tx, ty, shader)
	buffer.Barycentric(triangle, vs1, vs2, span, tx, ty, shader)
	buffer.WaitGroup.Done()
}

func (buffer *Buffer) Rasterize(triangle *Triangle, shader Shader) {
	triangle.Normalize()
	triangle.ScreenSpace()
	triangle.Sort()

	var split Vertex = Vertex{
		triangle.Vertices[0][X] + ((triangle.Vertices[1][Y]-triangle.Vertices[0][Y])/
			(triangle.Vertices[2][Y]-triangle.Vertices[0][Y]))*
			(triangle.Vertices[2][X]-triangle.Vertices[0][X]),

		triangle.Vertices[1][Y],
	}

	var vs1, vs2 Vertex = triangle.Span()
	var span float32 = 1 / vs1.CrossProduct(&vs2)

	buffer.WaitGroup.Add(12)

	for ty := 0; ty < Height; ty += TileYSize {
		for tx := 0; tx < Width; tx += TileXSize {
			go buffer.RasterizeChunk(triangle, &split, &vs1, &vs2, span, tx, ty, shader)
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
