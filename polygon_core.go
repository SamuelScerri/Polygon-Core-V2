package main

import (
	"fmt"

	"github.com/chewxy/math32"
)

type Vertex2D struct {
	x, y float32
}

type Vertex3D struct {
	x, y, z, w float32
}

type Matrix [][]float32

func (vertex *Vertex2D) convertToScreenSpace(width, height uint16) Vertex2D {
	return Vertex2D{
		((vertex.x*float32(height)/float32(width) + 1) * float32(width)) / 2,
		((-vertex.y + 1) * float32(height)) / 2,
	}
}

func (vertex *Vertex3D) convertToMatrix() Matrix {
	return Matrix{
		{vertex.x},
		{vertex.y},
		{vertex.z},
		{vertex.w},
	}
}

func (m1 *Matrix) multiplyMatrix(m2 *Matrix) (result Matrix) {
	result = make(Matrix, len(*m1))

	for i := range result {
		result[i] = make([]float32, len((*m2)[0]))

		for j := range result[i] {
			for k := 0; k < len((*m1)[0]); k++ {
				result[i][j] += (*m1)[i][k] * (*m2)[k][j]
			}
		}
	}

	return
}

func createProjectionMatrix(fov, aspect, near, far float32) Matrix {
	var tangent float32 = math32.Tan(fov / 2)

	return Matrix{}
}

func main() {
	fmt.Println("Initializing Polygon Core")
}
