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

	return Matrix{
		{1 / (tangent * aspect), 0, 0, 0},
		{0, 1 / tangent, 0, 0},
		{0, 0, (far + near) / (near - far), -1},
		{0, 0, (near * far * 2) / (near - far), 0},
	}
}

func main() {
	fmt.Println("Initializing Polygon Core")

	var width, height uint16 = 640, 360
	var aspect float32 = float32(width) / float32(height)

	var projectionMatrix Matrix = createProjectionMatrix(90, aspect, .1, 1000)
	var position Vertex3D = Vertex3D{1, 1, 4, 1}

	var positionConverted Matrix = position.convertToMatrix()

	fmt.Println(projectionMatrix.multiplyMatrix(&positionConverted))
}
