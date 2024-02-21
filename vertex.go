package main

const (
	X = 0
	Y = 1
	Z = 2
	W = 3
)

type Vertex []float32

func (v1 *Vertex) Sum(v2 *Vertex) {
	for index, vertex := range *v1 {
		vertex += (*v2)[index]
	}
}

func (v1 *Vertex) Multiply(v2 *Vertex) {
	for index, vertex := range *v1 {
		vertex *= (*v2)[index]
	}
}

func (v1 *Vertex) Dot(v2 *Vertex) (result float32) {
	for index, vertex := range *v1 {
		result += vertex * (*v2)[index]
	}

	return
}

func (v1 *Vertex) Cross(v2 *Vertex) Vertex {
	return Vertex{
		(*v1)[Y]*(*v2)[Z] - (*v1)[Z]*(*v2)[Y],
		(*v1)[Z]*(*v2)[X] - (*v1)[X]*(*v2)[Z],
		(*v1)[X]*(*v2)[Y] - (*v1)[Y]*(*v2)[X],
	}
}

func (v1 *Vertex) CrossProduct(v2 *Vertex) float32 {
	return (*v1)[X]*(*v2)[Y] - (*v1)[Y]*(*v2)[X]
}

func (v1 *Vertex) Interpolate(v2 *Vertex, factor float32) {
	for index, vertex := range *v1 {
		(*v1)[index] = vertex*(1-factor) + (*v2)[index]*factor
	}
}

func (v1 *Vertex) ScreenSpace() {
	(*v1)[X] = ((*v1)[X] + 1) * float32(Width) / 2
	(*v1)[Y] = ((*v1)[Y] + 1) * float32(Height) / 2
}

func (v1 *Vertex) Normalize() {
	var homogeneous float32 = 1 / (*v1)[W]

	(*v1)[X] *= homogeneous
	(*v1)[Y] *= homogeneous
	(*v1)[Z] *= homogeneous
}

func (v1 *Vertex) Matrix() Matrix {
	return Matrix{(*v1)[:]}
}

func (v1 *Vertex) Swap(v2 *Vertex) {
	var temporary Vertex = *v1
	*v1 = *v2
	*v2 = temporary
}

func (v1 *Vertex) Copy() Vertex {
	var copiedVertex Vertex = make(Vertex, len(*v1))
	copy(copiedVertex, *v1)

	return copiedVertex
}
