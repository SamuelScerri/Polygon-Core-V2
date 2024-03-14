package main

const (
	X = 0
	Y = 1
	Z = 2
	W = 3
)

type Vertex []float32

func (v1 *Vertex) Sum(v2 *Vertex) {
	for index := range *v1 {
		(*v1)[index] += (*v2)[index]
	}
}

func (v1 *Vertex) Sub(v2 *Vertex) {
	for index := range *v1 {
		(*v1)[index] -= (*v2)[index]
	}
}

func (v1 *Vertex) Multiply(v2 *Vertex) {
	for index := range *v1 {
		(*v1)[index] *= (*v2)[index]
	}
}

func (v1 *Vertex) Transform(m2 *Matrix) {
	var result Vertex = make(Vertex, len(*v1))

	for i := range *v1 {
		for j := range *m2 {
			result[i] += (*v1)[j] * (*m2)[j][i]
		}
	}

	*v1 = result
}

func (v1 *Vertex) Dot(v2 *Vertex) (result float32) {
	for index := range *v1 {
		result += (*v1)[index] * (*v2)[index]
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
	for index := range *v1 {
		(*v1)[index] += factor * ((*v2)[index] - (*v1)[index])
	}
}

func (v1 *Vertex) InsideClipSpace() bool {
	return (*v1)[X]*.25 >= -(*v1)[W] &&
		(*v1)[X]*.25 <= (*v1)[W] &&
		(*v1)[Y]*.25 >= -(*v1)[W] &&
		(*v1)[Y]*.25 <= (*v1)[W]
}

func (v1 *Vertex) ScreenSpace() {
	(*v1)[X] = (((*v1)[X] + 1) * float32(Width)) * .5
	(*v1)[Y] = ((-(*v1)[Y] + 1) * float32(Height)) * .5
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
	*v1, *v2 = *v2, *v1
}

func (v1 *Vertex) Copy() (copiedVertex Vertex) {
	return append(copiedVertex, (*v1)...)
}
