package main

import (
	"fmt"
	_ "image/jpeg"
	_ "image/png"
	"io"
	"log"
	"math"
	"os"
	"runtime"
	"sort"
	"strconv"
	"sync"

	"github.com/chewxy/math32"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

type AABB struct {
	position                              Vertex3D
	halfExtentX, halfExtentY, halfExtentZ float32
}

type Model struct {
	// For the v, vt and vn in the obj file.
	Normals, Vecs []Vertex3D
	Uvs           []Vertex2D
	TriangleData  []Triangle
	BoundingBox   [8]Vertex3D

	// For the fun "f" in the obj file.
	VecIndices, NormalIndices, UvIndices []int
}

// NewModel will read an OBJ model file and create a Model from its contents
func NewModel(file string) Model {
	// Open the file for reading and check for errors.
	objFile, err := os.Open(file)
	if err != nil {
		panic(err)
	}

	// Don't forget to close the file reader.
	defer objFile.Close()

	// Create a model to store stuff.
	model := Model{}

	// Read the file and get it's contents.
	for {
		var lineType string

		// Scan the type field.
		_, err := fmt.Fscanf(objFile, "%s", &lineType)

		// Check if it's the end of the file
		// and break out of the loop.
		if err != nil {
			if err == io.EOF {
				break
			}
		}

		// Check the type.
		switch lineType {
		// VERTICES.
		case "v":
			// Create a vec to assign digits to.
			vec := Vertex3D{}

			// Get the digits from the file.
			fmt.Fscanf(objFile, "%f %f %f\n", &vec.x, &vec.y, &vec.z)

			// Add the vector to the model.
			model.Vecs = append(model.Vecs, vec)

		// NORMALS.
		case "vn":
			// Create a vec to assign digits to.
			vec := Vertex3D{}

			// Get the digits from the file.
			fmt.Fscanf(objFile, "%f %f %f\n", &vec.x, &vec.y, &vec.z)

			// Add the vector to the model.
			model.Normals = append(model.Normals, vec)

		// TEXTURE VERTICES.
		case "vt":
			// Create a Uv pair.
			vec := Vertex2D{}

			// Get the digits from the file.
			fmt.Fscanf(objFile, "%f %f\n", &vec.x, &vec.y)

			// Add the uv to the model.
			model.Uvs = append(model.Uvs, vec)

		// INDICES.
		case "f":
			// Create a vec to assign digits to.
			norm := make([]int, 3)
			vec := make([]int, 3)
			uv := make([]int, 3)

			// Get the digits from the file.
			matches, _ := fmt.Fscanf(objFile, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vec[0], &uv[0], &norm[0], &vec[1], &uv[1], &norm[1], &vec[2], &uv[2], &norm[2])

			if matches != 9 {
				panic("Cannot read your file")
			}

			// Add the numbers to the model.
			model.NormalIndices = append(model.NormalIndices, norm[0])
			model.NormalIndices = append(model.NormalIndices, norm[1])
			model.NormalIndices = append(model.NormalIndices, norm[2])

			model.VecIndices = append(model.VecIndices, vec[0])
			model.VecIndices = append(model.VecIndices, vec[1])
			model.VecIndices = append(model.VecIndices, vec[2])

			model.UvIndices = append(model.UvIndices, uv[0])
			model.UvIndices = append(model.UvIndices, uv[1])
			model.UvIndices = append(model.UvIndices, uv[2])
		}
	}

	var longestX, longestY, longestZ float32 = math32.Abs(model.Vecs[0].x), math32.Abs(model.Vecs[0].y), math32.Abs(model.Vecs[0].z)

	for i := 1; i < len(model.Vecs); i++ {
		if math32.Abs(model.Vecs[i].x) > longestX {
			longestX = math32.Abs(model.Vecs[i].x)
		}

		if math32.Abs(model.Vecs[i].y) > longestY {
			longestY = math32.Abs(model.Vecs[i].y)
		}

		if math32.Abs(model.Vecs[i].z) > longestZ {
			longestZ = math32.Abs(model.Vecs[i].z)
		}
	}

	var newBoundingBox AABB = AABB{Vertex3D{0, 0, 0}, longestX, longestY, longestZ}
	fmt.Println(longestX, longestY, longestZ)

	for i := 0; i < len(model.VecIndices)/3; i++ {
		var tri Triangle = Triangle{
			[3]Vertex3D{model.Vecs[model.VecIndices[i*3]-1], model.Vecs[model.VecIndices[i*3+1]-1], model.Vecs[model.VecIndices[i*3+2]-1]},
			[3]Vertex2D{model.Uvs[model.UvIndices[i*3]-1], model.Uvs[model.UvIndices[i*3+1]-1], model.Uvs[model.UvIndices[i*3+2]-1]},
			[3]Vertex3D{model.Normals[model.NormalIndices[i*3]-1], model.Normals[model.NormalIndices[i*3+1]-1], model.Normals[model.NormalIndices[i*3+2]-1]},
		}

		model.TriangleData = append(model.TriangleData, tri)
	}

	//fmt.Println(model.VecIndices)
	fmt.Println(len(model.TriangleData[0].normals))

	model.BoundingBox = newBoundingBox.convertToVertices()

	// Return the newly created Model.
	return model
}

type Vertex2D struct {
	x, y float32
}

type Vertex2Di struct {
	x, y int
}

type Vertex3D struct {
	x, y, z float32
}

type Vertex4D struct {
	x, y, z, w float32
}

type Triangle struct {
	vertices [3]Vertex3D
	uv       [3]Vertex2D
	normals  [3]Vertex3D
}

type ComputedTriangle struct {
	vertices [3]Vertex4D
	uv       [3]Vertex2D
	normals  [3]Vertex3D

	at, bt, ct Vertex3D
	vs1, vs2   Vertex4D

	span float32

	minX, maxX, minY, maxY int
	na, nb, nc             Vertex3D
	fa, fb, fc             Vertex3D
}

type Buffer []byte
type FloatBuffer []float32
type Matrix [][]float32

const HT = 4
const VT = 3

type Tile []ComputedTriangle
type TileGrid [HT][VT]Tile

var tileSizeX int = width / HT
var tileSizeY int = height / VT

var screenBuffer Buffer
var depthBuffer FloatBuffer
var chunkSize, chunkSizeDepth int

var car, teapot, bunny, skull, monkey, person, cat, level Model

var width, height int = 640, 360
var aspectRatio float32 = float32(width) / float32(height)
var wg sync.WaitGroup
var mu sync.Mutex

var cobble *ebiten.Image
var cameraPosition, cameraRotation Vertex3D

var cobble_buffer Buffer
var fov float32 = 165

var projectionMatrix Matrix

func (vertex *Vertex4D) isInClipSpaceX(direction bool) bool {
	if direction {
		return vertex.x <= vertex.w
	} else {
		return vertex.x >= -vertex.w
	}
}

func (vertex *Vertex4D) isInClipSpaceY(direction bool) bool {
	if direction {
		return vertex.y <= vertex.w
	} else {
		return vertex.y >= -vertex.w
	}
}

func (boundingBox *AABB) convertToVertices() (vertices [8]Vertex3D) {
	//Left Plane: 0 2 4 6
	//Right Plane: 1 3 5 7
	//Front Plane: 0 1 4 5
	//Back Plane: 2 3 6 7
	//Top Plane: 4 5 6 7
	//Bottom Plane: 0 1 2 3

	vertices[0].x = boundingBox.position.x - boundingBox.halfExtentX
	vertices[0].y = boundingBox.position.y + boundingBox.halfExtentY
	vertices[0].z = boundingBox.position.z - boundingBox.halfExtentZ

	vertices[1].x = boundingBox.position.x + boundingBox.halfExtentX
	vertices[1].y = boundingBox.position.y + boundingBox.halfExtentY
	vertices[1].z = boundingBox.position.z - boundingBox.halfExtentZ

	vertices[2].x = boundingBox.position.x - boundingBox.halfExtentX
	vertices[2].y = boundingBox.position.y + boundingBox.halfExtentY
	vertices[2].z = boundingBox.position.z + boundingBox.halfExtentZ

	vertices[3].x = boundingBox.position.x + boundingBox.halfExtentX
	vertices[3].y = boundingBox.position.y + boundingBox.halfExtentY
	vertices[3].z = boundingBox.position.z + boundingBox.halfExtentZ

	vertices[4].x = boundingBox.position.x - boundingBox.halfExtentX
	vertices[4].y = boundingBox.position.y - boundingBox.halfExtentY
	vertices[4].z = boundingBox.position.z - boundingBox.halfExtentZ

	vertices[5].x = boundingBox.position.x + boundingBox.halfExtentX
	vertices[5].y = boundingBox.position.y - boundingBox.halfExtentY
	vertices[5].z = boundingBox.position.z - boundingBox.halfExtentZ

	vertices[6].x = boundingBox.position.x - boundingBox.halfExtentX
	vertices[6].y = boundingBox.position.y - boundingBox.halfExtentY
	vertices[6].z = boundingBox.position.z + boundingBox.halfExtentZ

	vertices[7].x = boundingBox.position.x + boundingBox.halfExtentX
	vertices[7].y = boundingBox.position.y - boundingBox.halfExtentY
	vertices[7].z = boundingBox.position.z + boundingBox.halfExtentZ

	return
}

func (vertex *Vertex4D) convertToScreenSpace() {
	vertex.x = (((vertex.x + 1) * float32(width+1)) / 2)
	vertex.y = (((-vertex.y + 1) * float32(height+1)) / 2)
}

func (v1 *Vertex4D) crossProduct(v2 *Vertex4D) float32 {
	return (v1.x * v2.y) - (v1.y * v2.x)
}

func (v1 *Vertex4D) cross(v2 *Vertex4D) Vertex4D {
	return Vertex4D{
		v1.y*v2.z - v1.z*v2.y,
		v1.z*v2.x - v1.x*v2.z,
		v1.x*v2.y - v1.y*v2.x,
		0,
	}
}

func (v1 *Vertex4D) subtract(v2 *Vertex4D) Vertex4D {
	return Vertex4D{
		v1.x - v2.x,
		v1.y - v2.y,
		v1.z - v2.z,
		v1.w - v2.w,
	}
}

func (v1 Vertex4D) multiply(v2 *Vertex4D) Vertex4D {
	return Vertex4D{
		v1.x * v2.x,
		v1.y * v2.y,
		v1.z * v2.z,
		v1.w * v2.w,
	}
}

func (v1 Vertex4D) add(v2 *Vertex4D) Vertex4D {
	return Vertex4D{
		v1.x + v2.x,
		v1.y + v2.y,
		v1.z + v2.z,
		v1.w + v2.w,
	}
}

func (v1 Vertex4D) dot(v2 *Vertex4D) float32 {
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
}

func (vertex *Vertex3D) convertToMatrix() Matrix {
	return Matrix{{vertex.x, vertex.y, vertex.z, 1}}
}

func (vertex *Vertex4D) convertToMatrix() Matrix {
	return Matrix{{vertex.x, vertex.y, vertex.z, vertex.w}}
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

func (matrix *Matrix) convertToVertex() Vertex4D {
	return Vertex4D{(*matrix)[0][0], (*matrix)[0][1], (*matrix)[0][2], (*matrix)[0][3]}
}

func (matrix *Matrix) convertToVertex3() Vertex3D {
	return Vertex3D{(*matrix)[0][0], (*matrix)[0][1], (*matrix)[0][2]}
}

func (triangle Triangle) multiplyMatrix(m2 *Matrix, normal bool) (result ComputedTriangle) {
	var tm1, tm2, tm3 Matrix = triangle.vertices[0].convertToMatrix(), triangle.vertices[1].convertToMatrix(), triangle.vertices[2].convertToMatrix()
	tm1, tm2, tm3 = tm1.multiplyMatrix(m2), tm2.multiplyMatrix(m2), tm3.multiplyMatrix(m2)

	var determinant float32 = (*m2)[0][0]*((*m2)[1][1]*(*m2)[2][2]-(*m2)[2][1]*(*m2)[1][2]) -
		(*m2)[0][1]*((*m2)[1][0]*(*m2)[2][2]-(*m2)[1][2]*(*m2)[2][0]) +
		(*m2)[0][2]*((*m2)[1][0]*(*m2)[2][1]-(*m2)[1][1]*(*m2)[2][0])

	var invDeterminant float32 = 1 / determinant

	var inverseMatrix Matrix = make(Matrix, len(*m2))

	for i := 0; i < len(*m2); i++ {
		inverseMatrix[i] = make([]float32, len((*m2)[0]))
		for j := 0; j < len((*m2)[0]); j++ {
			inverseMatrix[i][j] = (*m2)[i][j]
		}
	}

	inverseMatrix[0][0] = ((*m2)[1][1]*(*m2)[2][2] - (*m2)[2][1]*(*m2)[1][2]) * invDeterminant
	inverseMatrix[0][1] = ((*m2)[0][2]*(*m2)[2][1] - (*m2)[0][1]*(*m2)[2][2]) * invDeterminant
	inverseMatrix[0][2] = ((*m2)[0][1]*(*m2)[1][2] - (*m2)[0][2]*(*m2)[1][1]) * invDeterminant
	inverseMatrix[1][0] = ((*m2)[1][2]*(*m2)[2][0] - (*m2)[1][0]*(*m2)[2][2]) * invDeterminant
	inverseMatrix[1][1] = ((*m2)[0][0]*(*m2)[2][2] - (*m2)[0][2]*(*m2)[2][0]) * invDeterminant
	inverseMatrix[1][2] = ((*m2)[1][0]*(*m2)[0][2] - (*m2)[0][0]*(*m2)[1][2]) * invDeterminant
	inverseMatrix[2][0] = ((*m2)[1][0]*(*m2)[2][1] - (*m2)[2][0]*(*m2)[1][1]) * invDeterminant
	inverseMatrix[2][1] = ((*m2)[2][0]*(*m2)[0][1] - (*m2)[0][0]*(*m2)[2][1]) * invDeterminant
	inverseMatrix[2][2] = ((*m2)[0][0]*(*m2)[1][1] - (*m2)[1][0]*(*m2)[0][1]) * invDeterminant

	//inverseMatrix := Matrix{
	//	{(*m2)[0][0], (*m2)[1][0], (*m2)[2][0], (*m2)[0][3]},
	//	{(*m2)[0][1], (*m2)[1][1], (*m2)[2][1], (*m2)[1][3]},
	//	{(*m2)[0][2], (*m2)[1][2], (*m2)[2][2], (*m2)[2][3]},
	//	{(*m2)[3][0], (*m2)[3][1], (*m2)[3][2], (*m2)[3][3]},
	//}

	//inverseMatrix := *m2

	transposeMatrix := make(Matrix, len(inverseMatrix))

	for i := range inverseMatrix {
		transposeMatrix[i] = make([]float32, len(inverseMatrix[i]))
		for j := range inverseMatrix[i] {
			transposeMatrix[i][j] = inverseMatrix[j][i]
		}
	}

	var nm1, nm2, nm3 Matrix = triangle.normals[0].convertToMatrix(), triangle.normals[1].convertToMatrix(), triangle.normals[2].convertToMatrix()
	nm1, nm2, nm3 = nm1.multiplyMatrix(&transposeMatrix), nm2.multiplyMatrix(&transposeMatrix), nm3.multiplyMatrix(&transposeMatrix)

	result.vertices[0] = tm1.convertToVertex()
	result.vertices[1] = tm2.convertToVertex()
	result.vertices[2] = tm3.convertToVertex()

	result.normals[0] = nm1.convertToVertex3().normalize()
	result.normals[1] = nm2.convertToVertex3().normalize()
	result.normals[2] = nm3.convertToVertex3().normalize()

	//result.normals[0] = result.normals[0].normalize()

	result.uv = triangle.uv

	if !normal {
		result.normals = triangle.normals
	}

	return
}

func createProjectionMatrix(fov, aspect, near, far float32) Matrix {
	var tangent float32 = math32.Tan((fov * (math.Pi / 180)) / 2)

	return Matrix{
		{1 / (tangent * aspect), 0, 0, 0},
		{0, 1 / tangent, 0, 0},
		{0, 0, (far + near) / (near - far), (near * far * 2) / (near - far)},
		{0, 0, -1, 0},
	}
}

func createTransformationMatrix(position Vertex3D, r Vertex4D) Matrix {
	return Matrix{
		{1 - 2*r.y*r.y - 2*r.z*r.z, 2*r.x*r.y + 2*r.z*r.w, 2*r.x*r.z - 2*r.y*r.w, 0},
		{2*r.x*r.y - 2*r.z*r.w, 1 - 2*r.x*r.x - 2*r.z*r.z, 2*r.y*r.z + 2*r.w*r.x, 0},
		{2*r.x*r.z + 2*r.y*r.w, 2*r.y*r.z - 2*r.w*r.x, 1 - 2*r.x*r.x - 2*r.y*r.y, 0},
		{position.x, position.y, position.z, 1},
	}
}

func (triangle *ComputedTriangle) spanningVectors() (vs1, vs2 Vertex4D) {
	vs1 = Vertex4D{triangle.vertices[1].x - triangle.vertices[0].x, triangle.vertices[1].y - triangle.vertices[0].y, 0, 1}
	vs2 = Vertex4D{triangle.vertices[2].x - triangle.vertices[0].x, triangle.vertices[2].y - triangle.vertices[0].y, 0, 1}

	return
}

func (triangle *ComputedTriangle) bounds() (minX, minY, maxX, maxY int) {
	maxX = clamp(int(math32.Max(triangle.vertices[0].x, math32.Max(triangle.vertices[1].x, triangle.vertices[2].x))), 1, width+1)
	maxY = clamp(int(math32.Max(triangle.vertices[0].y, math32.Max(triangle.vertices[1].y, triangle.vertices[2].y))), 1, height+1)

	minX = clamp(int(math32.Min(triangle.vertices[0].x, math32.Min(triangle.vertices[1].x, triangle.vertices[2].x))), 1, width+1)
	minY = clamp(int(math32.Min(triangle.vertices[0].y, math32.Min(triangle.vertices[1].y, triangle.vertices[2].y))), 1, height+1)

	return
}

func (triangle *ComputedTriangle) barycentricCoordinates(vs1, vs2 *Vertex4D, x, y *int, span *float32) (s, t, w float32) {
	var q Vertex4D = Vertex4D{float32(*x) - triangle.vertices[0].x, float32(*y) - triangle.vertices[0].y, 0, 1}

	s = q.crossProduct(vs2) * (1.0 / *span)
	t = vs1.crossProduct(&q) * (1.0 / *span)

	w = 1 - s - t

	return
}

func (v *Vertex4D) convertToNormalized() {
	v.x /= v.w
	v.y /= v.w
	v.z /= v.w
}

func (v Vertex4D) normalize() Vertex4D {
	var magnitude float32 = math32.Sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z))

	return Vertex4D{v.x / magnitude, v.y / magnitude, v.z / magnitude, 0}
}

func (v Vertex3D) normalize() Vertex3D {
	var magnitude float32 = math32.Sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z))

	return Vertex3D{v.x / magnitude, v.y / magnitude, v.z / magnitude}
}

func (v1 *Vertex4D) interpolate(v2 *Vertex4D, factor float32) Vertex4D {
	return Vertex4D{
		v1.x*(1-factor) + v2.x*factor,
		v1.y*(1-factor) + v2.y*factor,
		v1.z*(1-factor) + v2.z*factor,
		v1.w*(1-factor) + v2.w*factor,
	}
}

func (v1 *Vertex3D) interpolate(v2 *Vertex3D, factor float32) Vertex3D {
	return Vertex3D{
		v1.x*(1-factor) + v2.x*factor,
		v1.y*(1-factor) + v2.y*factor,
		v1.z*(1-factor) + v2.z*factor,
	}
}

func (v1 *Vertex2D) interpolate(v2 *Vertex2D, factor float32) Vertex2D {
	return Vertex2D{
		v1.x*(1-factor) + v2.x*factor,
		v1.y*(1-factor) + v2.y*factor,
	}
}

func clamp(value, min, max int) int {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

func clampF(value, min, max float32) float32 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

func (buffer *Buffer) clearScreen() {
	for i := 0; i < (HT*VT)-1; i++ {
		wg.Add(1)

		go func(section int) {
			for p := section * chunkSize; p < section*chunkSize+chunkSize; p++ {
				(*buffer)[p] = 0
			}

			wg.Done()
		}(i)
	}

	for p := ((HT * VT) - 1) * chunkSize; p < ((HT*VT)-1)*chunkSize+chunkSize; p++ {
		(*buffer)[p] = 0
	}

	wg.Wait()
}

func (triangle *ComputedTriangle) edgeSpan(x, y *int) (w0, w1, w2 float32) {
	w0 = (triangle.vertices[2].y-triangle.vertices[1].y)*(float32(*x)-triangle.vertices[1].x) - (triangle.vertices[2].x-triangle.vertices[1].x)*(float32(*y)-triangle.vertices[1].y)
	w1 = (triangle.vertices[0].y-triangle.vertices[2].y)*(float32(*x)-triangle.vertices[2].x) - (triangle.vertices[0].x-triangle.vertices[2].x)*(float32(*y)-triangle.vertices[2].y)
	w2 = (triangle.vertices[1].y-triangle.vertices[0].y)*(float32(*x)-triangle.vertices[0].x) - (triangle.vertices[1].x-triangle.vertices[0].x)*(float32(*y)-triangle.vertices[0].y)

	return
}

func (buffer *FloatBuffer) clearDepth() {
	for i := 0; i < (HT*VT)-1; i++ {
		wg.Add(1)

		go func(section int) {
			for p := section * chunkSizeDepth; p < section*chunkSizeDepth+chunkSizeDepth; p++ {
				(*buffer)[p] = math32.MaxFloat32
			}

			wg.Done()
		}(i)
	}

	for i := ((HT * VT) - 1) * chunkSizeDepth; i < (HT*VT)*chunkSizeDepth; i++ {
		(*buffer)[i] = math32.MaxFloat32
	}
}

func (triangle *ComputedTriangle) renderToScreen(buffer *Buffer, depthBuffer *FloatBuffer, texture *Buffer, image *ebiten.Image, xPos, yPos int, tileGrid *TileGrid) {
	var copiedVertex [3]Vertex4D = triangle.vertices

	sort.Slice(copiedVertex[:], func(i, j int) bool {
		return copiedVertex[j].y > copiedVertex[i].y
	})

	var ti [3]Vertex2Di = [3]Vertex2Di{
		{int(copiedVertex[0].x), int(copiedVertex[0].y)},
		{int(copiedVertex[1].x), int(copiedVertex[1].y)},
		{int(copiedVertex[2].x), int(copiedVertex[2].y)},
	}

	var segHeightTrue int = ti[2].y - ti[1].y
	var segHeightFalse int = ti[1].y - ti[0].y

	for k := clamp(ti[0].y, yPos*tileSizeY, yPos*tileSizeY+tileSizeY); k < clamp(ti[2].y, yPos*tileSizeY, yPos*tileSizeY+tileSizeY); k++ {
		var i int = k - ti[0].y

		secondHalf := i > ti[1].y-ti[0].y || ti[1].y == ti[0].y
		var segmentHeight, betaHalf int

		if secondHalf {
			segmentHeight = segHeightTrue
			betaHalf = ti[1].y - ti[0].y
		} else {
			segmentHeight = segHeightFalse
		}

		var alpha float32 = float32(i) / float32(ti[2].y-ti[0].y)
		var beta float32 = float32(i-betaHalf) / float32(segmentHeight)

		var a Vertex2Di = Vertex2Di{
			ti[0].x + int(float32(ti[2].x-ti[0].x)*alpha),
			ti[0].y + int(float32(ti[2].y-ti[0].y)*alpha),
		}

		var b Vertex2Di

		if secondHalf {
			b = Vertex2Di{
				ti[1].x + int(float32(ti[2].x-ti[1].x)*beta),
				ti[1].y + int(float32(ti[2].y-ti[1].y)*beta),
			}
		} else {
			b = Vertex2Di{
				ti[0].x + int(float32(ti[1].x-ti[0].x)*beta),
				ti[0].y + int(float32(ti[1].y-ti[0].y)*beta),
			}
		}

		if a.x > b.x {
			var temp Vertex2Di = a
			a = b
			b = temp
		}

		for j := clamp(a.x, xPos*tileSizeX, xPos*tileSizeX+tileSizeX); j < clamp(b.x, xPos*tileSizeX, xPos*tileSizeX+tileSizeX); j++ {
			var s, t, w float32 = triangle.barycentricCoordinates(&triangle.vs1, &triangle.vs2, &j, &k, &triangle.span)
			var depth float32 = w*triangle.vertices[0].z + s*triangle.vertices[1].z + t*triangle.vertices[2].z

			var position int = k*width + j

			if depth < (*depthBuffer)[position] {
				var location int = position * 4

				var wt float32 = 1 / (w*triangle.at.z + s*triangle.bt.z + t*triangle.ct.z)

				var uvX float32 = (w*triangle.at.x + s*triangle.bt.x + t*triangle.ct.x) * wt
				var uvY float32 = (w*triangle.at.y + s*triangle.bt.y + t*triangle.ct.y) * wt

				var tx int = int(uvX * float32(image.Bounds().Dx()))
				var ty int = int((1 - uvY) * float32(image.Bounds().Dy()))

				var colorLocation int = ((ty*image.Bounds().Dx() + tx) * 4) % (image.Bounds().Dx() * image.Bounds().Dy() * 4)

				if colorLocation < 0 {
					colorLocation = -colorLocation
				}

				(*depthBuffer)[position] = depth

				//var fn float32 = w*triangle.fa.z + s*triangle.fb.z + t*triangle.fc.z
				var fragPos Vertex3D = Vertex3D{
					(w*triangle.fa.x + s*triangle.fb.x + t*triangle.fc.x) * wt,
					(w*triangle.fa.y + s*triangle.fb.y + t*triangle.fc.y) * wt,
					(w*triangle.fa.z + s*triangle.fb.z + t*triangle.fc.z) * wt,
				}

				var lightDir Vertex4D = Vertex4D{fragPos.x, fragPos.y, 4 - fragPos.z, 0}
				lightDir = lightDir.normalize()

				//var wn float32 = w*triangle.na.z + s*triangle.nb.z + t*triangle.nc.z
				var interpolatedNormal Vertex4D = Vertex4D{
					(w*triangle.na.x + s*triangle.nb.x + t*triangle.nc.x) * wt,
					(w*triangle.na.y + s*triangle.nb.y + t*triangle.nc.y) * wt,
					(w*triangle.na.z + s*triangle.nb.z + t*triangle.nc.z) * wt,
					0,
				}

				var negatedLightDir Vertex4D = Vertex4D{
					-lightDir.x,
					-lightDir.y,
					-lightDir.z,
					0,
				}

				interpolatedNormal = interpolatedNormal.normalize()
				var lightIntensity float32 = clampF(interpolatedNormal.dot(&lightDir), 0, math32.Inf(1))

				var negLightIntensity float32 = interpolatedNormal.dot(&negatedLightDir)

				var viewDir = Vertex4D{
					cameraPosition.x - fragPos.x,
					cameraPosition.y - fragPos.y,
					cameraPosition.z - fragPos.z, 0,
				}

				viewDir = viewDir.normalize()

				var reflectDir = Vertex4D{
					-lightDir.x - 2*negLightIntensity*interpolatedNormal.x,
					-lightDir.y - 2*negLightIntensity*interpolatedNormal.y,
					-lightDir.z - 2*negLightIntensity*interpolatedNormal.z,
					0,
				}

				var specular float32 = math32.Pow(clampF(viewDir.dot(&reflectDir), math32.Inf(-1), 0), 64) * 8
				var color [3]float32 = [3]float32{float32((*texture)[colorLocation]) / 255, float32((*texture)[colorLocation+1]) / 255, float32((*texture)[colorLocation+2]) / 255}

				(*buffer)[location] = uint8(clamp(int(color[0]*(.125+lightIntensity+specular)*255), 0, 255))
				(*buffer)[location+1] = uint8(clamp(int(color[1]*(.125+lightIntensity+specular)*255), 0, 255))
				(*buffer)[location+2] = uint8(clamp(int(color[2]*(.125+lightIntensity+specular)*255), 0, 255))
			}
		}
	}

}

func clip_axis(vertices *[]Vertex4D, uv *[]Vertex2D, normals *[]Vertex3D, worldVert *[]Vertex4D, factor float32, axis int) {
	var data []Vertex4D
	var uvData []Vertex2D
	var normalData []Vertex3D
	var worldVertData []Vertex4D

	var previousVertex *Vertex4D = &(*vertices)[len(*vertices)-1]
	var previousUV *Vertex2D = &(*uv)[len(*uv)-1]
	var previousNormal *Vertex3D = &(*normals)[len(*normals)-1]
	var previousWorldVert *Vertex4D = &(*worldVert)[len(*worldVert)-1]

	var previousComponent float32 = factor
	var previousInside bool

	switch axis {
	case 0:
		previousComponent *= previousVertex.x
	case 1:
		previousComponent *= previousVertex.y
	case 2:
		previousComponent *= previousVertex.z
	}

	if previousComponent <= previousVertex.w {
		previousInside = true
	}

	for v := 0; v < len(*vertices); v++ {
		var currentVertex *Vertex4D = &(*vertices)[v]
		var currentUV *Vertex2D = &(*uv)[v]
		var currentNormal *Vertex3D = &(*normals)[v]
		var currentWorldVert *Vertex4D = &(*worldVert)[v]

		var currentComponent float32 = factor
		var currentInside bool

		switch axis {
		case 0:
			currentComponent *= currentVertex.x
		case 1:
			currentComponent *= currentVertex.y
		case 2:
			currentComponent *= currentVertex.z
		}
		if currentComponent <= currentVertex.w {
			currentInside = true
		}

		if currentInside != previousInside {
			var factor float32 = (previousVertex.w - previousComponent) / ((previousVertex.w - previousComponent) - (currentVertex.w - currentComponent))

			data = append(data, previousVertex.interpolate(currentVertex, factor))
			uvData = append(uvData, previousUV.interpolate(currentUV, factor))
			normalData = append(normalData, previousNormal.interpolate(currentNormal, factor))
			worldVertData = append(worldVertData, previousWorldVert.interpolate(currentWorldVert, factor))
		}

		if currentInside {
			data = append(data, *currentVertex)
			uvData = append(uvData, *currentUV)
			normalData = append(normalData, *currentNormal)
			worldVertData = append(worldVertData, *currentWorldVert)
		}

		previousVertex = currentVertex
		previousComponent = currentComponent
		previousInside = currentInside
		previousUV = currentUV
		previousNormal = currentNormal
		previousWorldVert = currentWorldVert
	}

	*vertices = data
	*uv = uvData
	*normals = normalData
	*worldVert = worldVertData
}

func (v *Vertex3D) convertToQuaternion() Vertex4D {
	var rollRad float32 = v.x * (math.Pi / 180)
	var pitchRad float32 = v.y * (math.Pi / 180)
	var yawRad float32 = v.z * (math.Pi / 180)

	rollHalf := rollRad * .5
	pitchHalf := pitchRad * .5
	yawHalf := yawRad * .5

	cosRollHalf := math32.Cos(rollHalf)
	sinRollHalf := math32.Sin(rollHalf)
	cosPitchHalf := math32.Cos(pitchHalf)
	sinPitchHalf := math32.Sin(pitchHalf)
	cosYawHalf := math32.Cos(yawHalf)
	sinYawHalf := math32.Sin(yawHalf)

	// The order of quaternion components is different in this convention (XYZ order).
	x := sinRollHalf*cosPitchHalf*cosYawHalf - cosRollHalf*sinPitchHalf*sinYawHalf
	y := cosRollHalf*sinPitchHalf*cosYawHalf + sinRollHalf*cosPitchHalf*sinYawHalf
	z := cosRollHalf*cosPitchHalf*sinYawHalf - sinRollHalf*sinPitchHalf*cosYawHalf
	w := cosRollHalf*cosPitchHalf*cosYawHalf + sinRollHalf*sinPitchHalf*sinYawHalf

	return Vertex4D{x, y, z, w}
}

func (t *ComputedTriangle) clip(tileGrid *TileGrid, triData *ComputedTriangle) {
	var vertices []Vertex4D = t.vertices[:]

	var uvdat []Vertex2D = t.uv[:]
	var normaldat []Vertex3D = triData.normals[:]
	var worldVert []Vertex4D = triData.vertices[:]

	for i := 0; i < 2; i++ {
		if len(vertices) > 0 {
			clip_axis(&vertices, &uvdat, &normaldat, &worldVert, 1, i)

			if len(vertices) > 0 {
				clip_axis(&vertices, &uvdat, &normaldat, &worldVert, -1, i)
			} else {
				break
			}
		} else {
			break
		}
	}

	//We Pre-Convert The Vertices Here To Avoid Doing The Same Calculations Twice On Every Triangle
	//for i := 0; i < len(vertices); i++ {
	//
	//	}

	//Finally We Build All The Triangles

	if len(vertices) > 0 {
		vertices[0].convertToNormalized()
		vertices[0].convertToScreenSpace()

		vertices[1].convertToNormalized()
		vertices[1].convertToScreenSpace()

		inverseIndex := 1 / vertices[0].w
		inverseIndex1 := 1 / vertices[1].w

		t1 := vertices[1].subtract((&vertices[0]))

		for index := 0; index < len(vertices)-2; index++ {
			vertices[index+2].convertToNormalized()
			vertices[index+2].convertToScreenSpace()

			t2 := vertices[index+2].subtract(&vertices[0])

			crossed := t1.cross(&t2)

			if crossed.z < 0 {
				inverseIndex2 := 1 / vertices[index+2].w

				var newTriangle ComputedTriangle = ComputedTriangle{
					[3]Vertex4D{vertices[0], vertices[index+1], vertices[index+2]},
					[3]Vertex2D{uvdat[0], uvdat[index+1], uvdat[index+2]},
					[3]Vertex3D{normaldat[0], normaldat[index+1], normaldat[index+2]},

					Vertex3D{uvdat[0].x * inverseIndex, uvdat[0].y * inverseIndex, inverseIndex},
					Vertex3D{uvdat[index+1].x * inverseIndex1, uvdat[index+1].y * inverseIndex1, inverseIndex1},
					Vertex3D{uvdat[index+2].x * inverseIndex2, uvdat[index+2].y * inverseIndex2, inverseIndex2},

					Vertex4D{}, Vertex4D{}, 0,

					0, 0, 0, 0,
					Vertex3D{normaldat[0].x * inverseIndex, normaldat[0].y * inverseIndex, normaldat[0].z * inverseIndex},
					Vertex3D{normaldat[index+1].x * inverseIndex1, normaldat[index+1].y * inverseIndex1, normaldat[index+1].z * inverseIndex1},
					Vertex3D{normaldat[index+2].x * inverseIndex2, normaldat[index+2].y * inverseIndex2, normaldat[index+2].z * inverseIndex2},

					Vertex3D{worldVert[0].x * inverseIndex, worldVert[0].y * inverseIndex, worldVert[0].z * inverseIndex},
					Vertex3D{worldVert[index+1].x * inverseIndex1, worldVert[index+1].y * inverseIndex1, worldVert[index+1].z * inverseIndex1},
					Vertex3D{worldVert[index+2].x * inverseIndex2, worldVert[index+2].y * inverseIndex2, worldVert[index+2].z * inverseIndex2},
				}

				newTriangle.minX, newTriangle.minY, newTriangle.maxX, newTriangle.maxY = newTriangle.bounds()

				var tilePositionMaxX int = clamp(int(math32.Round(float32(newTriangle.maxX/tileSizeX))), 0, len(tileGrid)-1)
				var tilePositionMaxY int = clamp(int(math32.Round(float32(newTriangle.maxY/tileSizeY))), 0, len(tileGrid[0])-1)

				var tilePositionMinX int = clamp(int(math32.Round(float32(newTriangle.minX/tileSizeX))), 0, len(tileGrid)-1)
				var tilePositionMinY int = clamp(int(math32.Round(float32(newTriangle.minY/tileSizeY))), 0, len(tileGrid[0])-1)

				newTriangle.vs1, newTriangle.vs2 = newTriangle.spanningVectors()
				newTriangle.span = newTriangle.vs1.crossProduct(&newTriangle.vs2)

				inverseIndex1 = inverseIndex2
				t1 = t2

				for x := tilePositionMinX; x <= tilePositionMaxX; x++ {
					for y := tilePositionMinY; y <= tilePositionMaxY; y++ {
						mu.Lock()
						tileGrid[x][y] = append(tileGrid[x][y], newTriangle)
						mu.Unlock()
					}
				}

			}
		}
	}

}

type Game struct{}

func (g *Game) Update() error {
	var speed float32 = 2

	if ebiten.IsKeyPressed(ebiten.KeyRight) {
		cameraRotation.y += speed
	}

	if ebiten.IsKeyPressed(ebiten.KeyLeft) {
		cameraRotation.y -= speed
	}

	if ebiten.IsKeyPressed(ebiten.KeyUp) {
		cameraRotation.x -= speed
	}

	if ebiten.IsKeyPressed(ebiten.KeyDown) {
		cameraRotation.x += speed
	}

	if ebiten.IsKeyPressed(ebiten.KeyW) {
		cameraPosition.x += math32.Cos((cameraRotation.y+90)*(math.Pi/180)) * math32.Cos(cameraRotation.x*(math.Pi/180))
		cameraPosition.z += math32.Sin((cameraRotation.y+90)*(math.Pi/180)) * math32.Cos(cameraRotation.x*(math.Pi/180))
		cameraPosition.y += math32.Sin((cameraRotation.x) * (math.Pi / 180))
	}

	if ebiten.IsKeyPressed(ebiten.KeyS) {
		cameraPosition.x -= math32.Cos((cameraRotation.y+90)*(math.Pi/180)) * math32.Cos(cameraRotation.x*(math.Pi/180))
		cameraPosition.z -= math32.Sin((cameraRotation.y+90)*(math.Pi/180)) * math32.Cos(cameraRotation.x*(math.Pi/180))
		cameraPosition.y -= math32.Sin((cameraRotation.x) * (math.Pi / 180))
	}

	if ebiten.IsKeyPressed(ebiten.KeyD) {
		cameraPosition.x -= math32.Cos(cameraRotation.y * (math.Pi / 180))
		cameraPosition.z -= math32.Sin(cameraRotation.y * (math.Pi / 180))
	}

	if ebiten.IsKeyPressed(ebiten.KeyA) {
		cameraPosition.x += math32.Cos(cameraRotation.y * (math.Pi / 180))
		cameraPosition.z += math32.Sin(cameraRotation.y * (math.Pi / 180))
	}

	if inpututil.IsKeyJustPressed(ebiten.KeyQ) {
		if currentModelIndex > 0 {
			currentModelIndex--
		}
	}

	if inpututil.IsKeyJustPressed(ebiten.KeyE) {
		if currentModelIndex < len(modelList)-1 {
			currentModelIndex++
		}
	}

	return nil
}

var rotationDegrees Vertex3D

func (m *Model) processModel(transformationMatrix *Matrix, buffer *[]Triangle) {
	var amountOfLeftSide, amountOfRightSide, amountOfTopSide, amountOfBottomSide int = 0, 0, 0, 0

	for i := 0; i < 8; i++ {
		var transformed Matrix = m.BoundingBox[i].convertToMatrix()
		transformed = transformed.multiplyMatrix(transformationMatrix)

		var vertex Vertex4D = transformed.convertToVertex()

		if vertex.isInClipSpaceX(false) {
			amountOfLeftSide++
		}

		if vertex.isInClipSpaceX(true) {
			amountOfRightSide++
		}

		if vertex.isInClipSpaceY(false) {
			amountOfTopSide++
		}

		if vertex.isInClipSpaceY(true) {
			amountOfBottomSide++
		}
	}

	if amountOfLeftSide != 0 && amountOfRightSide != 0 && amountOfTopSide != 0 && amountOfBottomSide != 0 {
		*buffer = append(*buffer, m.TriangleData...)
	}
}

var rotation Vertex3D

func (g *Game) Draw(screen *ebiten.Image) {
	wg.Wait()

	if cobble_buffer == nil {
		cobble_buffer = make(Buffer, cobble.Bounds().Dx()*cobble.Bounds().Dy()*4)
		cobble.ReadPixels(cobble_buffer)
	}

	//smallRotation.z += 1
	rotationDegrees.y += 1
	//rotationDegrees.x += 1

	rotation.y = 0

	var cameraRotationX Vertex3D = Vertex3D{cameraRotation.x, 0, 0}
	var cameraRotationY Vertex3D = Vertex3D{0, cameraRotation.y, 0}
	var cameraRotationZ Vertex3D = Vertex3D{0, 0, cameraRotation.z}

	var tileGrid TileGrid
	var rotationMatrixX = createTransformationMatrix(Vertex3D{0, 0, 0}, cameraRotationX.convertToQuaternion())
	var rotationMatrixY = createTransformationMatrix(Vertex3D{0, 0, 0}, cameraRotationY.convertToQuaternion())
	var rotationMatrixZ = createTransformationMatrix(Vertex3D{0, 0, 0}, cameraRotationZ.convertToQuaternion())

	var emptyRotation Vertex3D

	var cameraMatrix Matrix = createTransformationMatrix(cameraPosition, emptyRotation.convertToQuaternion())
	var transformationMatrix Matrix = createTransformationMatrix(emptyRotation, rotationDegrees.convertToQuaternion())
	transformationMatrix = transformationMatrix.multiplyMatrix(&cameraMatrix)
	transformationMatrix = transformationMatrix.multiplyMatrix(&rotationMatrixY)
	transformationMatrix = transformationMatrix.multiplyMatrix(&rotationMatrixX)
	transformationMatrix = transformationMatrix.multiplyMatrix(&rotationMatrixZ)
	transformationMatrix = transformationMatrix.multiplyMatrix(&projectionMatrix)

	var triData []Triangle

	modelList[currentModelIndex].processModel(&transformationMatrix, &triData)

	var amount int = len(triData)
	var amountPerCore int = amount / (HT * VT)
	var amountLeft = amount % (HT * VT)

	for p := 0; p < (HT*VT)-1; p++ {
		wg.Add(1)

		go func(chunk int, model *Model, grid *TileGrid) {
			for t := chunk * amountPerCore; t < chunk*amountPerCore+amountPerCore; t++ {
				var r Matrix = createTransformationMatrix(emptyRotation, rotationDegrees.convertToQuaternion())
				convertedTriangle := triData[t].multiplyMatrix(&transformationMatrix, false)
				t := triData[t].multiplyMatrix(&r, true)

				convertedTriangle.clip(grid, &t)
			}

			wg.Done()
		}(p, &teapot, &tileGrid)
	}

	for t := ((HT * VT) - 1) * amountPerCore; t < (HT*VT)*amountPerCore+amountLeft; t++ {
		var r Matrix = createTransformationMatrix(emptyRotation, rotationDegrees.convertToQuaternion())
		convertedTriangle := triData[t].multiplyMatrix(&transformationMatrix, false)
		t := triData[t].multiplyMatrix(&r, true)

		convertedTriangle.clip(&tileGrid, &t)
	}

	wg.Wait()

	for x := 0; x < len(tileGrid); x++ {
		for y := 0; y < len(tileGrid[0]); y++ {
			if x == 0 && y == 0 {
				continue
			}

			wg.Add(1)

			go func(tileGrid *TileGrid, x, y int) {
				for _, t := range tileGrid[x][y] {
					t.renderToScreen(&screenBuffer, &depthBuffer, &cobble_buffer, cobble, x, y, tileGrid)
				}

				wg.Done()
			}(&tileGrid, x, y)
		}
	}

	for _, t := range tileGrid[0][0] {
		t.renderToScreen(&screenBuffer, &depthBuffer, &cobble_buffer, cobble, 0, 0, &tileGrid)
	}

	wg.Wait()

	screen.WritePixels(screenBuffer)
	ebitenutil.DebugPrint(screen, "FPS: "+strconv.Itoa(int(ebiten.ActualFPS())))
	ebitenutil.DebugPrintAt(screen, "TRIANGLES RASTERIZED: "+strconv.Itoa(amount), 0, 16)
	ebitenutil.DebugPrintAt(screen, "AMOUNT OF CORES: "+strconv.Itoa(runtime.NumCPU()), 0, 32)
	ebitenutil.DebugPrintAt(screen, "PRESS Q & E TO SWITCH MODELS", 0, 48)

	screenBuffer.clearScreen()
	depthBuffer.clearDepth()
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return width, height
}

func init() {
	var err error
	cobble, _, err = ebitenutil.NewImageFromFile("brick.png")

	//runtime.GOMAXPROCS(runtime.NumCPU())

	if err != nil {
		log.Fatal(err)
	}
}

var modelList []Model
var currentModelIndex int

func main() {
	fmt.Println("Initializing Polygon Core")

	ebiten.SetWindowSize(width, height)
	ebiten.SetWindowTitle("Polygon Core - V2")
	ebiten.SetVsyncEnabled(true)
	ebiten.SetTPS(ebiten.SyncWithFPS)
	ebiten.SetScreenClearedEveryFrame(false)
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)
	ebiten.SetFullscreen(false)

	cameraPosition.z -= 3

	screenBuffer = make(Buffer, width*height*4)
	chunkSize = (width * height * 4) / (HT * VT)

	fmt.Println("Screen Buffer Initialized")

	depthBuffer = make(FloatBuffer, width*height)
	chunkSizeDepth = (width * height) / (HT * VT)

	fmt.Println("Depth Buffer Initialized")

	projectionMatrix = createProjectionMatrix(fov, aspectRatio, .1, 100)
	fmt.Println("Projection Matrix Initialized")

	modelList = append(modelList, NewModel("Cube.obj"))
	modelList = append(modelList, NewModel("car.obj"))
	modelList = append(modelList, NewModel("bunny.obj"))
	modelList = append(modelList, NewModel("teapot.obj"))
	modelList = append(modelList, NewModel("Skull_HQ.obj"))
	modelList = append(modelList, NewModel("Monkey.obj"))

	fmt.Println("Triangle Data Initialized")

	if err := ebiten.RunGameWithOptions(&Game{}, &ebiten.RunGameOptions{GraphicsLibrary: ebiten.GraphicsLibraryMetal, InitUnfocused: false, ScreenTransparent: false, SkipTaskbar: false}); err != nil {
		log.Fatal(err)
	}
}
