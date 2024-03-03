package main

import (
	"fmt"
	"io"
	"os"
)

type Model []Triangle

func LoadModel(directory string) (model Model) {
	objFile, err := os.Open(directory)
	if err != nil {
		panic(err)
	}

	defer objFile.Close()

	var vertices, uvs, normals []Vertex
	var verticesIndices, uvsIndices, normalsIndices []int

	for {
		var lineType string

		_, err := fmt.Fscanf(objFile, "%s", &lineType)

		if err != nil {
			if err == io.EOF {
				break
			}
		}

		switch lineType {
		case "v":
			vec := Vertex{0, 0, 0, 1}
			fmt.Fscanf(objFile, "%f %f %f\n", &vec[X], &vec[Y], &vec[Z])
			vertices = append(vertices, vec)
		case "vn":
			vec := Vertex{0, 0, 0, 1}
			fmt.Fscanf(objFile, "%f %f %f\n", &vec[X], &vec[Y], &vec[Z])
			normals = append(normals, vec)
		case "vt":
			vec := Vertex{0, 0, 0, 1}
			fmt.Fscanf(objFile, "%f %f\n", &vec[X], &vec[Y])
			uvs = append(uvs, vec)
		case "f":
			norm := make([]int, 3)
			vec := make([]int, 3)
			uv := make([]int, 3)

			matches, _ := fmt.Fscanf(objFile, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vec[0], &uv[0], &norm[0], &vec[1], &uv[1], &norm[1], &vec[2], &uv[2], &norm[2])

			if matches != 9 {
				panic("Cannot read your file")
			}

			normalsIndices = append(normalsIndices, norm[0])
			normalsIndices = append(normalsIndices, norm[1])
			normalsIndices = append(normalsIndices, norm[2])

			verticesIndices = append(verticesIndices, vec[0])
			verticesIndices = append(verticesIndices, vec[1])
			verticesIndices = append(verticesIndices, vec[2])

			uvsIndices = append(uvsIndices, uv[0])
			uvsIndices = append(uvsIndices, uv[1])
			uvsIndices = append(uvsIndices, uv[2])
		}
	}

	for i := 0; i < len(verticesIndices)/3; i++ {
		var tri Triangle = Triangle{
			UV: [3]Vertex{uvs[uvsIndices[i*3]-1], uvs[uvsIndices[i*3+1]-1], uvs[uvsIndices[i*3+2]-1]},

			Vertices: [3]Vertex{vertices[verticesIndices[i*3]-1], vertices[verticesIndices[i*3+1]-1], vertices[verticesIndices[i*3+2]-1]},

			Normals: [3]Vertex{normals[normalsIndices[i*3]-1], normals[normalsIndices[i*3+1]-1], normals[normalsIndices[i*3+2]-1]},

			Texture: &Brick,

			Shader: BasicShader,
		}

		model = append(model, tri)
	}

	fmt.Println("Successfully Loaded:", directory)
	fmt.Println("Vertex Size:", len(model))
	fmt.Println("")

	return model
}
