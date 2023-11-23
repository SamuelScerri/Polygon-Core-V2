# What Is Ghetty?
The Ghetty Engine is a Software Renderer built with speed in mind and is designed to run in real-time, it utilizes Goroutines to speed up performance significantly and to allow other things such as game logic to run in the background. Since it utilizes multiple cores to rasterize a triangle, remember that with something such as a Dual-Core CPU or a Quad-Core CPU, you might not get that much of a performance increase.

# Any Experimental Features?
The future iterations of Polygon Core will utilize a Super Resolution Model to increase performance by rendering everything with the CPU using a low resolution, and then upscaling the screen buffer with the GPU. This will of course be optional and it is unsure whether this would even increase performance or not, but that's why it's called experimental!

# Libraries Used:
- Ebiten: For Rendering The Screen Buffer & Keyboard Input
- Math32: For Fast Single Precision Math (Golang Used Double Precision Math Functions In The Default Library)

# Current Features:
- Barycentric Triangle Rasterization
- Perspective Correct Texutre Mapping
- Depth Buffer Support
- Triangle Clipping
- Multicore Rendering
- Backface Culling
- Frustum Culling
- Translation & Rotation Support
- Wavefront OBJ Support
- Basic Diffuse Lighting

# Features That Will Be Added:
- Super Resolution Upscaling

# Features That MIGHT Be Added:
- SDL2 Version
- Occlusion Culling
- Normal Mapping
