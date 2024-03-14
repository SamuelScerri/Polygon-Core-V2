# What is Ghetty?
Ghetty is a Software Renderer designed to run in real-time with excellent performance, whilst also being quite efficient.
Multiple goroutines are utilised to process and rasterize triangles, allowing every core to be used in conjunction with one another, to deliver a better framerate!
Naturally since it is a Software Renderer, it is bound by certain limitations. It can't handle millions of triangles like a modern GPU, and will be less efficient than one.
Ensure that your PC has a proper cooling solution, since all of the CPU cores will be at 100% utilization.

# Current Features
- Triangle Rasterization (Duh!)
- Perspective Correct Texture Mapping
- Depth Buffer Support
- Triangle Clipping
- Multicore Rendering (Also Duh)
- Backface Culling
- Matrix Multiplication
- Wavefront OBJ Support
- Shaders! (Vertex & Fragment)

# Any Experimental Features?
The future iteration of Ghetty will feature a Super Resolution Model that will be integrated into the renderer. The model in this case will be handled by the GPU,
and will upscale the screen buffer up to 2x to 4x (maybe up to 8x) times the original resolution! Of course this will be optional, and shouldn't be available on hardware
that doesn't present any GPUs. Since Go, the language that is used to write this renderer doesn't have good support for using models such as ESRGAN, FSRCNN etc. A Python API
might be created which will just render a scene into a buffer, which can then be manipulated by Python.
