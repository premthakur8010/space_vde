// =============================================================================
// include/Renderer.hpp
// Declares the OpenGL 4.5 3D rendering subsystem: GLSL Shader wrapper, 4x MSAA
// off-screen Framebuffer, Wavefront OBJ Mesh loader, interactive Z-up
// OrbitCamera, immediate-mode 3D Painter, coordinate-axis triad helper, and the
// VdeScene and SpacecraftScene 3D visualizers.
// =============================================================================
#pragma once
#include "Client.hpp"

#include <glad/gl.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace vde::render {

// Owns a compiled and linked OpenGL shader program and provides typed uniform setters.
class Shader {
public:
	Shader() = default;
	~Shader();
	Shader(const Shader&) = delete;
	Shader& operator=(const Shader&) = delete;

	bool build(const char* vertexSrc, const char* fragmentSrc);
	void use() const { glUseProgram(m_program); }
	void set(const char* name, const glm::mat4& m) const;
	void set(const char* name, const glm::vec3& v) const;
	void set(const char* name, float f) const;

private:
	GLuint m_program = 0;
};

// Off-screen 4x multisampled render target resolved into a standard 2D RGBA8 texture
// so 3D scenes can be embedded inside Dear ImGui windows via ImGui::Image().
class Framebuffer {
public:
	Framebuffer() = default;
	~Framebuffer();
	Framebuffer(const Framebuffer&) = delete;
	Framebuffer& operator=(const Framebuffer&) = delete;

	void resize(int width, int height);
	void bind() const;
	void resolve() const;
	void readPixels(unsigned char* rgba) const;

	GLuint texture() const { return m_texture; }
	int    width() const   { return m_width; }
	int    height() const  { return m_height; }

private:
	void release();
	int    m_width = 0, m_height = 0;
	GLuint m_msaaFbo = 0, m_msaaColor = 0, m_msaaDepth = 0;
	GLuint m_resolveFbo = 0, m_texture = 0;
};

// Owns an OpenGL Vertex Array Object (VAO) and interleaved position+normal Vertex Buffer Object (VBO),
// loaded from Wavefront .obj files for unit geometry primitives.
class Mesh {
public:
	Mesh() = default;
	~Mesh();
	Mesh(const Mesh&) = delete;
	Mesh& operator=(const Mesh&) = delete;

	void upload(const std::vector<float>& posNormal);
	void draw() const;

	static bool loadObj(Mesh& m, const std::string& path);

private:
	GLuint  m_vao = 0, m_vbo = 0;
	GLsizei m_count = 0;
};

// Spherical Z-up orbit camera supporting mouse drag rotation, screen-space panning, and scroll-wheel zoom.
struct OrbitCamera {
	glm::vec3 target{0.0f};
	float yawDeg = -60.0f, pitchDeg = 20.0f, distance = 7.0f, fovDeg = 40.0f;

	glm::vec3 position() const;
	glm::mat4 view() const;
	glm::mat4 projection(float aspect) const;

	void rotate(float dxPixels, float dyPixels);
	void pan(float dxPixels, float dyPixels, float viewportHeight);
	void zoom(float wheelSteps);
};

// Distinguishes compact coordinate-frame tags from leader-line hardware/spacecraft callouts.
enum class LabelKind { Frame, Object };

// 3D world-space anchor and text queued during scene rendering for 2D projection overlay in SceneView.
struct Label {
	glm::vec3   world;
	std::string text;
	LabelKind   kind;
	float       side = 1.0f; // Horizontal leader-line direction: +1 right, -1 left
};

// Immediate-mode 3D shape renderer that shares reusable unit meshes and collects 3D annotation labels.
class Painter {
public:
	Painter();

	void begin(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cameraPos);
	void overlayPass();

	void box(const glm::mat4& model, const glm::vec3& color);
	void box(const glm::vec3& center, const glm::vec3& size, const glm::vec3& color,
			 const glm::mat4& parent = glm::mat4(1.0f));
	void cylinder(const glm::vec3& a, const glm::vec3& b, float radius, const glm::vec3& color);
	void cone(const glm::vec3& base, const glm::vec3& apex, float radius, const glm::vec3& color);
	void sphere(const glm::vec3& center, float radius, const glm::vec3& color, int detail = 0);

	void label(const glm::vec3& world, std::string text, LabelKind kind, float side = 1.0f);
	const std::vector<Label>& labels() const { return m_labels; }

private:
	void draw(const Mesh& mesh, const glm::mat4& model, const glm::vec3& color);
	static glm::mat4 alongSegment(const glm::vec3& a, const glm::vec3& b, float radius);

	Shader             m_shader;
	Mesh               m_box, m_cylinder, m_cone, m_sphere, m_sphereFine;
	std::vector<Label> m_labels;
};

// Draws an RGB XYZ coordinate-frame triad (Red = X, Green = Y, Blue = Z) and optional frame-origin label.
void drawAxisTriad(Painter& painter, const glm::mat4& frame, float length, const char* name);

// Abstract interface implemented by each 3D scene rendered inside a SceneView viewport.
class IScene {
public:
	virtual ~IScene() = default;
	virtual void render(Painter& painter, const SimSnapshot& snapshot) = 0;
	virtual glm::vec3 clearColor() const = 0;
	virtual OrbitCamera defaultCamera() const = 0;
};

// 3D visualization of the physical VDE testbed: base, 6-legged Stewart platform, overhead gantry,
// Chaser/Target docking mechanisms, and coordinate frames B_sp, P_sp, and O_sp.
class VdeScene : public IScene {
public:
	void render(Painter& painter, const SimSnapshot& snapshot) override;
	glm::vec3 clearColor() const override { return {0.93f, 0.94f, 0.95f}; }
	OrbitCamera defaultCamera() const override;
};

// 3D orbital visualization of the Chaser and Target spacecraft, solar arrays, docking adapters,
// Earth horizon, and body/docking coordinate frames B_C, D_C, D_T, and B_T.
class SpacecraftScene : public IScene {
public:
	void render(Painter& painter, const SimSnapshot& snapshot) override;
	glm::vec3 clearColor() const override { return {0.06f, 0.08f, 0.13f}; }
	OrbitCamera defaultCamera() const override;
};

} // namespace vde::render
