// =============================================================================
// src/Renderer.cpp
// Implements the OpenGL 4.5 3D rendering classes: two-light GLSL 450 shader,
// Direct State Access (DSA) 4x MSAA Framebuffer, procedural Mesh builders,
// OrbitCamera math, immediate-mode Painter, RGB coordinate triads, VdeScene
// (hexapod + overhead gantry), and SpacecraftScene (orbital Chaser & Target).
// =============================================================================
#include "Renderer.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace vde::render {

namespace {
constexpr GLsizei kSamples          = 4;
constexpr float   kPi               = std::numbers::pi_v<float>;
constexpr float   kRotateDegPerPx   = 0.3f;
constexpr float   kMaxPitchDeg      = 89.0f;
constexpr float   kZoomPerWheelStep = 0.88f;
constexpr float   kMinDistanceM     = 0.5f;
constexpr float   kMaxDistanceM     = 400.0f;

// Compiles a single GLSL shader stage and prints driver diagnostics on syntax/compile errors.
GLuint compileShader(GLenum type, const char* src) {
	GLuint s = glCreateShader(type);
	glShaderSource(s, 1, &src, nullptr);
	glCompileShader(s);
	GLint ok = 0;
	glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		char infoLog[1024];
		glGetShaderInfoLog(s, sizeof(infoLog), nullptr, infoLog);
		std::fprintf(stderr, "[SHADER] %s\n", infoLog);
	}
	return s;
}

// Appends one interleaved 6-float vertex (px, py, pz, nx, ny, nz) into a mesh buffer.
void pushVertex(std::vector<float>& v, glm::vec3 p, glm::vec3 n) {
	v.insert(v.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
}

// Vertex shader transforming local positions and normals into world space and clip space.
const char* kVertex = R"(#version 450 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_model;
uniform mat4 u_viewProjection;
out vec3 v_normal;
out vec3 v_world;
void main() {
	vec4 world = u_model * vec4(a_position, 1.0);
	v_world = world.xyz;
	v_normal = mat3(transpose(inverse(u_model))) * a_normal;
	gl_Position = u_viewProjection * world;
}
)";

// Fragment shader applying ambient + key + fill directional lighting with two-sided normal flipping.
const char* kFragment = R"(#version 450 core
in vec3 v_normal;
in vec3 v_world;
uniform vec3 u_color;
uniform vec3 u_cameraPos;
out vec4 fragColor;
void main() {
	vec3 n = normalize(v_normal);
	if (dot(n, u_cameraPos - v_world) < 0.0) n = -n;
	float key  = max(dot(n, normalize(vec3(0.4, -0.6, 0.8))), 0.0);
	float fill = max(dot(n, normalize(vec3(-0.6, 0.5, 0.3))), 0.0);
	vec3 c = u_color * (0.38 + 0.55 * key + 0.22 * fill);
	fragColor = vec4(min(c, vec3(1.0)), 1.0);
}
)";
} // namespace

// ============================================================================
// Shader, Framebuffer & Mesh
// ============================================================================

Shader::~Shader() { if (m_program) glDeleteProgram(m_program); }

bool Shader::build(const char* vertexSrc, const char* fragmentSrc) {
	GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
	GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
	m_program = glCreateProgram();
	glAttachShader(m_program, vs);
	glAttachShader(m_program, fs);
	glLinkProgram(m_program);
	glDeleteShader(vs);
	glDeleteShader(fs);
	GLint ok = 0;
	glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
	return ok != 0;
}

void Shader::set(const char* name, const glm::mat4& m) const {
	glProgramUniformMatrix4fv(m_program, glGetUniformLocation(m_program, name), 1, GL_FALSE, glm::value_ptr(m));
}
void Shader::set(const char* name, const glm::vec3& v) const {
	glProgramUniform3fv(m_program, glGetUniformLocation(m_program, name), 1, glm::value_ptr(v));
}
void Shader::set(const char* name, float f) const {
	glProgramUniform1f(m_program, glGetUniformLocation(m_program, name), f);
}

Framebuffer::~Framebuffer() { release(); }

void Framebuffer::release() {
	if (m_msaaFbo) glDeleteFramebuffers(1, &m_msaaFbo);
	if (m_resolveFbo) glDeleteFramebuffers(1, &m_resolveFbo);
	if (m_msaaColor) glDeleteRenderbuffers(1, &m_msaaColor);
	if (m_msaaDepth) glDeleteRenderbuffers(1, &m_msaaDepth);
	if (m_texture) glDeleteTextures(1, &m_texture);
	m_msaaFbo = m_resolveFbo = m_msaaColor = m_msaaDepth = m_texture = 0;
}

// Allocates (or reallocates on viewport resize) 4x MSAA color/depth renderbuffers and a single-sample resolve texture.
void Framebuffer::resize(int width, int height) {
	width  = std::max(width, 1);
	height = std::max(height, 1);
	if (width == m_width && height == m_height && m_msaaFbo) return;
	release();
	m_width = width; m_height = height;

	glCreateRenderbuffers(1, &m_msaaColor);
	glNamedRenderbufferStorageMultisample(m_msaaColor, kSamples, GL_RGBA8, width, height);
	glCreateRenderbuffers(1, &m_msaaDepth);
	glNamedRenderbufferStorageMultisample(m_msaaDepth, kSamples, GL_DEPTH24_STENCIL8, width, height);

	glCreateFramebuffers(1, &m_msaaFbo);
	glNamedFramebufferRenderbuffer(m_msaaFbo, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, m_msaaColor);
	glNamedFramebufferRenderbuffer(m_msaaFbo, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_msaaDepth);

	glCreateTextures(GL_TEXTURE_2D, 1, &m_texture);
	glTextureStorage2D(m_texture, 1, GL_RGBA8, width, height);
	glTextureParameteri(m_texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(m_texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(m_texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(m_texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glCreateFramebuffers(1, &m_resolveFbo);
	glNamedFramebufferTexture(m_resolveFbo, GL_COLOR_ATTACHMENT0, m_texture, 0);
}

void Framebuffer::bind() const {
	glBindFramebuffer(GL_FRAMEBUFFER, m_msaaFbo);
	glViewport(0, 0, m_width, m_height);
}

// Downsamples the 4x MSAA color attachment into `m_texture` so Dear ImGui can sample it.
void Framebuffer::resolve() const {
	glBlitNamedFramebuffer(m_msaaFbo, m_resolveFbo, 0, 0, m_width, m_height, 0, 0, m_width, m_height,
						   GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::readPixels(unsigned char* rgba) const {
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glGetTextureImage(m_texture, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_width * m_height * 4, rgba);
}

Mesh::~Mesh() {
	if (m_vbo) glDeleteBuffers(1, &m_vbo);
	if (m_vao) glDeleteVertexArrays(1, &m_vao);
}

// Configures DSA vertex attributes (location 0 = position, location 1 = normal) and uploads triangle data.
void Mesh::upload(const std::vector<float>& data) {
	if (!m_vao) {
		glCreateVertexArrays(1, &m_vao);
		glCreateBuffers(1, &m_vbo);
		glVertexArrayVertexBuffer(m_vao, 0, m_vbo, 0, 6 * sizeof(float));
		glEnableVertexArrayAttrib(m_vao, 0);
		glVertexArrayAttribFormat(m_vao, 0, 3, GL_FLOAT, GL_FALSE, 0);
		glVertexArrayAttribBinding(m_vao, 0, 0);
		glEnableVertexArrayAttrib(m_vao, 1);
		glVertexArrayAttribFormat(m_vao, 1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float));
		glVertexArrayAttribBinding(m_vao, 1, 0);
	}
	glNamedBufferData(m_vbo, (GLsizeiptr)(data.size() * sizeof(float)), data.data(), GL_STATIC_DRAW);
	m_count = (GLsizei)(data.size() / 6);
}

void Mesh::draw() const {
	glBindVertexArray(m_vao);
	glDrawArrays(GL_TRIANGLES, 0, m_count);
	glBindVertexArray(0);
}

// Builds a unit cube centred at the origin spanning [-0.5, +0.5] along each axis.
void Mesh::makeBox(Mesh& m) {
	std::vector<float> v;
	const glm::vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
	for (const glm::vec3& n : normals) {
		const glm::vec3 a = std::abs(n.z) > 0.5f ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);
		const glm::vec3 b = glm::cross(n, a), c = n * 0.5f;
		const glm::vec3 p0 = c - a * 0.5f - b * 0.5f, p1 = c + a * 0.5f - b * 0.5f;
		const glm::vec3 p2 = c + a * 0.5f + b * 0.5f, p3 = c - a * 0.5f + b * 0.5f;
		pushVertex(v, p0, n); pushVertex(v, p1, n); pushVertex(v, p2, n);
		pushVertex(v, p0, n); pushVertex(v, p2, n); pushVertex(v, p3, n);
	}
	m.upload(v);
}

// Builds a capped unit cylinder of radius 1 along Z in [0, 1].
void Mesh::makeCylinder(Mesh& m, int seg) {
	std::vector<float> v;
	for (int i = 0; i < seg; ++i) {
		const float a0 = 2 * kPi * i / seg, a1 = 2 * kPi * (i + 1) / seg;
		const glm::vec3 d0(std::cos(a0), std::sin(a0), 0), d1(std::cos(a1), std::sin(a1), 0), up(0, 0, 1);
		pushVertex(v, d0, d0); pushVertex(v, d1, d1); pushVertex(v, d1 + up, d1);
		pushVertex(v, d0, d0); pushVertex(v, d1 + up, d1); pushVertex(v, d0 + up, d0);
		pushVertex(v, up, up); pushVertex(v, d0 + up, up); pushVertex(v, d1 + up, up);
		pushVertex(v, glm::vec3(0), -up); pushVertex(v, d1, -up); pushVertex(v, d0, -up);
	}
	m.upload(v);
}

// Builds a capped unit cone with base radius 1 at z = 0 and apex at z = 1.
void Mesh::makeCone(Mesh& m, int seg) {
	std::vector<float> v;
	const glm::vec3 apex(0, 0, 1);
	for (int i = 0; i < seg; ++i) {
		const float a0 = 2 * kPi * i / seg, a1 = 2 * kPi * (i + 1) / seg;
		const glm::vec3 d0(std::cos(a0), std::sin(a0), 0), d1(std::cos(a1), std::sin(a1), 0);
		const glm::vec3 n0 = glm::normalize(d0 + apex), n1 = glm::normalize(d1 + apex);
		pushVertex(v, d0, n0); pushVertex(v, d1, n1); pushVertex(v, apex, glm::normalize(n0 + n1));
		pushVertex(v, glm::vec3(0), {0, 0, -1}); pushVertex(v, d1, {0, 0, -1}); pushVertex(v, d0, {0, 0, -1});
	}
	m.upload(v);
}

// Builds a UV sphere of radius 1 centred at the origin.
void Mesh::makeSphere(Mesh& m, int stacks, int slices) {
	std::vector<float> v;
	auto at = [](float theta, float phi) {
		return glm::vec3(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
	};
	for (int i = 0; i < stacks; ++i) {
		const float t0 = kPi * i / stacks, t1 = kPi * (i + 1) / stacks;
		for (int j = 0; j < slices; ++j) {
			const float p0 = 2 * kPi * j / slices, p1 = 2 * kPi * (j + 1) / slices;
			const glm::vec3 a = at(t0, p0), b = at(t1, p0), c = at(t1, p1), d = at(t0, p1);
			pushVertex(v, a, a); pushVertex(v, b, b); pushVertex(v, c, c);
			pushVertex(v, a, a); pushVertex(v, c, c); pushVertex(v, d, d);
		}
	}
	m.upload(v);
}

// ============================================================================
// OrbitCamera, Painter & Axis Triad
// ============================================================================

glm::vec3 OrbitCamera::position() const {
	const float yaw = glm::radians(yawDeg), pitch = glm::radians(pitchDeg);
	return target + glm::vec3(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)) * distance;
}
glm::mat4 OrbitCamera::view() const { return glm::lookAt(position(), target, glm::vec3(0, 0, 1)); }
glm::mat4 OrbitCamera::projection(float aspect) const {
	return glm::perspective(glm::radians(fovDeg), std::max(aspect, 0.01f), distance * 0.01f, distance * 100.0f + 500.0f);
}
void OrbitCamera::rotate(float dx, float dy) {
	yawDeg -= dx * kRotateDegPerPx;
	pitchDeg = std::clamp(pitchDeg + dy * kRotateDegPerPx, -kMaxPitchDeg, kMaxPitchDeg);
}
void OrbitCamera::pan(float dx, float dy, float viewportHeight) {
	const glm::mat4 v = view();
	const glm::vec3 right(v[0][0], v[1][0], v[2][0]), up(v[0][1], v[1][1], v[2][1]);
	const float scale = 2.0f * distance * std::tan(glm::radians(fovDeg) * 0.5f) / std::max(viewportHeight, 1.0f);
	target += (-dx * right + dy * up) * scale;
}
void OrbitCamera::zoom(float wheelSteps) {
	distance = std::clamp(distance * std::pow(kZoomPerWheelStep, wheelSteps), kMinDistanceM, kMaxDistanceM);
}

// Compiles the lighting shader and uploads shared unit meshes once per SceneView.
Painter::Painter() {
	m_shader.build(kVertex, kFragment);
	Mesh::makeBox(m_box);
	Mesh::makeCylinder(m_cylinder, 32);
	Mesh::makeCone(m_cone, 24);
	Mesh::makeSphere(m_sphere, 16, 24);
	Mesh::makeSphere(m_sphereFine, 96, 192);
}

void Painter::begin(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cameraPos) {
	m_labels.clear();
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	glDisable(GL_SCISSOR_TEST);
	m_shader.use();
	m_shader.set("u_viewProjection", proj * view);
	m_shader.set("u_cameraPos", cameraPos);
}

// Clears the depth buffer so coordinate-frame triads drawn afterwards stay visible in front of solid geometry.
void Painter::overlayPass() { glClear(GL_DEPTH_BUFFER_BIT); }

void Painter::draw(const Mesh& mesh, const glm::mat4& model, const glm::vec3& color) {
	m_shader.set("u_model", model);
	m_shader.set("u_color", color);
	mesh.draw();
}
void Painter::box(const glm::mat4& model, const glm::vec3& color) { draw(m_box, model, color); }
void Painter::box(const glm::vec3& center, const glm::vec3& size, const glm::vec3& color, const glm::mat4& parent) {
	draw(m_box, glm::scale(glm::translate(parent, center), size), color);
}

// Constructs an orthonormal transform mapping the unit [0, 1] Z axis onto segment a -> b with XY scaled by `radius`.
glm::mat4 Painter::alongSegment(const glm::vec3& a, const glm::vec3& b, float radius) {
	const glm::vec3 d = b - a;
	const float len = glm::length(d);
	const glm::vec3 z = len > 1e-6f ? d / len : glm::vec3(0, 0, 1);
	const glm::vec3 helper = std::abs(z.z) < 0.9f ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0);
	const glm::vec3 x = glm::normalize(glm::cross(helper, z)), y = glm::cross(z, x);
	glm::mat4 m(1.0f);
	m[0] = glm::vec4(x * radius, 0); m[1] = glm::vec4(y * radius, 0);
	m[2] = glm::vec4(z * len, 0);    m[3] = glm::vec4(a, 1);
	return m;
}
void Painter::cylinder(const glm::vec3& a, const glm::vec3& b, float radius, const glm::vec3& color) {
	draw(m_cylinder, alongSegment(a, b, radius), color);
}
void Painter::cone(const glm::vec3& base, const glm::vec3& apex, float radius, const glm::vec3& color) {
	draw(m_cone, alongSegment(base, apex, radius), color);
}
void Painter::sphere(const glm::vec3& center, float radius, const glm::vec3& color, int detail) {
	draw(detail > 0 ? m_sphereFine : m_sphere, glm::scale(glm::translate(glm::mat4(1.0f), center), glm::vec3(radius)), color);
}
void Painter::label(const glm::vec3& world, std::string text, LabelKind kind, float side) {
	m_labels.push_back({world, std::move(text), kind, side});
}

// Renders 3D RGB arrows along the columns of `frame` and queues a frame-name tag at its origin.
void drawAxisTriad(Painter& painter, const glm::mat4& frame, float length, const char* name) {
	static const glm::vec3 kAxisColors[3] = {{0.88f, 0.19f, 0.19f}, {0.18f, 0.63f, 0.26f}, {0.18f, 0.44f, 0.88f}};
	const glm::vec3 origin(frame[3]);
	for (int i = 0; i < 3; ++i) {
		const glm::vec3 axis = glm::normalize(glm::vec3(frame[i]));
		const glm::vec3 shaftEnd = origin + axis * (length * 0.78f);
		painter.cylinder(origin, shaftEnd, length * 0.025f, kAxisColors[i]);
		painter.cone(shaftEnd, origin + axis * length, length * 0.07f, kAxisColors[i]);
	}
	painter.sphere(origin, length * 0.045f, glm::vec3(0.25f));
	if (name) painter.label(origin, name, LabelKind::Frame);
}

// ============================================================================
// VdeScene & SpacecraftScene
// ============================================================================

namespace {
constexpr float kBaseTop = 0.12f, kPlatformNominal = 1.10f, kOverheadZ = 3.20f;
constexpr float kBaseAnchorR = 0.85f, kPlatAnchorR = 0.52f;

// Computes a circular actuator joint anchor point in cylindrical coordinates.
glm::vec3 anchor(float radius, float angleDeg, float z) {
	const float a = glm::radians(angleDeg);
	return {radius * std::cos(a), radius * std::sin(a), z};
}

// Draws a cylindrical docking collar with three angled alignment guide petals.
void dockingMechanism(Painter& p, const glm::mat4& frame, float dir, const glm::vec3& color) {
	auto at = [&](float x, float y, float z) { return glm::vec3(frame * glm::vec4(x, y, z * dir, 1)); };
	p.cylinder(at(0, 0, 0), at(0, 0, 0.30f), 0.28f, color * 0.85f);
	p.cylinder(at(0, 0, 0.30f), at(0, 0, 0.36f), 0.36f, color);
	for (int i = 0; i < 3; ++i) {
		const float a = glm::radians(120.0f * i + 30.0f);
		p.cylinder(at(0.30f * std::cos(a), 0.30f * std::sin(a), 0.36f),
				   at(0.16f * std::cos(a), 0.16f * std::sin(a), 0.56f), 0.035f, color * 1.1f);
	}
}

// Builds a rigid-body 4x4 transform from position and ZYX Roll-Pitch-Yaw angles (radians).
glm::mat4 attitude(const glm::vec3& pos, const Rpy& rpy) {
	glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
	m = glm::rotate(m, (float)rpy.yaw, {0, 0, 1});
	m = glm::rotate(m, (float)rpy.pitch, {0, 1, 0});
	return glm::rotate(m, (float)rpy.roll, {1, 0, 0});
}

// Draws symmetric port/starboard solar panel booms and wings attached to a spacecraft body frame.
void solarWings(Painter& p, const glm::mat4& body, float halfWidth, float span) {
	for (float side : {-1.0f, 1.0f}) {
		p.box({0, side * (halfWidth + 0.25f), 0}, {0.08f, 0.5f, 0.08f}, {0.62f, 0.64f, 0.68f}, body);
		p.box({0, side * (halfWidth + 0.5f + span * 0.5f), 0}, {span * 0.45f, span, 0.04f}, {0.16f, 0.24f, 0.45f}, body);
	}
}
} // namespace

OrbitCamera VdeScene::defaultCamera() const { return {{0.0f, 0.0f, 1.55f}, -62.0f, 16.0f, 7.4f, 40.0f}; }

// Renders the floor, Stewart platform base (B_sp), 6 telescoping actuators, moving platform (P_sp)
// driven by `snap.vde`, 4-post overhead gantry (O_sp), docking mechanisms, labels, and axis triads.
void VdeScene::render(Painter& p, const SimSnapshot& snap) {
	const VdeState& s = snap.vde;
	const glm::vec3 kSteel(0.55f, 0.57f, 0.60f), kDark(0.33f, 0.35f, 0.38f);

	p.box({0, 0, -0.02f}, {4.2f, 4.2f, 0.04f}, {0.82f, 0.83f, 0.85f});
	p.cylinder({0, 0, 0}, {0, 0, kBaseTop}, 1.0f, kDark);
	const glm::mat4 bSp = glm::translate(glm::mat4(1.0f), {0, 0, kBaseTop});

	glm::mat4 pSp = glm::translate(bSp, glm::vec3(s.positionM.x, s.positionM.y, s.positionM.z) + glm::vec3(0, 0, kPlatformNominal));
	pSp = glm::rotate(pSp, (float)s.orientationRad.yaw, {0, 0, 1});
	pSp = glm::rotate(pSp, (float)s.orientationRad.pitch, {0, 1, 0});
	pSp = glm::rotate(pSp, (float)s.orientationRad.roll, {1, 0, 0});

	for (int i = 0; i < 6; ++i) {
		const int pair = i / 2;
		const float side = (i % 2 == 0) ? -1.0f : 1.0f;
		const glm::vec3 b = glm::vec3(bSp * glm::vec4(anchor(kBaseAnchorR, 120.0f * pair + side * 14.0f, 0.0f), 1));
		const glm::vec3 t = glm::vec3(pSp * glm::vec4(anchor(kPlatAnchorR, 120.0f * pair + side * 46.0f, -0.06f), 1));
		const glm::vec3 mid = b + (t - b) * 0.55f;
		p.cylinder(b, mid, 0.050f, kDark);
		p.cylinder(mid, t, 0.032f, kSteel);
		p.sphere(b, 0.07f, kSteel);
		p.sphere(t, 0.06f, kSteel);
	}

	p.cylinder(glm::vec3(pSp * glm::vec4(0, 0, -0.08f, 1)), glm::vec3(pSp[3]), 0.66f, {0.62f, 0.70f, 0.80f});
	dockingMechanism(p, pSp, 1.0f, {0.86f, 0.64f, 0.25f});

	const float post = 1.75f, topZ = kOverheadZ + 0.35f;
	for (float x : {-post, post})
		for (float y : {-post, post}) p.box({x, y, topZ * 0.5f}, {0.12f, 0.12f, topZ}, kSteel);
	for (float y : {-post, post}) p.box({0, y, topZ}, {2 * post + 0.12f, 0.12f, 0.12f}, kSteel);
	p.box({0, 0, topZ}, {0.16f, 2 * post, 0.12f}, kSteel);
	p.box({0, 0, kOverheadZ + 0.08f}, {1.6f, 1.6f, 0.16f}, {0.70f, 0.72f, 0.75f});
	p.box({0, 0, (kOverheadZ + topZ) * 0.5f + 0.08f}, {0.2f, 0.2f, topZ - kOverheadZ - 0.1f}, kSteel);
	glm::mat4 oSp = glm::translate(glm::mat4(1.0f), {0, 0, kOverheadZ});
	dockingMechanism(p, oSp, -1.0f, {0.36f, 0.56f, 0.82f});

	p.label({-0.9f, 0.0f, kBaseTop * 0.5f}, "Base (B_sp)", LabelKind::Object, -1.0f);
	p.label(glm::vec3(pSp * glm::vec4(-0.6f, 0.0f, -0.04f, 1)), "Moving platform (P_sp)", LabelKind::Object, -1.0f);
	p.label({-0.8f, 0.0f, kOverheadZ + 0.08f}, "Overhead platform (O_sp)", LabelKind::Object, -1.0f);
	p.label(glm::vec3(pSp * glm::vec4(0.3f, 0, 0.25f, 1)), "Chaser docking mechanism", LabelKind::Object, 1.0f);
	p.label({0.3f, 0.0f, kOverheadZ - 0.25f}, "Target docking mechanism", LabelKind::Object, 1.0f);

	p.overlayPass();
	drawAxisTriad(p, bSp, 0.6f, "B_sp");
	drawAxisTriad(p, pSp, 0.6f, "P_sp");
	drawAxisTriad(p, oSp, 0.6f, "O_sp");
}

OrbitCamera SpacecraftScene::defaultCamera() const { return {{0.3f, 0.0f, -1.2f}, -76.0f, 10.0f, 12.5f, 40.0f}; }

// Renders the curved Earth horizon sphere, Chaser and Target spacecraft buses, solar arrays,
// docking adapters, object callouts, and body/docking coordinate frames (B_C, D_C, D_T, B_T).
void SpacecraftScene::render(Painter& p, const SimSnapshot& snap) {
	const SpacecraftBundle& sc = snap.spacecraft;
	const glm::vec3 kMech(0.62f, 0.64f, 0.68f);

	p.sphere({0, 0, -75.0f}, 70.0f, {0.20f, 0.42f, 0.75f}, 1);
	p.label({-6.0f, -6.0f, -5.3f}, "Earth", LabelKind::Object, 1.0f);

	const float gap = 0.9f + 0.5f * (float)std::sin(snap.timeS * 0.25);
	Rpy chaserAtt = sc.chaser.eciOrientationRad; chaserAtt.yaw = 0.0;
	Rpy targetAtt = sc.target.eciOrientationRad; targetAtt.yaw = 0.0;
	targetAtt.roll *= 0.4; targetAtt.pitch *= 0.4;

	const glm::vec3 targetSize(3.0f, 2.6f, 2.6f), chaserSize(2.0f, 1.7f, 1.7f);
	const float mech = 0.45f;
	const glm::mat4 bT = attitude({targetSize.x * 0.5f + mech + 0.1f, 0, 0}, targetAtt);
	const glm::mat4 bC = attitude({-(chaserSize.x * 0.5f + mech + gap * 0.5f + 0.1f), 0, 0}, chaserAtt);

	p.box({0, 0, 0}, targetSize, {0.86f, 0.87f, 0.89f}, bT);
	p.box({0, 0, 0}, chaserSize, {0.85f, 0.70f, 0.38f}, bC);
	solarWings(p, bT, targetSize.y * 0.5f, 3.2f);
	solarWings(p, bC, chaserSize.y * 0.5f, 2.0f);

	const glm::mat4 dC = glm::translate(bC, {chaserSize.x * 0.5f + mech, 0, 0});
	const glm::mat4 dT = glm::rotate(glm::translate(bT, {-(targetSize.x * 0.5f + mech), 0, 0}), glm::radians(180.0f), {0, 0, 1});
	for (const glm::mat4* d : {&dC, &dT}) {
		const glm::vec3 tip(*d * glm::vec4(0, 0, 0, 1)), root(*d * glm::vec4(-mech, 0, 0, 1)), lip(*d * glm::vec4(-0.08f, 0, 0, 1));
		p.cylinder(root, tip, 0.38f, kMech);
		p.cylinder(lip, tip, 0.48f, kMech * 1.15f);
	}

	p.label(glm::vec3(bC * glm::vec4(0, 0, -chaserSize.z * 0.5f - 0.35f, 1)), "CHASER", LabelKind::Object, -1.0f);
	p.label(glm::vec3(bT * glm::vec4(0, 0, -targetSize.z * 0.5f - 0.35f, 1)), "TARGET", LabelKind::Object, 1.0f);

	p.overlayPass();
	drawAxisTriad(p, bC, 0.9f, "B_C");
	drawAxisTriad(p, dC, 0.7f, "D_C");
	drawAxisTriad(p, dT, 0.7f, "D_T");
	drawAxisTriad(p, bT, 0.9f, "B_T");
}

} // namespace vde::render
