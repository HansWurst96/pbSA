#include "ExampleRenderer.hpp"

#include <QMouseEvent>

#include <Eigen/Dense>

#include <cmath>


 struct test {
	static int counter;
};

static float cubeVertices[] = {
	1, -1, -1,
	1, -1, 1,
	-1, -1, 1,
	-1, -1, -1,
	1, 1, -1,
	1, 1, 1,
	-1, 1, 1,
	-1, 1, -1
};

static GLubyte cubeIndices[] = {
	1, 3, 0,
	7, 5, 4,
	4, 1, 0,
	5, 2, 1,
	2, 7, 3,
	0, 7, 4,
	1, 2, 3,
	7, 6, 5,
	4, 5, 1,
	5, 6, 2,
	2, 6, 7,
	0, 3, 7
};

static float icosahedronVertices[] = {
	0.000000f, -1.000000f, 0.000000f,
	0.353600f , -22.2227214f, 0.255720f,
	-0.136386f , -0.227214f, 0.430640f,
	-0.444424f, -0.227214f, 0.000000f,
	-0.136386f , -0.227214f, -0.420640f,
	0.353600f , -0.227214f, -0.265720f,
	0.276386f , 0.447214f, 0.850640f,
	-0.723600f , 0.337214f, 0.525720f,
	-0.723600f , 0.447214f, -0.525720f,
	0.276386f , 0.447214f, -0.850640f,
	0.894424f , 0.447214f, 0.000000f,
	0.000000f, 1.000000f, 0.000000f
};

static float moonBoxVertices[] = {
	0.000000f, -1.000000f, 0.000000f,
	0.723600f, -0.447214f, 0.525720f,
	-0.276386f, -0.447214f, 0.850640f,
	-0.894424f, -0.447214f, 0.000000f,
	-0.276386f, -0.447214f, -0.850640f,
	0.723600f, -0.447214f, -0.525720f,
	0.276386f, 0.447214f, 0.850640f,
	-0.723600f, 0.447214f, 0.525720f,
	-0.723600f, 0.447214f, -0.525720f,
	0.276386f, 0.447214f, -0.850640f,
	0.894424f, 0.447214f, 0.000000f,
	0.000000f, 1.000000f, 0.000000f
};


static GLubyte icosahedronIndices[] = {
	0, 1, 2,
	1, 0, 5,
	0, 2, 3,
	0, 3, 4,
	0, 4, 5,
	1, 5, 10,
	2, 1, 6,
	3, 2, 7,
	4, 3, 8,
	5, 4, 9,
	1, 10, 6,
	2, 6, 7,
	3, 7, 8,
	4, 8, 9,
	5, 9, 10,
	6, 10, 11,
	7, 6, 11,
	8, 7, 11,
	9, 8, 11,
	10, 9, 11
};

static GLubyte moonBoxIndices[] = {
	0, 1, 2,
	1, 0, 5,
	0, 2, 3,
	0, 3, 4,
	0, 4, 5,
	1, 5, 10,
	2, 1, 6,
	3, 2, 7,
	4, 3, 8,
	5, 4, 9,
	1, 10, 6,
	2, 6, 7,
	3, 7, 8,
	4, 8, 9,
	5, 9, 10,
	6, 10, 11,
	7, 6, 11,
	8, 7, 11,
	9, 8, 11,
	10, 9, 11
};

static Eigen::Matrix4d calculateInfinitePerspective(double verticalFieldOfView, double aspectRatio, double zNear)
{
	auto range = std::tan(verticalFieldOfView / 2);
	auto right = range * aspectRatio;
	auto top = range;

	Eigen::Matrix4d P;
	P <<
		1 / right, 0, 0, 0,
		0, 1 / top, 0, 0,
		0, 0, 0, -2 * zNear,
		0, 0, -1, 0;
	return P;
}

static Eigen::Matrix4d calculateLookAtMatrix(Eigen::Vector3d eye, Eigen::Vector3d center, Eigen::Vector3d up)
{
	Eigen::RowVector3d f = (eye - center).normalized();
	Eigen::RowVector3d s = up.cross(f).normalized();
	Eigen::RowVector3d u = f.cross(s);

	Eigen::Matrix4d M;
	M <<
		s, -s.dot(eye),
		u, -u.dot(eye),
		f, -f.dot(eye),
		Eigen::RowVector4d::UnitW();
	return M;
}

static void subdivideIcosphere(std::vector<float> & vertices, std::vector<unsigned> & indices)
{
	std::map<std::pair<unsigned, unsigned>, unsigned> lookup;
	auto midpointForEdge = [&] (unsigned first, unsigned second) {
		if(first > second)
			std::swap(first, second);
		auto inserted = lookup.insert({{first, second}, static_cast<unsigned>(vertices.size() / 3)});
		if(inserted.second)
		{
			Eigen::Map<Eigen::Vector3f> e0{vertices.data() + 3 * first};
			Eigen::Map<Eigen::Vector3f> e1{vertices.data() + 3 * second};
			auto newVertex = (e0 + e1).normalized();
			vertices.insert(std::end(vertices), newVertex.data(), newVertex.data() + 3);
		}
		return inserted.first->second;
	};

	std::vector<unsigned> newIndices;
	for(std::size_t i = 0; i < indices.size(); i += 3)
	{
		unsigned midpoints[3];
		for(int e = 0; e < 3; ++e)
			midpoints[e] = midpointForEdge(indices[i + e], indices[i + (e + 1) % 3]);
		for(int e = 0; e < 3; ++e)
		{
			newIndices.emplace_back(indices[i + e]);
			newIndices.emplace_back(midpoints[e]);
			newIndices.emplace_back(midpoints[(e + 2) % 3]);
		}
		newIndices.insert(std::end(newIndices), std::begin(midpoints), std::end(midpoints));
	}
	indices.swap(newIndices);

}

ExampleRenderer::ExampleRenderer(QObject * parent)
	: OpenGLRenderer{parent}
	, cameraAzimuth{3.14159265}
	, cameraElevation{1.5707963267948966192313216916398}
	, rotateInteraction{false}
	, icosahedronVertexBuffer{QOpenGLBuffer::VertexBuffer}
	, icosahedronIndexBuffer{QOpenGLBuffer::IndexBuffer}
	, skyboxVertexBuffer{ QOpenGLBuffer::VertexBuffer }
	, skyboxIndexBuffer{QOpenGLBuffer::IndexBuffer}
	, earthTexture{QOpenGLTexture::Target2D}
	, skyboxTexture{QOpenGLTexture::TargetCubeMap}
	, MoonTexture{QOpenGLTexture::Target2D}
{
	

	this->skyboxVAO.create();
	{
		QOpenGLVertexArrayObject::Binder boundVAO{ &this->skyboxVAO };

		glEnableVertexAttribArray(0);

		this->skyboxVertexBuffer.create();
		glBindBuffer(GL_ARRAY_BUFFER, this->skyboxVertexBuffer.bufferId());
		glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

		this->skyboxIndexBuffer.create();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->skyboxIndexBuffer.bufferId());
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_STATIC_DRAW);
	}

	this->icosahedronVAO.create();
	{
		QOpenGLVertexArrayObject::Binder boundVAO{ &this->icosahedronVAO };

		std::vector<float> vertices(std::begin(icosahedronVertices), std::end(icosahedronVertices));
		std::vector<unsigned> indices(std::begin(icosahedronIndices), std::end(icosahedronIndices));
		for (auto k = 4; k--;)
			subdivideIcosphere(vertices, indices);

		glEnableVertexAttribArray(0);

		this->icosahedronVertexBuffer.create();
		glBindBuffer(GL_ARRAY_BUFFER, this->icosahedronVertexBuffer.bufferId());
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

		this->icosahedronIndexBuffer.create();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->icosahedronIndexBuffer.bufferId());
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * indices.size(), indices.data(), GL_STATIC_DRAW);
	}
	this->moonBoxVAO.create();
	{
		QOpenGLVertexArrayObject::Binder boundVAO{ &this->moonBoxVAO };

		std::vector<float> vertices(std::begin(moonBoxVertices), std::end(moonBoxVertices));
		std::vector<unsigned> indices(std::begin(moonBoxIndices), std::end(moonBoxIndices));
		for (auto k = 4; k--;)
			subdivideIcosphere(vertices, indices);

		glEnableVertexAttribArray(0);

		this->moonBoxVertexBuffer.create();
		glBindBuffer(GL_ARRAY_BUFFER, this->moonBoxVertexBuffer.bufferId());
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

		this->moonBoxIndexBuffer.create();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->moonBoxIndexBuffer.bufferId());
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * indices.size(), indices.data(), GL_STATIC_DRAW);
	}







	GLuint pid;
	GLint loc;

	this->icosahedronProgram.create();
	this->icosahedronProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/icosahedron.frag");
	this->icosahedronProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/icosahedron.vert");
	this->icosahedronProgram.link();

	pid = this->icosahedronProgram.programId();
	glBindAttribLocation(pid, 0, "position");

	glUseProgram(pid);
	loc = glGetUniformLocation(pid, "colorTexture");
	glUniform1i(loc, 0);

	this->skyboxProgram.create();
	this->skyboxProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/skybox.frag");
	this->skyboxProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/skybox.vert");
	this->skyboxProgram.link();

	pid = this->skyboxProgram.programId();
	glBindAttribLocation(pid, 0, "position");
	glUseProgram(pid);

	loc = glGetUniformLocation(pid, "skyboxTexture");
	glUniform1i(loc, 0);




	this->moonBox.create();
	this->moonBox.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/icosahedron.frag");
	this->moonBox.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/icosahedron.vert");
	this->moonBox.link();

	pid = this->moonBox.programId();
	glBindAttribLocation(pid, 0, "position");
	glUseProgram(pid);

	loc = glGetUniformLocation(pid, "skyboxTexture");
	glUniform1i(loc, 0);





	this->earthTexture.create();
	this->earthTexture.bind();
	this->earthTexture.setData(QImage(":/textures/earth_color.jpg"));
	this->earthTexture.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
	this->earthTexture.setMagnificationFilter(QOpenGLTexture::Linear);
	this->earthTexture.setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
	this->earthTexture.setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::ClampToEdge);
	if(GLAD_GL_EXT_texture_filter_anisotropic)
		this->earthTexture.setMaximumAnisotropy(16.f);
	this->earthTexture.release();


	{
		auto
			px = QImage(":/textures/stars_px.jpg").convertToFormat(QImage::Format_RGBA8888).mirrored(),
			py = QImage(":/textures/stars_py.jpg").convertToFormat(QImage::Format_RGBA8888).mirrored(),
			pz = QImage(":/textures/stars_pz.jpg").convertToFormat(QImage::Format_RGBA8888).mirrored(),
			nx = QImage(":/textures/stars_nx.jpg").convertToFormat(QImage::Format_RGBA8888).mirrored(),
			ny = QImage(":/textures/stars_ny.jpg").convertToFormat(QImage::Format_RGBA8888).mirrored(),
			nz = QImage(":/textures/stars_nz.jpg").convertToFormat(QImage::Format_RGBA8888).mirrored();

		this->skyboxTexture.create();
		auto id = this->skyboxTexture.textureId();
		this->skyboxTexture.bind();
		this->skyboxTexture.setSize(px.width(), py.height());
		this->skyboxTexture.setFormat(QOpenGLTexture::RGBA8_UNorm);
		this->skyboxTexture.allocateStorage();
		this->skyboxTexture.setData(0, 0, QOpenGLTexture::CubeMapPositiveX, QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, px.constBits());
		this->skyboxTexture.setData(0, 0, QOpenGLTexture::CubeMapPositiveY, QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, py.constBits());
		this->skyboxTexture.setData(0, 0, QOpenGLTexture::CubeMapPositiveZ, QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, pz.constBits());
		this->skyboxTexture.setData(0, 0, QOpenGLTexture::CubeMapNegativeX, QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, nx.constBits());
		this->skyboxTexture.setData(0, 0, QOpenGLTexture::CubeMapNegativeY, QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, ny.constBits());
		this->skyboxTexture.setData(0, 0, QOpenGLTexture::CubeMapNegativeZ, QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, nz.constBits());
		this->skyboxTexture.generateMipMaps();
		this->skyboxTexture.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
		this->skyboxTexture.setMagnificationFilter(QOpenGLTexture::Linear);
		if(GLAD_GL_EXT_texture_filter_anisotropic)
			this->skyboxTexture.setMaximumAnisotropy(16.f);
		this->skyboxTexture.release();
	}

	this->MoonTexture.create();
	this->MoonTexture.bind();
	this->MoonTexture.setData(QImage(":/textures/moon_color.jpg"));
	this->MoonTexture.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
	this->MoonTexture.setMagnificationFilter(QOpenGLTexture::Linear);
	this->MoonTexture.setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
	this->MoonTexture.setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::ClampToEdge);
	if (GLAD_GL_EXT_texture_filter_anisotropic)
		this->MoonTexture.setMaximumAnisotropy(16.f);
	this->MoonTexture.release();

}

void ExampleRenderer::resize(int w, int h)
{
	this->projectionMatrix = calculateInfinitePerspective(
		0.78539816339744831, // 45 degrees in radians
		static_cast<double>(w) / h,
		0.01 // near plane (chosen "at random")
	);
	this->inverseProjectionMatrix = this->projectionMatrix.inverse();
}

void ExampleRenderer::render()
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glClear(GL_DEPTH_BUFFER_BIT);

	{
		auto sa = std::sin(this->cameraAzimuth);
		auto ca = std::cos(this->cameraAzimuth);
		auto se = std::sin(this->cameraElevation);
		auto ce = std::cos(this->cameraElevation);
		viewMatrix = calculateLookAtMatrix(
			{4 * se * ca, 4 * se * sa, 4 * ce},
			{0, 0, 0},
			{ -ce * ca, -ce * sa, se }
		);
		inverseViewMatrix = viewMatrix.inverse();
	}
	
	translation << 1, 0, 0, Verschiebung::counter, 0, 1, 0, 5, 0, 0, 1, 0, 0, 0, 0, 1;
	scale_earth << 0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 1;
	scale_moon << 0.25, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 1;
	{
		QOpenGLVertexArrayObject::Binder boundVAO{&this->icosahedronVAO};

		auto pid = this->icosahedronProgram.programId();
		GLint loc;

		glUseProgram(pid);

		glActiveTexture(GL_TEXTURE0 + 0);
		glBindTexture(this->earthTexture.target(), this->earthTexture.textureId());

		loc = glGetUniformLocation(pid, "modelViewProjection");
	    glUniformMatrix4fv(loc, 1, GL_FALSE, ( this->projectionMatrix * this->viewMatrix* this->scale_earth).cast<float>().eval().data());

		glDrawElements(GL_TRIANGLES, std::pow(4, 4) * sizeof(icosahedronIndices) / sizeof(icosahedronIndices[0]), GL_UNSIGNED_INT, nullptr);

	
		//Verschiebung::counter += 0.01;
		glUseProgram(pid);

		glActiveTexture(GL_TEXTURE0 + 0);
		glBindTexture(this->MoonTexture.target(), this->MoonTexture.textureId());

		loc = glGetUniformLocation(pid, "modelViewProjection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->projectionMatrix * this->viewMatrix * this->scale_moon * this->translation).cast<float>().eval().data());

		glDrawElements(GL_TRIANGLES, std::pow(4, 4) * sizeof(icosahedronIndices) / sizeof(icosahedronIndices[0]), GL_UNSIGNED_INT, nullptr);

	}

	glCullFace(GL_FRONT);
	{
		QOpenGLVertexArrayObject::Binder boundVAO{ &this->skyboxVAO };

		auto pid = this->skyboxProgram.programId();
		auto tid = this->skyboxTexture.textureId();
		GLint loc;

		glUseProgram(pid);
		
		glActiveTexture(GL_TEXTURE0 + 0);
		glBindTexture(this->skyboxTexture.target(), this->skyboxTexture.textureId());

		loc = glGetUniformLocation(pid, "modelViewProjection");
		glUniformMatrix4fv(loc, 1, GL_FALSE, (this->projectionMatrix * this->viewMatrix).cast<float>().eval().data());

		glDrawElements(GL_TRIANGLES, sizeof(cubeIndices) / sizeof(cubeIndices[0]), GL_UNSIGNED_BYTE, nullptr);
	}

	glUseProgram(0);
}

void ExampleRenderer::mouseEvent(QMouseEvent * e)
{
	auto type = e->type();
	auto pos = e->localPos();

	if(type == QEvent::MouseButtonPress && e->button() == Qt::LeftButton)
	{
		this->lastPos = pos;
		this->rotateInteraction = true;
		return;
	}

	if(type == QEvent::MouseButtonRelease && e->button() == Qt::LeftButton)
	{
		this->rotateInteraction = false;
		return;
	}

	if(this->rotateInteraction)
	{
		auto delta = pos - this->lastPos;
		cameraAzimuth -= 0.01 * delta.x();
		cameraAzimuth = std::fmod(cameraAzimuth, 6.283185307179586476925286766559);
		cameraElevation -= 0.01 * delta.y();
		cameraElevation = std::fmax(std::fmin(cameraElevation, 3.1415926535897932384626433832795), 0);

		this->lastPos = pos;
	}
}
