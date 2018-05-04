#pragma once

#include "OpenGLRenderer.hpp"

#include <glad/glad.h>

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>

#include <Eigen/Core>

#include "Verschiebung.h"
class ExampleRenderer : public OpenGLRenderer
{
	Q_OBJECT

public:
	ExampleRenderer(QObject * parent);

	void resize(int w, int h) override;
	void render() override;

	void mouseEvent(QMouseEvent * e) override;

private:
	double cameraAzimuth, cameraElevation;
	bool rotateInteraction;
	QPointF lastPos;

	Eigen::Matrix4d
		projectionMatrix, inverseProjectionMatrix,
		viewMatrix, inverseViewMatrix, translation, scale_earth, scale_moon;

	QOpenGLBuffer
		icosahedronVertexBuffer, icosahedronIndexBuffer,
		skyboxVertexBuffer, skyboxIndexBuffer,
		moonBoxVertexBuffer, moonBoxIndexBuffer;


	QOpenGLVertexArrayObject
		icosahedronVAO,
		skyboxVAO,
		moonBoxVAO;

	QOpenGLShaderProgram
		icosahedronProgram,
		skyboxProgram,
		moonBox;

	QOpenGLTexture
		earthTexture,
		skyboxTexture,
		MoonTexture;
};