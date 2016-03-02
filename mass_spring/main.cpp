/*
*	glfw test program
*	Requirements
*	glfw - 3.1.2. for WIN32
*		glfw3.h
*		glfw3.lib
*		glfw3dll.lib
*		glfw3.dll
*	openGL (Pre-Installed)
*		opengl32.lib
*
*
*/
#define GLFW_INCLUDE_GLU
#include<GLFW/glfw3.h>
#include<gl\GL.h>
#include<stdio.h>

#include <stdlib.h>
#include <stdio.h>
#include"dynamics.h"

class MassSpringObject{
public:
	std::vector<GLfloat> vertices;
	std::vector<Spring> springs;
	int numPoint, numSpring;
	std::vector<GLfloat> rest_length;
	void draw();
};

class SolidSphere : public MassSpringObject
{
protected:
	std::vector<GLfloat> normals;
	std::vector<GLfloat> texcoords;
	std::vector<GLushort> indices;

public:

	SolidSphere(float radius, unsigned int rings, unsigned int sectors)
	{
		float const R = 1. / (float)(rings - 1);
		float const S = 1. / (float)(sectors - 1);
		int r, s;

		vertices.resize(rings * sectors * 3);
		normals.resize(rings * sectors * 3);
		texcoords.resize(rings * sectors * 2);
		numPoint = vertices.size();

		std::vector<GLfloat>::iterator v = vertices.begin();
		std::vector<GLfloat>::iterator n = normals.begin();
		std::vector<GLfloat>::iterator t = texcoords.begin();
		for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
			float const y = sin(-M_PI_2 + M_PI * r * R);
			float const x = cos(2 * M_PI * s * S) * sin(M_PI * r * R);
			float const z = sin(2 * M_PI * s * S) * sin(M_PI * r * R);

			*t++ = s*S;
			*t++ = r*R;

			*v++ = x * radius;
			*v++ = y * radius;
			*v++ = z * radius;

			*n++ = x;
			*n++ = y;
			*n++ = z;
		}

		indices.resize(rings * sectors * 4);
		std::vector<GLushort>::iterator i = indices.begin();
		for (r = 0; r < rings - 1; r++) for (s = 0; s < sectors - 1; s++) {
			*i++ = r * sectors + s;
			*i++ = r * sectors + (s + 1);
			*i++ = (r + 1) * sectors + (s + 1);
			*i++ = (r + 1) * sectors + s;
		}
	}

	void draw(GLfloat x, GLfloat y, GLfloat z)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(x, y, z);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);

		glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
		glNormalPointer(GL_FLOAT, 0, &normals[0]);
		glTexCoordPointer(2, GL_FLOAT, 0, &texcoords[0]);
		glDrawElements(GL_QUADS, indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
		glPopMatrix();
	}
};

class Cloth2D : public MassSpringObject
{
protected:
	std::vector<GLfloat> normals;
	std::vector<GLfloat> texcoords;
	std::vector<GLshort> tri;
public:

	Cloth2D(){}
	Cloth2D(int row, int col, float length){
		vertices.resize(row*col * 3);
		normals.resize(row*col * 3);
		texcoords.resize(row*col * 2);
		std::vector<GLfloat>::iterator v = vertices.begin();
		std::vector<GLfloat>::iterator n = normals.begin();
		std::vector<GLfloat>::iterator t = texcoords.begin();
		for (int i = 0; i < row; i++){
			for (int j = 0; j < col; j++){
				*t++ = (float)i / row;
				*t++ = (float)j / col;

				*v++ = i*length - (length*row/2);
				*v++ = j*length - (length*col/2);
				*v++ = 0;

				*n++ = 0;
				*n++ = 0;
				*n++ = 1;
			}
		}
		
		for (int i = 0; i < row; i++){
			for (int j = 0; j < col; j++){
				if (i + 1 < row){
					springs.push_back({ i*row + j, (i + 1)*row + j });
					rest_length.push_back(getSpringLength(vertices[(i*row + j)*3], vertices[(i*row + j)*3+1], vertices[(i*row + j)*3+2],
						vertices[((i + 1)*row + j) * 3], vertices[((i + 1)*row + j) * 3 + 1], vertices[((i + 1)*row + j) * 3 + 2]));
				}
				if (j + 1 < col){
					springs.push_back({ i*row + j, i*row + j + 1 });
					rest_length.push_back(getSpringLength(vertices[(i*row + j) * 3], vertices[(i*row + j) * 3 + 1], vertices[(i*row + j) * 3 + 2],
						vertices[(i*row + j+1) * 3], vertices[(i*row + j+1) * 3 + 1], vertices[(i*row + j+1) * 3 + 2]));
				}
				if (i + 1 < row && j - 1 >= 0){
					springs.push_back({ i*row + j, (i + 1)*row + j - 1 });
					rest_length.push_back(getSpringLength(vertices[(i*row + j) * 3], vertices[(i*row + j) * 3 + 1], vertices[(i*row + j) * 3 + 2],
						vertices[((i + 1)*row + j - 1) * 3], vertices[((i + 1)*row + j - 1) * 3 + 1], vertices[((i + 1)*row + j - 1) * 3 + 2]));
				}
				if (i + 1 < row && j + 1 < col){
					springs.push_back({ i*row + j, (i + 1)*row + j + 1 });
					rest_length.push_back(getSpringLength(vertices[(i*row + j) * 3], vertices[(i*row + j) * 3 + 1], vertices[(i*row + j) * 3 + 2],
						vertices[((i + 1)*row + j + 1) * 3], vertices[((i + 1)*row + j + 1) * 3 + 1], vertices[((i + 1)*row + j + 1) * 3 + 2]));
				}
			}
		}

		tri.resize((row - 1)*(col - 1) * 2 * 3);
		std::vector<GLshort>::iterator tr = tri.begin();
		for (int i = 0; i < row - 1; i++){
			for (int j = 0; j < col - 1; j++){
				*tr++ = i*row + j;
				*tr++ = i*row + (j + 1);
				*tr++ = (i + 1)*row + j + 1;

				*tr++ = i*row + j;
				*tr++ = (i + 1) * row + j;
				*tr++ = (i + 1) * row + j + 1;
			}
		}

		numPoint = row*col;
		numSpring = springs.size();
	}

	void draw(){

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		//glTranslatef(x, y, z);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);

		glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
		glNormalPointer(GL_FLOAT, 0, &normals[0]);
		glTexCoordPointer(2, GL_FLOAT, 0, &texcoords[0]);
		glDrawElements(GL_TRIANGLES, tri.size(), GL_UNSIGNED_SHORT, &tri[0]);
		glPopMatrix();
	}
};

GLFWwindow* window;

SolidSphere sphere(1, 12, 24);
Cloth2D cloth;
vector3 camPos, camLook, camUp;
float theta, pi;
float camDist;
vector2 cursorPos;
bool mouse[3];
bool isPlaying = false;
float k_stiffness = 200;
float time_step = 0.033;
int num_iteration = 5;

bool draw(void){
	float ratio;
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;

	glViewport(0, 0, width, height);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, ratio, 1, 10);
	gluLookAt(camPos.x, camPos.y, camPos.z, camLook.x, camLook.y, camLook.z, camUp.x, camUp.y, camUp.z);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	//sphere.draw(0, 0, 0);
	cloth.draw();

	return true;
}
static void error_callback(int error, const char* description)
{

	fputs(description, stderr);
}
static void mousebutton_callback(GLFWwindow *window, int button, int action, int mods){
	if (action == GLFW_PRESS){
		mouse[button] = true;
	}
	else if (action == GLFW_RELEASE){
		mouse[button] = false;
	}
}
static void cursorpos_callback(GLFWwindow *window, double x, double y){
	//printf("Current Cursor : (%f, %f)\n", x, y);
	if (mouse[0]){

		theta += (x - cursorPos.x) * 0.01;
		pi += (y - cursorPos.y) * 0.01;
		camPos.x = camDist * sinf(theta)*cosf(pi);
		camPos.y = camDist * sinf(pi);
		camPos.z = -camDist * cosf(theta) * cosf(pi);
		camUp.x = -1 * sinf(theta) * sinf(pi);
		camUp.y = cosf(pi);
		camUp.z = cosf(theta)*sinf(pi);
	}
	if (mouse[1]){
	}
	cursorPos.x = x;
	cursorPos.y = y;
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{	
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS){
		if (isPlaying)
			printf("Pause Simulation\n");
		else
			printf("Start Simulation\n");
		isPlaying = !isPlaying;
	}
	else if (key == GLFW_KEY_UP && action == GLFW_PRESS){
		k_stiffness *= 2.0;
		initiateMassSpring(cloth.numPoint, cloth.numSpring, time_step, k_stiffness, num_iteration, 9.8, cloth.vertices, cloth.springs, cloth.rest_length);
		printf("K_stiffness Increased : %lf\n", k_stiff);
	}
	else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS){
		k_stiffness /= 2.0;
		initiateMassSpring(cloth.numPoint, cloth.numSpring, time_step, k_stiffness, num_iteration, 9.8, cloth.vertices, cloth.springs, cloth.rest_length);
		printf("K_stiffness Increased : %lf\n", k_stiff);
	}
	else if (key == GLFW_KEY_I && action == GLFW_PRESS){
		cloth = Cloth2D(4, 4, 0.3);
		initiateMassSpring(cloth.numPoint, cloth.numSpring, time_step, k_stiffness, num_iteration, 9.8, cloth.vertices, cloth.springs, cloth.rest_length);
		printf("Restart Simulation\n");
	}
}
static void mousescroll_callback(GLFWwindow *window, double x, double y){

	camDist -= 0.3*y;
	camPos.x = camDist * sinf(theta)*cosf(pi);
	camPos.y = camDist * sinf(pi);
	camPos.z = -camDist * cosf(theta) * cosf(pi);
}
int main(void)
{
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);
	window = glfwCreateWindow(640.0, 480.0, "Simple example", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	mouse[0] = mouse[1] = mouse[2] = false;
	camPos = { 0, 0, -5 };
	camLook = { 0, 0, 0 };
	camUp = { 0, 1, 0 };
	theta = pi = 0.0f;
	camDist = 5.0;
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetCursorPosCallback(window, cursorpos_callback);
	glfwSetMouseButtonCallback(window, mousebutton_callback);
	glfwSetKeyCallback(window, key_callback);

	glfwSetScrollCallback(window, mousescroll_callback);
	
	cloth = Cloth2D(4, 4, 0.3);
	initiateMassSpring(cloth.numPoint, cloth.numSpring, time_step, k_stiffness, num_iteration, 9.8, cloth.vertices, cloth.springs, cloth.rest_length);
	//sphere = SolidSphere(3, 10, 10);
	// initiateMassSpring(sphere.)

	printf("\n\n**Mass Spring Simulation**\n\n");

	printf("\"I\" : Initialize\n");
	printf("\"Space\" : Start/Pause simulation\n");
	printf("\"Up\" : Increase Stiffness\n");
	printf("\"Down\" : Decrease Stiffness\n");

	while (!glfwWindowShouldClose(window))
	{
		if (isPlaying){
			simulateMassSpring(cloth.vertices, cloth.springs);
			//isPlaying = false;
		}
		draw();
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}