#define _USE_MATH_DEFINES
#define GLFW_INCLUDE_GLU
#include<GLFW/glfw3.h>
#include<math.h>
#include<vector>
#include<Eigen/Dense>
#define oo 100000

struct vector2{
	double x, y;
};
struct vector3{
	double x, y, z;
	struct vector3& operator+=(const vector3& a){
		x += a.x;
		y += a.y;
		z += a.z;
		return *this;
	}
	struct vector3& operator-=(const vector3& a){
		x -= a.x;
		y -= a.y;
		z -= a.z;
		return *this;
	}
	struct vector3& operator*=(const double a){
		x *= a;
		y *= a;
		z *= a;
		return *this;
	}
	struct vector3& operator/=(const double a){
		x /= a;
		y /= a;
		z /= a;
		return *this;
	}
};
vector3 operator+(vector3 a, const vector3& b){
	return a += b;
}
vector3 operator-(vector3 a, const vector3& b){
	return a -= b;
}
vector3 operator*(vector3 a, const double b){
	return a *= b;
}
vector3 operator/(vector3 a, const double b){
	return a /= b;
}

vector3 vec_uniform(vector3 a){
	double len = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
	return a / len;

}
vector3 vec_cross(vector3 a, vector3 b){
	vector3 c;
	c.x = a.y*b.z - a.z*b.y;
	c.y = a.z*b.x - a.x*b.z;
	c.z = a.x*b.y - a.y*b.x;
	return c;
}

struct Spring{
	int p1, p2;
};

Eigen::MatrixXf A;
Eigen::MatrixXf La;
Eigen::MatrixXf L;
std::vector<GLfloat> X, X1, X2;
std::vector<GLfloat> Y;
std::vector<GLfloat> M;
std::vector<GLfloat> rest_length;

float h;
int m, s;

float k_stiff;
int numIteration;
float gravity;

float getSpringLength(float ax, float ay, float az, float bx, float by, float bz){
	return sqrtf((ax - bx)*(ax - bx) + (ay-by)*(ay-by)+(az-bz)*(az-bz));
}

//모든 스프링의 stiffness, rest_length는 균일하다고 가정
bool initiateMassSpring(int numPoint, int numSpring, float timestep, float springstiffness, int iteration, float gravityForce, std::vector<GLfloat> points, std::vector<Spring> springs, std::vector<GLfloat> restLength){
	m = numPoint;
	s = numSpring;
	h = timestep;
	k_stiff = springstiffness;
	numIteration = iteration;
	gravity = gravityForce;

	X.resize(m * 3);
	X1.resize(m * 3);
	X2.resize(m * 3);
	for (int i = 0; i < m * 3; i++){
		X[i] = X1[i] = X2[i] = points[i];
	}

	rest_length = restLength;
	
	//각 Point의 mass는 균일하다고 가정
	//Diagonal Matrix이므로 그냥 1차원 배열로 선언
	M.resize(m);
	for (int i = 0; i < m; i++){
		M[i] = 1.0;
	}
	M[3] = 100000;
	//M[15] = 100000;

	A = Eigen::MatrixXf::Zero(m * 3, s * 3);

	//A = (p1,i) = 1, (p2, i) = -1
	//A = A ⓧ I (3 x 3)
	
	for (int i = 0; i < s; i++){
		int p1 = springs[i].p1;
		int p2 = springs[i].p2;
		A(p1 * 3, i*3) = 1;
		A(p1 * 3 + 1, i*3 + 1) = 1;
		A(p1 * 3 + 2, i*3 + 2) = 1;

		A(p2 * 3, i*3) = -1;
		A(p2 * 3 + 1, i*3 + 1) = -1;
		A(p2 * 3 + 2, i*3 + 2) = -1;
	}

	Eigen::MatrixXf At = A.transpose();

	Eigen::MatrixXf AAt = A * At;

	//Create  (M+h^2L)
	L = h*h*k_stiff*AAt;
	for (int i = 0; i < m; i++){
		L(i*3, i*3)+=M[i];
		L(i * 3 + 1, i * 3 + 1) += M[i];
		L(i * 3 + 2, i * 3 + 2) += M[i];
	}

	return true;
}

bool simulateMassSpring(std::vector<GLfloat> &vertices, std::vector<Spring> springs){
	
	// X1->X2
	// points -> X1
	X2 = X1;
	X1 = vertices;
	X = vertices;

	//Create y (as in 2 X2-X1)
	Y.resize(m * 3);

	for (int i = 0; i < m *3; i+=3){
		Y[i] = X1[i]*2;
		Y[i] -= X2[i];

		Y[i+1] = X1[i+1] * 2;
		Y[i+1] -= X2[i+1];

		if (Y[i + 1] < -1.2){
			Y[i + 1] = -1.2;
		}

		Y[i+2] = X1[i+2] * 2;
		Y[i+2] -= X2[i+2];
	}
	

	//Iteration for main Mass_Spring Simulation
	for (int k = 0; k < numIteration;k++){
		Eigen::MatrixXf D, JD;
		//Calculate D
		
		D = Eigen::MatrixXf::Zero(s * 3, 1);
		for (int i = 0; i < s; i++){
			int p1 = springs[i].p1, p2 = springs[i].p2;
			float curr_length = getSpringLength(X[p1 * 3], X[p1 * 3 + 1], X[p1 * 3 + 2], X[p2 * 3], X[p2 * 3 + 1], X[p2 * 3 + 2]);

			D(i*3, 0) += h*h*k_stiff * ((X[p1*3] - X[p2*3]) / curr_length)*rest_length[i];
			D(i*3+1, 0) += h*h*k_stiff * ((X[p1 * 3 + 1] - X[p2 * 3 + 1]) / curr_length)*rest_length[i];
			D(i*3+2, 0) += h*h*k_stiff * ((X[p1 * 3 + 2] - X[p2 * 3 + 2]) / curr_length)*rest_length[i];

			//printf("%lf\n", D(i * 3, 0)*D(i * 3, 0) + D(i * 3+1, 0)*D(i * 3+1, 0) + D(i * 3+2, 0)*D(i * 3+2, 0));
		}


		// create JD
		JD = A*D;

		// add external force
		for (int i = 0; i < m; i++){
			JD(i * 3, 0) += M[i] * Y[i * 3];
			JD(i * 3 + 1, 0) += M[i] * Y[i * 3 + 1];
			JD(i * 3 + 2, 0) += M[i] * Y[i * 3 + 2];
			JD(i * 3 + 1, 0) -= h*h*gravity;
		}

		//Add inertia
		
		// From D, Calculate X thru Cholesky
		Eigen::MatrixXf answerX = L.llt().solve(JD);

		//AnswerX into X
		for (int i = 0; i < m; i++){
			X[i*3] = answerX(i * 3, 0);
			X[i * 3 + 1] = answerX(i * 3 + 1, 0);
			X[i * 3 + 2] = answerX(i * 3 + 2, 0);
		}
	}
	
	vertices = X;
	return true;
}