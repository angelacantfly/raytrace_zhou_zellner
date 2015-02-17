#include "sphere.h"
#include "parse.h"
#include "material.h"
#include "main.h"
#include "common.h"

#define PI 3.14159
// single glu quadric object to do opengl sphere drawing
static GLUquadricObj* sphereQuadric = gluNewQuadric();


Sphere::Sphere ()
: Shape ()
{
}


Sphere::~Sphere ()
{
}

// Source: http://www.cplusplus.com/forum/beginner/9735/
bool quadraticFormula(double a, double b, double c, double *totalResults)
{
    double D = sqrt( (b*b) - (4*a*c) );//shorter name
    
    if (!a)// a==0 => 2*a==0
    {
        cout<<"Denominator is zero\n";
        return false; // function failed -you can also use exceptions-
    }
    
    totalResults[0] = ( -b + D)/(2*a);
    totalResults[1] = ( -b - D)/(2*a);
    
    return true;//function succeded
}

double Sphere::intersect (Intersection& intersectionInfo)
{
	/*
	* This method determines if intersectionInfo.theRay visibly intersects
	* the sphere from the position of the start of intersectionInfo.theRay
    *
    * The first visible intersection point is at alpha along
    * the ray where
	*	alpha is the smallest non-negative real root of
	*	||v||^2 alpha^2 - 2 (u dot v)alpha + ||u||^2 - r^2
	*	where 
	*	v is the unit direction vector of intersectionInfo.theRay
	*   u is the vector for the start of intersectionInfo.theRay to the
	*		center of the sphere
	*	r is the radius of the sphere
	*/
    double beta;
    Point3d centerSphere = this->center;
    double radius = this->radius;
    
    Rayd theRay = intersectionInfo.theRay;
    Point3d camPos = theRay.getPos();
    Vector3d v = theRay.getDir();
    Vector3d u = Vector3d(camPos, centerSphere);
    
    
    double answers[2];
    answers[0] = -1;
    answers[1] = -1;
    
    double a = pow(v.length(),2);
    double b = -2 * u.dot(v);
    double c = pow(u.length(),2) - pow(radius, 2);
    


    if (!quadraticFormula(a, b ,c, &answers[0]))
        return -1;

    
    if (isnan(answers[0])) answers[0] = -1;
    if (isnan(answers[1])) answers[1] = -1;
    
    
    // Determine if intersectionInfo.theRay intersects the sphere in front of the camera
    if (answers[0] < 0)
        if (answers[1] < 0) return -1;
        else beta = answers[1];
    else
        if (answers[1] < 0) beta = answers[0];
        else beta = min(answers[0], answers[1]);
    
    
    // Store intersectionInfo on intersection point in intersectionInfo
    Point3d intersectionPoint = camPos + beta * v;
    intersectionInfo.iCoordinate = intersectionPoint;
    
    Vector3d normalV = Vector3d(center, intersectionPoint);
    normalV.normalize();
    intersectionInfo.normal = normalV;
    intersectionInfo.textured = this->textured;
    intersectionInfo.material = this->material;
    
    
    Vector3d w = Vector3d(center, intersectionPoint);
    
    // RAY_CASTING TODO (sphere intersection)
    // Determine if intersectionInfo.theRay intersects the sphere in front of the camera
    // if so, store intersectionInfo on intersection point in intersectionInfo
    // RAY_CASTING TODO (sphere intersection)
    
    return beta;

}


void Sphere::glDraw ()
{
	/*
	* draw the sphere with the appropriate material and textured status
	* at the desired position and radius, using a glu quadric (for
	* easy texture coordinate generation)
	*/

	material->glLoad();

	glPushMatrix();
	// move to the origin of the sphere
	glTranslatef(center[0], center[1], center[2]);

	// set up the glu object's parameters - smooth filled polys with normals
	gluQuadricDrawStyle(sphereQuadric, GLU_FILL);
	gluQuadricOrientation(sphereQuadric, GLU_OUTSIDE);
	gluQuadricNormals(sphereQuadric, GLU_SMOOTH);

	// only calculate tex coords if we need to
	if (options->textures && textured && material->textured())
	{
		gluQuadricTexture(sphereQuadric, GLU_TRUE);
		glEnable(GL_TEXTURE_2D);
	}

	// draw the sphere
	gluSphere(sphereQuadric, radius, 50, 50);
	glPopMatrix();

	material->glUnload();

	glDisable(GL_TEXTURE_2D);
}


inline istream& operator>> (istream& in, Sphere& s)
{
	return s.read(in);
}


istream& Sphere::read (istream& in)
{
	int numFlags = 4;
	Flag flags[4] = { { (char*)"-n", STRING, false, MAX_NAME,     name      },
	{ (char*)"-m", STRING, true,  MAX_NAME,     matname   },
	{ (char*)"-t", BOOL,   false, sizeof(bool), &textured },
	{ (char*)"-u", DOUBLE, false, sizeof(double), &bumpScale }
	};

	if (parseFlags(in, flags, numFlags) == -1)
	{
		cerr << "ERROR: Sphere: flag parsing failed!" << endl;
		return in;
	}

	if (bumpScale != 0)
		bumpMapped = true;


	in >> center;

	radius = nextDouble(in);

	if (in.fail())
	{
		cerr << "ERROR: Sphere: unknown stream error" << endl;
		return in;
	}

	return in;
}


inline ostream& operator<< (ostream& out, Sphere& s)
{
	return s.write(out);
}


ostream& Sphere::write (ostream& out)
{
	out << "#sphere -m " << matname << flush;
	if (name[0] != '\0')
		out << " -n " << name << flush;
	if (textured)
		out << " -t" << flush;
	out << " --" << endl;

	out << "  " << center << endl;
	out << "  " << radius << endl;

	return out;
}
