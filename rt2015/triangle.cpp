#include "triangle.h"
#include "parse.h"
#include "material.h"
#include "main.h"


Triangle::Triangle ()
: Shape ()
{
}


Triangle::~Triangle ()
{
}


double Triangle::intersect (Intersection& intersectionInfo)
{
	/*
	* This method solves for inersection between intersectionInfo.theRay and
	* the triangle. 
	*   If there is no intersection OR if the ray lies in the
	*	plane containing the triangle we return -1;
	*	Otherwise we return alpha the distance from the
	*	starting point of intersectionInfo.theRay to the intersection point
	
	*
	* Then, decompose the vector p - v0 into a linear combination of
	*/

    Point3d v0 = this->v[0];
    Point3d v1 = this->v[1];
    Point3d v2 = this->v[2];
    Point3d camPos = intersectionInfo.theRay.getPos();
    Vector3d u(camPos, v0);
    Vector3d ray = intersectionInfo.theRay.getDir();
    
    // (1) determine intersection with plane (if any)
    // the ray is parallel to one of the sides of the triangle
    // therefore, no intersection
    if (this->n.dot(ray) == 0)
        return -1;
    
    double alpha = (n.dot(u))/(n.dot(ray));
    if (alpha < 0)
        return -1;

    // (2) test if intersection point is in triangle
    Vector3d w1(v0,v1);
    Vector3d w2(v0,v2);

    
    
    double av_u_w1 = (alpha*ray - u).dot(w1);
    double av_u_w2 = (alpha*ray - u).dot(w2);
    double w1w1 = w1.dot(w1);
    double w1w2 = w1.dot(w2);
    double w2w2 = w2.dot(w2);
    
    double gamma = - (w1w2 * av_u_w1 - av_u_w2 * w1w1)/ (w1w1 * w2w2 - pow(w1w2,2));
    double beta = (av_u_w1 - gamma * w1w2)/(w1w1);

    if (!(gamma >= 0 && beta >= 0 && beta + gamma <= 1))
        return -1;
    
    // (3) compute distance from start of ray to intersection point & normal in direction of incoming ray
    Point3d intersect = camPos + alpha * ray;
    intersectionInfo.iCoordinate = intersect;
    
    Vector3d w1w2pos = (w1.cross(w2)).getUnit();
    Vector3d w1w2neg = -w1w2pos;
    
    if (w1w2pos.dot(intersect-camPos) < 0)
        intersectionInfo.normal = n;
    else
        intersectionInfo.normal = -n;
    intersectionInfo.normal.normalize();

    // RAY_CASTING TODO (intersection)
    // (1) determine intersection with plane (if any)
    // (2) test if intersection point is in triangle
    // (3) compute distance from start of ray to intersection point & normal in direction of incoming ray
    
    intersectionInfo.textured = this->textured;
    intersectionInfo.material = this->material;
    
    // Texture mapping
    intersectionInfo.textured = this->textured;
    if (gamma >=0 && beta >0 && gamma + beta <= 1)
    {
        if (intersectionInfo.textured || material->bumpMapped())
        {
            // FIXME: ugh, conversion consequences?
            TexCoord2d f0 = this->t[0];
            TexCoord2d f1 = this->t[1];
            TexCoord2d f2 = this->t[2];
            Point2d ff0(f0[0],f0[1]);
            Point2d ff1(f1[0], f1[1]);
            Point2d ff2(f2[0], f2[1]);
            
            Vector2d t1(ff0, ff1);
            Vector2d t2(ff0,ff2);
            Point2d texOrigin(t[0][0], t[0][1]);
        
            Point2d texCoord_intersect = texOrigin + beta * t1 + gamma * t2;
            for (int c = 0; c <2; c++) {
                while (texCoord_intersect[c] < 0) {
                    texCoord_intersect[c] += 1;
                }
                while (texCoord_intersect[c] > 1) {
                    texCoord_intersect[c] -= 1;
                }
            }
            TexCoord2d coord(texCoord_intersect[0],texCoord_intersect[1]);
//          TexCoord2d coord(gamma,beta);
            intersectionInfo.texCoordinate = coord;
        
            if (material->bumpMapped()) {
                // Bump Mapping
                Vector3d up = Vector3d(0,1,0) - intersectionInfo.normal.dot(Vector3d(0,1,0)) * intersectionInfo.normal;
                up.normalize();
                Vector3d right = -intersectionInfo.normal.cross(up);
                right.normalize();
//                Vector3d up = w1.getUnit();
//                Vector3d right = -w2.getUnit();
                intersectionInfo.material->bumpNormal(intersectionInfo.normal, up, right, intersectionInfo, this->bumpScale);
            }
        }
    }

	return alpha;
}




void Triangle::glDraw ()
{
	/*
	* draw the sphere with the appropriate material and textured status
	* at the desired position and radius
	*/

	material->glLoad();

	if (options->textures && textured && material->textured())
	{
		glEnable(GL_TEXTURE_2D);
		// draw the triangle with normal and tex coords
		glBegin(GL_TRIANGLES);
		n.glLoad();
		t[0].glLoad();
		v[0].glLoad();
		t[1].glLoad();
		v[1].glLoad();
		t[2].glLoad();
		v[2].glLoad();
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}
	else
	{
		// draw the triangle with normal but no tex coords
		glBegin(GL_TRIANGLES);
		n.glLoad();
		v[0].glLoad();
		v[1].glLoad();
		v[2].glLoad();
		glEnd();
	}

	material->glUnload();
}


inline istream& operator>> (istream& in, Triangle& t)
{
	return t.read(in);
}


istream& Triangle::read (istream& in)
{
	int numFlags = 4;
	Flag flags[4] = { { (char*)"-n", STRING, false, MAX_NAME,     name      },
	{ (char*)"-m", STRING, true,  MAX_NAME,     matname   },
	{ (char*)"-t", BOOL,   false, sizeof(bool), &textured },
	{ (char*)"-u", DOUBLE, false, sizeof(double), &bumpScale }

	};

	if (parseFlags(in, flags, numFlags) == -1)
	{
		cerr << "ERROR: Triangle: flag parsing failed!" << endl;
		return in;
	}
	if (bumpScale != 0)
		bumpMapped = true;

	if (textured || bumpMapped)
		in >> v[0] >> t[0] >> v[1] >> t[1] >> v[2] >> t[2];
	else
		in >> v[0] >> v[1] >> v[2];

	if (in.fail())
	{
		cerr << "ERROR: Triangle: unknown stream error" << endl;
		return in;
	}

	// Vectors from vertex 0 to the other vertices
	Vector3d v1 = v[1]-v[0];
	Vector3d v2 = v[2]-v[0];  

	// Calculate this triangle's normal
	n = (v1).cross(v2);
	n.normalize();

	return in;
}


inline ostream& operator<< (ostream& out, Triangle& t)
{
	return t.write(out);
}


ostream& Triangle::write (ostream& out)
{
	out << "#triangle -m " << matname << flush;
	if (name[0] != '\0')
		out << " -n " << name << flush;
	if (textured)
		out << " -t" << flush;
	out << " --" << endl;

	if (textured)
	{
		out << "  " << v[0] << "   " << t[0] << endl
			<< "  " << v[1] << "   " << t[1] << endl
			<< "  " << v[2] << "   " << t[2] << endl;
	}
	else
	{
		out << "  " << v[0] << endl
			<< "  " << v[1] << endl
			<< "  " << v[2] << endl;
	}

	return out;
}
