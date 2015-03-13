#include "box.h"
#include "parse.h"
#include "material.h"
#include "main.h"



Box::Box ()
: Shape ()
{
}


Box::~Box ()
{
}


double Box::planeIntersect (Rayd& ray, Point3d& p0, Vector3d& n)
{
    Vector3d l = ray.getDir();
    double d = -1;
    
    if (l.dot(n) != 0) {
        // there is a single pt of intersection
        Vector3d diff = p0-ray.getPos();
        d = diff.dot(n) / l.dot(n);
    }
    if (d >= 0)
        return d;
    else
        return 0;
}


double Box::intersect (Intersection& info)
{
    int l = size[0];
    int w = size[2];
    int h = size[1];
    
    // CENTERPOINTS
    Point3d topcenter(center[0],center[1]+h/2.0,center[2]);
    Point3d botcenter(center[0],center[1]-h/2.0,center[2]);
    Point3d rightcenter(center[0]+l/2.0,center[1],center[2]);
    Point3d leftcenter(center[0]-l/2.0,center[1],center[2]);
    Point3d frontcenter(center[0],center[1],center[2]+w/2.0);
    Point3d backcenter(center[0],center[1],center[2]-w/2.0);
    Point3d centerpoints[6] = {frontcenter, rightcenter, topcenter, leftcenter, backcenter, botcenter};
    
    Vector3d frontnorm = Vector3d(0,0,1);
    Vector3d  backnorm = Vector3d(0,0,-1);
    Vector3d leftnorm = Vector3d(-1,0,0);
    Vector3d rightnorm = Vector3d(1,0,0);
    Vector3d topnorm = Vector3d(0,1,0);
    Vector3d botnorm = Vector3d(0,-1,0);
    Vector3d norms[6] = {frontnorm, rightnorm, topnorm, leftnorm, backnorm, botnorm};

    double alpha = -1;
    
    for (int j=0; j<6; j++) {
//        cout << "norm:: " << norms[j] << endl;
        double dInt = planeIntersect(info.theRay, centerpoints[j], norms[j]);
        // dInt should be the shortest one we can get because we want the closest intersection
//        cout << dInt << endl;
        if (dInt != -1) {
            Point3d i = info.theRay.getPos() + dInt*info.theRay.getDir();
            info.iCoordinate = i;
           
            
//        cout << "intersection: " << i << endl;
            Vector3d n;
            n = norms[j];
            if (n.dot(info.theRay.getDir()) < 0) {
                info.entering = true;
            }
            else {
                n = -n;
                info.entering = false;
            }
            n.normalize();
            info.normal = n;
            
            info.material = this->material;
           
            info.textured = this->textured;
            if (info.textured || material->bumpMapped()) {
               
            
                
                
            }
            
        
        if (info.iCoordinate[0] <= center[0]+l/2.0 && info.iCoordinate[0] >= center[0]-l/2.0 &&
            info.iCoordinate[1] <= center[1]+h/2.0 && info.iCoordinate[1] >= center[1]-h/2.0 &&
            info.iCoordinate[2] <= center[2]+w/2.0 && info.iCoordinate[2] >= center[2]-w/2.0 &&
            info.entering)
            return Vector3d(info.iCoordinate-info.theRay.getPos()).length();
        }
    }
    return -1;
}



TexCoord2d Box::getTexCoordinates (Point3d i)
{
    int l = size[0];
    int w = size[2];
    int h = size[1];
    
    
    
    TexCoord2d t1(0,w);
    TexCoord2d t2(w,w);
    TexCoord2d t3(w,0);
    TexCoord2d t4(w+l,0);
    TexCoord2d t5(w+l,w);
    TexCoord2d t6((2*w)+l,w);
    TexCoord2d t7(2*(w+l),w);
    TexCoord2d t8(2*(w+l),w+h);
    TexCoord2d t9((2*w)+l,w+h);
    TexCoord2d t10(w+l,w+h);
    TexCoord2d t11(w+l,(2*w)+h);
    TexCoord2d t12(w,(2*w)+h);
    TexCoord2d t13(w,w+h);
    TexCoord2d t14(0,w+h);
    
    Vector2d frontleft();
    Vector2d frontright();
    Vector2d backleft();
    Vector2d backright();
    Vector2d topfront();
    Vector2d topback();
    Vector2d topleft();
    Vector2d topright();
    Vector2d botfront();
    Vector2d botback();
    Vector2d botleft();
    Vector2d botright();
    
//	return tCoord;
}



void Box::glutTexturedCube (GLfloat size)
{
	static GLfloat n[6][3] =
	{ 
		{-1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0},
		{1.0, 0.0, 0.0},
		{0.0, -1.0, 0.0},
		{0.0, 0.0, 1.0},
		{0.0, 0.0, -1.0}
	};
	static GLint faces[6][4] =
	{
		{0, 1, 2, 3},
		{3, 2, 6, 7},
		{7, 6, 5, 4},
		{4, 5, 1, 0},
		{5, 6, 2, 1},
		{7, 4, 0, 3}
	};
	GLfloat v[8][3];
	GLint i;

	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -size / 2;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = size / 2;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -size / 2;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = size / 2;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = size / 2;

	for (i = 5; i >= 0; i--) {
		glBegin(GL_QUADS);
		glNormal3fv(&n[i][0]);
		glTexCoord2f(0.0,0.0);
		glVertex3fv(&v[faces[i][0]][0]);
		glTexCoord2f(1.0,0.0);
		glVertex3fv(&v[faces[i][1]][0]);
		glTexCoord2f(1.0,1.0);
		glVertex3fv(&v[faces[i][2]][0]);
		glTexCoord2f(0.0,1.0);
		glVertex3fv(&v[faces[i][3]][0]);
		glEnd();
	}
}


void Box::glDraw ()
{
	material->glLoad();

	glPushMatrix();
	// move to the origin of the Box
	glTranslatef(center[0], center[1], center[2]);

	// scale to the appropriate size
	glScalef(size[0], size[1], size[2]);

	if (options->textures && textured && material->textured())
	{
		glEnable(GL_TEXTURE_2D);
	}

	// draw a unit cube
	glutTexturedCube(1);
	glPopMatrix();

	material->glUnload();
	glDisable(GL_TEXTURE_2D);
}


inline istream& operator>> (istream& in, Box& s)
{
	return s.read(in);
}


istream& Box::read (istream& in)
{
	int numFlags = 4;
	Flag flags[4] = { { (char*)"-n", STRING, false, MAX_NAME,       name      },
	{ (char*)"-m", STRING, true,  MAX_NAME,       matname   },
	{ (char*)"-t", BOOL,   false, sizeof(bool),   &textured },
	{ (char*)"-u", DOUBLE, false, sizeof(double), &bumpScale }
	};

	if (parseFlags(in, flags, numFlags) == -1)
	{
		cerr << "ERROR: Box: flag parsing failed!" << endl;
		return in;
	}

	if (bumpScale != 0)
		bumpMapped = true;

	in >> center;
	in >> size;

	size[0] = fabs(size[0]);
	size[1] = fabs(size[1]);
	size[2] = fabs(size[2]);

	if (in.fail())
	{
		cerr << "ERROR: Box: unknown stream error" << endl;
		return in;
	}

	return in;
}


inline ostream& operator<< (ostream& out, Box& s)
{
	return s.write(out);
}


ostream& Box::write (ostream& out)
{
	out << "#box -m " << matname << flush;
	if (name[0] != '\0')
		out << " -n " << name << flush;
	if (textured)
		out << " -t" << flush;
	if (bumpMapped)
		out << " -u" << flush;
	out << " --" << endl;

	out << "  " << center << endl;
	out << "  " << size << endl;

	return out;
}
