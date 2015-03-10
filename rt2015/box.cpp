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
//        cout << "d is: " << d << endl;
        return d;
    }
    // ray and plane are parallel
    return 0;
}


double Box::intersect (Intersection& info)
{
    // ray-slab intersection approach adapted from
    // http://tavianator.com/2011/05/fast-branchless-raybounding-box-intersections/
    // Axis-aligned bounding boxes (AABBs)
    // the box is the space within the three bounding plane pairs
    Vector3d dir = info.theRay.getDir();
    Vector3d cam = info.theRay.getPos();
    double invx = 1.0/dir[0];
    double invy = 1.0/dir[1];
    double invz = 1.0/dir[2];
    
    // Left bottom is minimal, right top is maximal
    int l = size[0];
    int w = size[2];
    int h = size[1];

    Point3d lb(center[0]-l/2.0,center[1]-h/2.0,center[2]-w/2.0);
    Point3d rt(center[0]+l/2.0,center[1]+h/2.0,center[2]+w/2.0);
    
    // x boundaries
    double tx1 = (lb[0] - cam[0]) * invx;
    double tx2 = (rt[0] - cam[0]) * invx;
    // y boundaries
    double ty1 = (lb[1] - cam[1]) * invy;
    double ty2 = (rt[1] - cam[1]) * invy;
    // z boundaries
    double tz1 = (lb[2] - cam[2]) * invz;
    double tz2 = (rt[2] - cam[2]) * invz;

    // the largest minimum bounds and the smallest maximum bounds
    float tmin = max(max(min(tx1, tx2), min(ty1, ty2)), min(tz1, tz2));
    float tmax = min(min(max(tx1, tx2), max(ty1, ty2)), max(tz1, tz2));
//    cout << "tmin: " << tmin << " tmax: " << tmax << endl;

    // case 1: the ray intersects AABB but the tmax is behind the camera
    // case 2: the ray does not intersect AABB
    if (tmin > tmax) {
       // info.normal = Vector3d(0,1,0);
        //cout << info.normal << endl;
        return -1;
        }
    
    Point3d topfrontright(center[0]+l/2.0,center[1]+h/2.0,center[2]+w/2.0);
    Point3d topfrontleft(center[0]-l/2.0,center[1]+h/2.0,center[2]+w/2.0);
    Point3d topbackright(center[0]+l/2.0,center[1]+h/2.0,center[2]-w/2.0);
    Point3d topbackleft(center[0]-l/2.0,center[1]+h/2.0,center[2]+w/2.0);
    Point3d botfrontright(center[0]+l/2.0,center[1]-h/2.0,center[2]+w/2.0);
    Point3d botfrontleft(center[0]-l/2.0,center[1]-h/2.0,center[2]+w/2.0);
    Point3d botbackright(center[0]+l/2.0,center[1]-h/2.0,center[2]-w/2.0);
    Point3d botbackleft(center[0]-l/2.0,center[1]-h/2.0,center[2]-w/2.0);
//    cout << topfrontright << "/" << topfrontleft << "/" << topbackright << "/" << topbackleft << endl;
//    cout << botfrontright << "/" << botfrontleft << "/" << botbackright << "/" << botbackleft << endl;
//    
//    Vector3d topleft = Vector3d(topbackleft-topfrontleft);
//    Vector3d topback = Vector3d(topbackleft-topbackright);
//    Vector3d topright = Vector3d(topfrontright-topbackright);
//    Vector3d topfront = Vector3d(topfrontright-topfrontleft);
    Vector3d botleft = Vector3d(botfrontleft-botbackleft);
    Vector3d botback = Vector3d(botbackright-botbackleft);
    Vector3d botright = Vector3d(botbackright-botfrontright);
    Vector3d botfront = Vector3d(botfrontright-botfrontleft);
//    Vector3d leftfront = Vector3d(topfrontleft-botfrontleft);
//    Vector3d leftback = Vector3d(topbackleft-botbackleft);
    Vector3d rightfront1 = Vector3d(topfrontright-botfrontright);
    Vector3d rightfront2 = Vector3d(topfrontleft-botfrontleft);
//    Vector3d rightback = Vector3d(botbackleft-topbackleft);

    
    Vector3d frontnorm = Vector3d(topfrontleft-botfrontleft).cross(Vector3d(botfrontright-botfrontleft));
    Vector3d backnorm = Vector3d(topbackright-botbackright).cross(Vector3d(botbackleft-botbackright));
    Vector3d leftnorm = Vector3d(topbackleft-botbackleft).cross(Vector3d(botfrontleft-botbackleft));
    Vector3d rightnorm = Vector3d(topfrontright-botfrontright).cross(Vector3d(botbackright-botfrontright));
    Vector3d topnorm = Vector3d(topbackleft-topfrontleft).cross(Vector3d(topfrontright-topfrontleft));
    Vector3d botnorm = Vector3d(botfrontleft-botbackleft).cross(Vector3d(botbackright-botbackleft));
    Vector3d norms [6] = {frontnorm, leftnorm, rightnorm, topnorm, backnorm, botnorm};
//    Vector3d frontnorm = rightfront2.cross(botfront);
//    Vector3d backnorm = botfront.cross(rightfront2);
//    Vector3d leftnorm = botright.cross(rightfront1);
//    Vector3d rightnorm = rightfront1.cross(botright);
//    Vector3d topnorm = botback.cross(botleft);
//    Vector3d botnorm = botleft.cross(botback);
//    Vector3d norms [6] = {frontnorm, leftnorm, rightnorm, topnorm, backnorm, botnorm};
//    for (int n=0; n<6; n++){
//        cout << norms[n]<< endl;
//    }

    for (int j=0; j<6; j++) {
//        cout << "norm:: " << norms[j] << endl;
        double dInt = planeIntersect(info.theRay, info.iCoordinate, norms[j]);
//        cout << dInt << endl;
        Point3d i = info.theRay.getPos() + dInt*info.theRay.getDir();
        info.iCoordinate = i;
//        cout << "intersection: " << i << endl;
//        cout << "j: " << j << endl;
        Vector3d n;
        n = norms[j];
        if (n.dot(info.theRay.getDir()) < 0)
            n = -n;
        n.normalize();
        info.normal = n;
        if (true){
//        if (i[0]<=rt[0]) {
            //        if (i[0]>tx1 || i[1]>ty1 || i[2]>tz1 || i[0]<tx2) {
            //            cout << "in POS x bounds" << endl;
            info.material = this->material;
            
            info.textured = this->textured;
            //  cout << "tmin: " << tmin << endl;
            return tmin;
            //        }

        
        if (info.normal.dot(info.theRay.getDir()) < 0)
            info.entering = false;
        else info.entering = true;
        
        // If there is a single-point plane intersection, is it within the rectangular prism?
//        if (dInt != 0) {
               }
    
        // Texture mapping
        
//        if (info.textured || material->bumpMapped())
//        {
//            double x = intersectionPoint[0];
//            double y = intersectionPoint[1];
//            double z = intersectionPoint[2];
//            double phi = acos((z)/this->radius); // (-pi, pi]
//            double theta = atan2(y, x); // [0, pi]
//            phi = phi/M_PI;
//            theta = ((theta/M_PI) + 1)/2;
//            TexCoord2d coord(theta,phi);
//            intersectionInfo.texCoordinate = coord;
//            
//            if (material->bumpMapped()) {
//                // Bump Mapping
//                Vector3d up = Vector3d(0,1,0) - intersectionInfo.normal.dot(Vector3d(0,1,0)) * intersectionInfo.normal;
//                up.normalize();
//                Vector3d right = -intersectionInfo.normal.cross(up);
//                //            Vector3d right = up.cross(intersectionInfo.normal);
//                right.normalize();
//                intersectionInfo.material->bumpNormal(intersectionInfo.normal, up, right, intersectionInfo, this->bumpScale);
//            }
//        }

//}
//                // then the intersection is in the box
//                info.iCoordinate = i;
//                // entering
//            cout << norms[j].dot(info.theRay.getDir()) << endl;
//            if (norms[j].dot(info.theRay.getDir()) > 0) {
//                    cout << "ENTERING" << endl;
//                    info.normal = -norms[j];
//            }
//                else
//                    info.normal = norms[j];
//                return 1;
            }
            
            
            
           // info.normal = -info.theRay.getDir();
            
//            info.normal = Vector3d(0,1,0);
//            cout << "normal: " << info.normal<< "\n";
    

//    return tmin; // the length of the vector until intersection
//    if (tmax < tmin) {
//        cout << "TRUE" << endl;
//        return 1;
//    }
//    cout << "TMIN " << tmin <<endl;
//    return -1;
    
    return -1;
}


TexCoord2d Box::getTexCoordinates (Point3d i)
{
// skeleton delete
	TexCoord2d tCoord(0.0, 0.0);

	return tCoord;
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
