#include "rayfile.h"
#include "image.h"
#include "camera.h"
#include "group.h"
#include "main.h"
#include "material.h"
#include "light.h"
#include "shape.h"
#include <vector>


/*
* method to cast rays through each pixel of image into the scene.
* uses getColor to find the color of an intersection point for each 
* pixel.  makes pretty pictures.
*/
void RayFile::raytrace (Image* image)
{

	// these will be useful
	int imageWidth = image->getWidth();
	int imageHeight = image->getHeight();
    
    // RAY_CASTING TODO
    Camera* cam = getCamera();
    Vector3d cam_up = cam->getUp();
    Vector3d cam_right = cam->getRight();
    Vector3d cam_dir = cam->getDir();
    Point3d cameraPos = cam->getPos();
    double cameraHalfAngleRad = cam->getHalfHeightAngle()/180 * M_PI;
    
    // Read camera info and compute distance D to view window and the coordinates of its center and top left
    double distance = (0.5 * imageHeight)/tan(cameraHalfAngleRad) ;
    Point3d center = cameraPos + cam_dir * distance;
    Point3d topLeft = center + cam_up * 0.5 * imageHeight - cam_right * 0.5 * imageWidth;
    
	// for printing progress to stderr...
	double nextMilestone = 1.0;

	// 
	// ray trace the scene
	//

	for (int j = 0; j < imageHeight; ++j)
	{
		for (int i = 0; i<imageWidth; ++i)
		{
            if (i==imageWidth/2 && j==imageHeight/2) {
                cout << "stop";
            }

            // Compute the ray to trace
            Rayd theRay;
            Point3d currentPoint = topLeft - cam_up * (j + 0.5) + cam_right * (i + 0.5);
    
            // RAY_CASTING TODO
            // Compute and set the starting poisition and direction of the ray through pixel i,j
            // HINT: be sure to normalize the direction vector
            // RAY_CASTING TODO
            Vector3d currentDir = Vector3d(cameraPos, currentPoint);
            theRay.setPos(cameraPos);
            currentDir = currentDir.getUnit();
            theRay.setDir(currentDir);
            
			// get the color at the closest intersection point

			Color3d theColor = getColor(theRay, options->recursiveDepth);
            theColor.clampTo(0.0, 1.0);
			// the image class doesn't know about color3d so we have to convert to pixel
			// update pixel
			Pixel p;

			p.r = theColor[0];
			p.g = theColor[1];
			p.b = theColor[2];

			image->setPixel(i, j, p);

		} // end for-i

		// update display 
		// you don't need to touch this part!

		if (options->progressive)
		{

				display();
		}
		else if (!options->quiet)
		{
			if (((double) j / (double) imageHeight) <= nextMilestone)
			{
				cerr << "." << flush;
				nextMilestone -= (1.0 / 79.0);
			}
		}
	} // end for-j
}


/* 
* get the color of the scene with respect to theRay
*/


Color3d RayFile::getColor(Rayd theRay, int rDepth)
{
	// some useful constants
	Color3d white(1,1,1);
	Color3d black(0,0,0);

	// check for intersections
	Intersection intersectionInfo;
	intersectionInfo.theRay=theRay;
	double dist = root->intersect(intersectionInfo);

	if (dist <=EPSILON)
		return background;

	// intersection found so compute color
	Color3d color;

    // add emissive term
    Color3d emissive = intersectionInfo.material->getEmissive();
    color += emissive;
	// add ambient term
    Color3d ambient = intersectionInfo.material->getAmbient();
    color += ambient;
    
    // add contribution from each light
    for (VECTOR(Light*)::iterator theLight = lights.begin(); theLight != lights.end(); ++theLight)
    {
        if (!((*theLight)->getShadow(intersectionInfo, root))) {
            color += (*theLight)->getDiffuse(intersectionInfo) ;
            color += (*theLight)->getSpecular(intersectionInfo);


        }
    }


	// stop if no more recursion required
	if (rDepth == 0)
		return color; // done


	// stop if we are already at white
	color.clampTo(0,1);
	if (color==white) // can't add any more
		return color;

	// recursive step
    Rayd reflNewRay;
    Vector3d normal = intersectionInfo.normal;
    if (intersectionInfo.theRay.getDir().dot(normal) > 0)
        normal *= -1;
  //  normal.normalize();
    
//    intersectionInfo.normal = normal; // FIXME: do we need this?
    
    // Compute reflected ray according to Snell's law
    reflNewRay.setDir(intersectionInfo.theRay.getDir() + 2 * (- intersectionInfo.theRay.getDir()).dot(normal) * normal);
    // Offset the starting point of the ray a little
    reflNewRay.setPos(intersectionInfo.iCoordinate + EPSILON * normal);

    Color3d reflectionColor = getColor(reflNewRay, rDepth -1);
    for (int c = 0; c < 3; c++)
        color[c] += (intersectionInfo.material->getSpecular())[c] * reflectionColor[c];

	// transmission
    if (intersectionInfo.material->getKtrans() != 0) {
        Rayd transNewRay;
        // Compute transmitted ray according to Snell's law
        transNewRay.setDir(intersectionInfo.theRay.getDir().getUnit());
        // Offset the starting point of the ray a little
        transNewRay.setPos(intersectionInfo.iCoordinate - EPSILON * normal);
        // Calculate the index of refraction of the inner material
        
        double beta;
        double refind = intersectionInfo.material->getRefind();
        if (intersectionInfo.entering)
            beta = refind;
        else
            beta = 1/refind;
        
        bool doTrans = false;
        
        Vector3d v = intersectionInfo.theRay.getDir().getUnit();
        double cosThetaIn = (-1 * normal).dot(v);
        double sinThetaIn = sqrt( 1 - pow(cosThetaIn, 2));
        double sinThetaOut = beta * sinThetaIn;
        double cosThetaOut = sqrt(1 - pow(sinThetaOut, 2));
        Vector3d v_trans;
        if (beta* sinThetaIn == 0) {
            doTrans = true;
            v_trans = v;
        }
        else if ((beta* sinThetaIn > 0) and (beta * sinThetaIn < 1)) {
            doTrans = true;
            Vector3d v_s = (v - (cosThetaIn * (-1 * normal)))/ (sinThetaIn);
            v_trans = cosThetaOut * (-1 * normal) + sinThetaOut * (v_s.getUnit());
        }
        if (doTrans) {
            v_trans.normalize();
            transNewRay.setDir(v_trans);
            Color3d transColor = getColor(transNewRay, rDepth -1);
            for (int c = 0; c < 3; c++)
                color[c] += (intersectionInfo.material->getSpecular())[c] * transColor[c] * intersectionInfo.material->getKtrans();
        }
        color.clampTo(0, 1);
    }
    
	// compute transmitted ray using snell's law
    
    // ---------------
    // |            |
    // |  /      \  |
    // |  -      -  |
    // |      3     |
    // |            |
    // ---------------

	return color;
}

