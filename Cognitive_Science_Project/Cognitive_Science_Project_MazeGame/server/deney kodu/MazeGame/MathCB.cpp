/**************************************************************************
	Developed by: Chih-Hao Ho and Cagatay Basdogan
	Purpose: Calculate the force for the Phantom.
	Date: Apr. 22, 1997
	Laboratory for Human and Machine Haptics
	Massachusetts Institute of Technology
 **************************************************************************/
//INCLUDE FILES
#include "Public.h"
#include "MathCB.h"
#include "Main.h"

double DistanceBetweenPointAndLine(Point pt,Point pt0,Point pt1,Point &nearPt);
double DistanceBetweenPointAndPolygon(Point pt,Point pt0,Point pt1,
				      Point pt2,Vector normal,Point &nearPt);
SBoolean edge_intersects_triangle(Point p1,Point p2, Point v1,
				  Point v2,Point v3,Vector normal,
				  double d,Point &contactPoint);
SBoolean point_is_within_triangle(Point p,Point v1,Point v2,
				  Point v3,Vector normal);
SBoolean triangles_intersect(Point v1, Point v2, Point v3,
			     Vector normal1, double d_tri1,
			     Point v4, Point v5, Point v6,
			     Vector normal2, double d_tri2);
void calc_polygon_normal(Point pt1, Point pt2, Point pt3, Point normal);
SBoolean triangle_intersects_polyhedron(PolyhedronStruct* bone, SbVec3f p1,
					SbVec3f p2, SbVec3f p3);
double smoothstep(double r);



double DistanceBetweenPointAndLine(Point pt,Point pt0,Point pt1,Point &nearPt)
{
  Vector a=pt-pt0;
  Vector b=pt1-pt0;
  double dot,mag,ans;
  mag = b.length();
  if (mag == 0)
    {
      nearPt = pt0;
      return a.length();
    }
  dot = a.dot(b)/mag;
  if (dot<=0){
    nearPt = pt0;
    ans = a.length();
  }
  else if (dot >= mag){
    nearPt = pt1;
    a = pt-pt1;
    ans = a.length();
  }
  else
    {
      nearPt = pt0+dot*b/mag;
      a = pt - nearPt;
      ans = a.length();
    }
  return ans;
};

double DistanceBetweenPointAndPolygon(Point pt,Point pt0,Point pt1,
				      Point pt2,Vector normal,Point &nearPt)
{
  Vector tempVector;
  Point tempPoint;
  double temp,temp1;
  tempVector = pt-pt0;
  temp = tempVector.dot(normal);
  tempPoint = pt - temp*normal;
  if (point_is_within_triangle(tempPoint,pt0,pt1,pt2,normal)==yes)
    {
      nearPt = tempPoint;
      if (temp<0) temp= -temp;
      return temp;
    }
  else
    {
      double l0,l1,l2;
      tempVector = tempPoint - pt0;
      l0 = tempVector.length();
      tempVector = tempPoint - pt1;
      l1 = tempVector.length();
      tempVector = tempPoint - pt2;
      l2 = tempVector.length();
      if (l0 > l1){
	if (l1>l2){nearPt = pt2;tempVector = pt - pt2;}
	else{nearPt = pt1;tempVector = pt - pt1;}
      }
      else {
	if (l0>l2){nearPt = pt2;tempVector = pt - pt2;}
	else{nearPt = pt0;tempVector = pt - pt0;}
      }
      temp = tempVector.length();
      temp1=DistanceBetweenPointAndLine(pt,pt0,pt1,tempPoint);
      if (temp1<temp) {temp=temp1;nearPt = tempPoint;}
      temp1=DistanceBetweenPointAndLine(pt,pt1,pt2,tempPoint);
      if (temp1<temp) {temp=temp1;nearPt = tempPoint;}
      temp1=DistanceBetweenPointAndLine(pt,pt0,pt2,tempPoint);
      if (temp1<temp) {temp=temp1;nearPt = tempPoint;}
      return temp;
    }
};


SBoolean edge_intersects_triangle(Point p1,Point p2, Point v1,
				  Point v2,Point v3,Vector normal,
				  double d,Point &contactPoint)
{
  double dist1,dist2,percent;
  Point inter_pt;
  SBoolean answer = no;
  
  dist1 = normal.dot(p1)+d;
  dist2 = normal.dot(p2)+d;

  if((!(EQUAL_WITHIN_ERROR2(dist1,0.0)))&&(!(EQUAL_WITHIN_ERROR2(dist2,0.0))))
    {
      if(SSIGN(dist1) == SSIGN(dist2))
	return(no);
    }


  // if you made it to here, then the edge intersects the plane, though
  // not necessarily the triangle. Next, find the intersection
  // point so that you can check to see if it lies within the triangle
  //tmp = dist1/(dist2-dist1);
  //percent = SABS(tmp);
  percent = -dist1/(dist2-dist1);
  inter_pt = p1 + percent*(p2-p1);
  
  answer = point_is_within_triangle(inter_pt,v1,v2,v3,normal);
  if(answer == yes)
    {
      contactPoint = inter_pt;
      return(yes);
    }
  else
    {
      return(no);
    }
};


SBoolean point_is_within_triangle(Point pt,Point p1,Point p2,
				  Point p3,Vector normal)
{
  int sign1,sign2,sign3;
  Vector vec1,vec2,vec3,cross_vec;
  double dot;

  //form a vector from p to each of the vertices, and then
  //use each pair of vectors to form three cross products. When you dot
  //each of these cross products with the polygon normal, you will
  //get either a positive or a negative number. If p lies within polygon,
  //then all the cross products will point in the same direction
  //(and thus their dot products with the normal vector will have the same
  //sign) If the dot products don't all have the same sign, then p is not
  //within the polygon
	
  vec1=p1-pt;
  vec2=p2-pt;
  vec3=p3-pt;

  cross_vec=vec1.cross(vec2);
  dot = cross_vec.dot(normal);
  if(EQUAL_WITHIN_ERROR2(dot,0.0))
		return(yes);
  else
    sign1 = SSIGN(dot);
  
  cross_vec=vec2.cross(vec3);
  dot = cross_vec.dot(normal);
  if(EQUAL_WITHIN_ERROR2(dot,0.0))
    return(yes);
  else
    sign2 = SSIGN(dot);
  
  cross_vec=vec3.cross(vec1);
  dot = cross_vec.dot(normal);
  if(EQUAL_WITHIN_ERROR2(dot,0.0))
    return(yes);
  else
    sign3 = SSIGN(dot);
  
  if((sign1 == sign2)&&(sign2 == sign3))
    {
      return(yes);
    }
  else
    {
      return(no);
    }
};


SBoolean triangles_intersect(Point v1, Point v2, Point v3,
			     Vector normal1, double d_tri1,
			     Point v4, Point v5, Point v6,
			     Vector normal2, double d_tri2)
{

	Point tempContact1;
	Point tempContact2;
	Point tempContact3;
	Point tempContact4;
	Point tempContact5;
	Point tempContact6;


   /* The triangles intersect if any one of the edges intersects
    * the other triangle.
    */


   if (edge_intersects_triangle(v4,v5,v1,v2,v3,normal1,d_tri1,tempContact1) == yes)
      return (yes);

   if (edge_intersects_triangle(v5,v6,v1,v2,v3,normal1,d_tri1,tempContact2) == yes)
      return (yes);

   if (edge_intersects_triangle(v6,v4,v1,v2,v3,normal1,d_tri1,tempContact3) == yes)
      return (yes);

   if (edge_intersects_triangle(v1,v2,v4,v5,v6,normal2,d_tri2,tempContact4) == yes)
      return (yes);

   if (edge_intersects_triangle(v2,v3,v4,v5,v6,normal2,d_tri2,tempContact5) == yes)
      return (yes);

   if (edge_intersects_triangle(v3,v1,v4,v5,v6,normal2,d_tri2,tempContact6) == yes)
      return (yes);

   return (no);

};


void calc_polygon_normal(Point pt1, Point pt2, Point pt3, Point normal)
{

   Point vec1, vec2;
   float mag;

   vec1[0] = pt2[0] - pt1[0];
   vec1[1] = pt2[1] - pt1[1];
   vec1[2] = pt2[2] - pt1[2];

   vec2[0] = pt3[0] - pt1[0];
   vec2[1] = pt3[1] - pt1[1];
   vec2[2] = pt3[2] - pt1[2];

   normal = vec1.cross(vec2);
   mag = normal.normalize();

};

SBoolean triangle_intersects_polyhedron(PolyhedronStruct* bone, SbVec3f p1,
					SbVec3f p2, SbVec3f p3)
{

   int i, v1, v2, v3, num_intersections = 0;
   Point pt1,pt2,pt3;
   Vector mynormal;
   double my_d;

   pt1[0] = p1[0];
   pt1[1] = p1[1];
   pt1[2] = p1[2];
   pt2[0] = p2[0];
   pt2[1] = p2[1];
   pt2[2] = p2[2];
   pt3[0] = p3[0];
   pt3[1] = p3[1];
   pt3[2] = p3[2];

   /* First check for intersection of the polyhedra's bounding cubes */

   if (pt1[0] < bone->bc.x1 && pt2[0] < bone->bc.x1 && pt3[0] < bone->bc.x1)
      return (no);
   if (pt1[0] > bone->bc.x2 && pt2[0] > bone->bc.x2 && pt3[0] > bone->bc.x2)
      return (no);

   if (pt1[1] < bone->bc.y1 && pt2[1] < bone->bc.y1 && pt3[1] < bone->bc.y1)
      return (no);
   if (pt1[1] > bone->bc.y2 && pt2[1] > bone->bc.y2 && pt3[1] > bone->bc.y2)
      return (no);

   if (pt1[2] < bone->bc.z1 && pt2[2] < bone->bc.z1 && pt3[2] < bone->bc.z1)
      return (no);
   if (pt1[2] > bone->bc.z2 && pt2[2] > bone->bc.z2 && pt3[2] > bone->bc.z2)
      return (no);

   // calculate the normal of the single triangle

   calc_polygon_normal(pt1,pt2,pt3,mynormal);

   // shortest distance to triangle
   // take any of the vertices of the polygon and find the 
   // dot product with the normal

   my_d = -pt1[0]*mynormal[0] - pt1[1]*mynormal[1] - pt1[2]*mynormal[2];

   // If you make it to here, you'll be checking for intersection
   // between the triangle and each triangle in the polyhedron.
   // So compute the normal and the d value for the triangle.

   for (i=0; i<bone->GetNumberOfPolygon(); i++)
   {
      // Check for intersection of the bounding cubes

      if (pt1[0] < bone->polygon[i].bc.x1 && pt2[0] < bone->polygon[i].bc.x1 && pt3[0] < bone->polygon[i].bc.x1)
	 continue;
      if (pt1[0] > bone->polygon[i].bc.x2 && pt2[0] > bone->polygon[i].bc.x2 && pt3[0] > bone->polygon[i].bc.x2)
	 continue;

      if (pt1[1] < bone->polygon[i].bc.y1 && pt2[1] < bone->polygon[i].bc.y1 && pt3[1] < bone->polygon[i].bc.y1) 
	 continue;
      if (pt1[1] > bone->polygon[i].bc.y2 && pt2[1] > bone->polygon[i].bc.y2 && pt3[1] > bone->polygon[i].bc.y2) 
	 continue;

      if (pt1[2] < bone->polygon[i].bc.z1 && pt2[2] < bone->polygon[i].bc.z1 && pt3[2] < bone->polygon[i].bc.z1)
	 continue;
      if (pt1[2] > bone->polygon[i].bc.z2 && pt2[2] > bone->polygon[i].bc.z2 && pt3[2] > bone->polygon[i].bc.z2)
	 continue;

      // Now see if the line segment intersects the triangle

      v1 = bone->polygon[i].vertex_index[0];
      v2 = bone->polygon[i].vertex_index[1];
      v3 = bone->polygon[i].vertex_index[2];

      if (triangles_intersect(pt1,pt2,pt3,mynormal,my_d,
			      bone->vertex[v1].coord,
			      bone->vertex[v2].coord,
			      bone->vertex[v3].coord,
			      bone->polygon[i].normal,bone->polygon[i].d) == yes)
	 return (yes);
   }

   return (no);

};

double smoothstep(double r)
{
	float LOWER_LIMIT = 20.0;
	float UPPER_LIMIT = 40.0;

	if(r < LOWER_LIMIT)
		return 1;
	if(r >= UPPER_LIMIT)
		return 0;
	r = (r - LOWER_LIMIT)/(UPPER_LIMIT - LOWER_LIMIT); // normalize to [0:1]
	return (1.0 - (3.0*r*r - 2.0*r*r*r));

};

