#ifndef GEOM_H
#define GEOM_H

#include <iostream>
using namespace std;
 
// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000
 
struct Point{
    double x;
    double y;
    bool clean;
};

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r);
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r);
 
// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2);
 
// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(Point polygon[], int n, Point p);

#endif