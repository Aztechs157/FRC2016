package org.usfirst.frc157.FRC2016;

import java.util.List;
import java.util.ArrayList;

public class FieldMap {

	// Need to represent zones one field
	//  A zone is defined by an ordered series of points and the line segments between them
	//   e.g. a rectangular zone would be defined by 4 points and the such as (1,1)(1,3)(2,3)(2,1)
	//        and the boundary segments would be bounded by the segments:
	//        (1,1) to (1,3)
	//        (1,3) to (2,3)
	//        (2,3) to (2,1)
	//        (2,1) to (1,1)
	//   note: segments in the zone definition are not permitted to cross
	
	
	// Need to represent sensable objects/boundaries
	
	// Note: zones may or may not or may be partially bounded by sensable objects/boudaries
	
	// need methods to:
		// determine which zone a coordinate is in
		// 
	
	
	public class Point
	{
		double x,y;
	}
	
	public class Segment
	{
		Point a, b;
		public Segment(Point a, Point b)
		{
			this.a = a;
			this.b = b;
		}
	}
		
	// Given three colinear points p, q, r, the function checks if
	// point q lies on line segment 'pr'
	boolean onSegment(Point p, Point q, Point r)
	{
	    if (q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
	        q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y))
	       return true;
	 
	    return false;
	}
	 
	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are colinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise
	int orientation(Point p, Point q, Point r)
	{
	    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
	    // for details of below formula.
	    double val = (q.y - p.y) * (r.x - q.x) -
	                 (q.x - p.x) * (r.y - q.y);
	 
	    if (val == 0) return 0;  // colinear
	 
	    return (val > 0)? 1: 2; // clock or counterclock wise
	}
	 
	// The main function that returns true if line segment a and b intersect
	boolean doIntersect(Segment a, Segment b)
	{
		
		Point p1 = a.a;
		Point q1 = a.b;
		Point p2 = b.a;
		Point q2 = b.b;
	    // Find the four orientations needed for general and
	    // special cases
	    int o1 = orientation(p1, q1, p2);
	    int o2 = orientation(p1, q1, q2);
	    int o3 = orientation(p2, q2, p1);
	    int o4 = orientation(p2, q2, q1);
	 
	    // General case
	    if (o1 != o2 && o3 != o4)
	        return true;
	 
	    // Special Cases
	    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
	    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
	 
	    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
	    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
	 
	    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
	    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
	 
	     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
	    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
	 
	    return false; // Doesn't fall in any of the above cases
	}
	 
	// Driver program to test above functions
	
	public class Zone
	{
		ArrayList<Point> boundary = new ArrayList<Point>();
		
		public Zone()
		{
			boundary.clear();
		}
		public Zone(Point a, Point b, Point c, Point d)
		{
			this();
			addPoint(a);
			addPoint(b);
			addPoint(c);
			addPoint(d);
		}
		public Zone(Point a, Point b, Point c)
		{
			this();
			addPoint(a);
			addPoint(b);
			addPoint(c);			
		}
		public void addPoint(Point p)
		{
			boundary.add(p);
		}
		public boolean isZoneOK()
		{
			for(int idx = 0; idx < boundary.size(); idx ++)
			{
				for(int idy = idx; idx < boundary.size(); idy++)
				{
					Segment a = new Segment(boundary.get(idx), boundary.get(idx+1));
					Segment b = new Segment(boundary.get(idy), boundary.get(idy+1));
					
					if(doIntersect(a,b))
					{
						return false;
					}
				}
			}
			return true;
		}
		public boolean isPointInZone(Point p)
		{
			Segment t = new Segment(boundary.get(0), p);
			int crossings = 0;
			
			for(int idx = 0; idx < boundary.size(); idx ++)
			{
				Segment s = new Segment(boundary.get(idx), boundary.get(idx+1));
				
				if(doIntersect(t,s))
				{
					crossings ++;
				}				
			}					
			if(crossings % 2 == 0)
			{
				// even number of crossings implies outside
				return false;
			}
			else
			{
				// odd number of crossings imples inside
				return true;
			}
		}
	}
}
