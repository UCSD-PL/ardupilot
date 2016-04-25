#include <AP_gtest.h>
#include <AP_Math/intersection.h>

TEST(Intersection,RaySegmentIntersection1)
{
  Vector2f p(0,0);
  Vector2f r(0,1);
  Vector2f q(-1,1);
  Vector2f s(2,0);
  Vector2f intersect;
  bool found = simple_intersection(p,r,q,s,intersect);
  EXPECT_TRUE(found);
  Vector2f result(0,1);
  EXPECT_EQ(intersect,result);
}

TEST(Intersection,RaySegmentIntersection2)
{
  Vector2f p(0.5,0.5);
  Vector2f r(0.532,0.532);
  Vector2f q(-1,1);
  Vector2f s(2.1,0);
  Vector2f intersect;
  bool found = simple_intersection(p,r,q,s,intersect);
  EXPECT_TRUE(found);
  Vector2f result(1,1);
  EXPECT_EQ(intersect,result);
}

TEST(Intersection,RaySegmentIntersection3)
{
  Vector2f p(0.5,0.5);
  Vector2f r(0.532,0.532);
  Vector2f q(-1,1);
  Vector2f s(1.9,0);
  Vector2f intersect;
  bool found = simple_intersection(p,r,q,s,intersect);
  EXPECT_FALSE(found);
}

TEST(Intersection,RaySegmentIntersection4)
{
  Vector2f p(0,0);
  Vector2f r(1,0.5);
  Vector2f q(0,3);
  Vector2f s(3,-3);
  Vector2f intersect;
  bool found = simple_intersection(p,r,q,s,intersect);
  EXPECT_TRUE(found);
  Vector2f result(2,1);
  EXPECT_EQ(intersect,result);
}

TEST(Intersection,PolyRayIntersection1)
{
  Vector2f position(0,0);
  Vector2f direction(0,1);
  Vector2f poly[] = {
    Vector2f(-1, -1),
    Vector2f(1, -1),
    Vector2f(1, 1),
    Vector2f(-1, 1)
  };
  unsigned nvert = 4;
  Vector2f intersect;
  unsigned num_intersect = poly_intersection(position,direction,poly,nvert,intersect);
  EXPECT_EQ(num_intersect, 1);
  Vector2f result(0,1);
  EXPECT_EQ(intersect, result);
}

TEST(Intersection,PolyRayIntersection2)
{
  Vector2f position(0,0.456);
  Vector2f direction(0.12432,0);
  Vector2f poly[] = {
    Vector2f(-1, -1),
    Vector2f(1, -1),
    Vector2f(1, 1),
    Vector2f(-1, 1)
  };
  unsigned nvert = 4;
  Vector2f intersect;
  unsigned num_intersect = poly_intersection(position,direction,poly,nvert,intersect);
  EXPECT_EQ(num_intersect, 1);
  Vector2f result(1,0.456);
  EXPECT_EQ(intersect, result);
}

TEST(Intersection,PolyRayIntersection3)
{
  Vector2f position(1.5,0);
  Vector2f direction(1,0);
  Vector2f poly[] = {
    Vector2f(-1, -1),
    Vector2f(1, -1),
    Vector2f(1, 1),
    Vector2f(-1, 1)
  };
  unsigned nvert = 4;
  Vector2f intersect;
  unsigned num_intersect = poly_intersection(position,direction,poly,nvert,intersect);
  EXPECT_EQ(num_intersect, 0);
}

TEST(Intersection,PolyRayIntersection4)
{
  Vector2f position(0,0);
  Vector2f direction(1,0);
  Vector2f poly[] = {
    Vector2f(-1, -1),
    Vector2f(1, -1),
    Vector2f(1, 1),
    Vector2f(2, 1),
    Vector2f(2, -1),
    Vector2f(3, -1),
    Vector2f(3, 2),
    Vector2f(-1, 2)
  };
  unsigned nvert = 8;
  Vector2f intersect;
  unsigned num_intersect = poly_intersection(position,direction,poly,nvert,intersect);
  EXPECT_EQ(num_intersect, 3);
  Vector2f result(1,0);
  EXPECT_EQ(intersect, result);
}

TEST(Intersection,PolyRayIntersection5)
{
  // Ray intersects a vertex
  Vector2f position(0,0);
  Vector2f direction(-1,-1);
  Vector2f poly[] = {
    Vector2f(-1, -1),
    Vector2f(1, -1),
    Vector2f(1, 1),
    Vector2f(-1, 1)
  };
  unsigned nvert = 4;
  Vector2f intersect;
  unsigned num_intersect = poly_intersection(position,direction,poly,nvert,intersect);
  EXPECT_EQ(num_intersect, 1);
  Vector2f result(-1,-1);
  EXPECT_EQ(intersect, result);
}

TEST(Intersection,PolyRayIntersection6)
{
  // Ray is collinear with one edge
  Vector2f position(0,0);
  Vector2f direction(1,0);
  Vector2f poly[] = {
    Vector2f(-1, -1),
    Vector2f(1, -1),
    Vector2f(1, 0),
    Vector2f(2, 0),
    Vector2f(2, 1),
    Vector2f(-1, 1)
  };
  unsigned nvert = 6;
  Vector2f intersect;
  unsigned num_intersect = poly_intersection(position,direction,poly,nvert,intersect);
  EXPECT_EQ(num_intersect, 1);
  Vector2f result(2,0);
  EXPECT_EQ(intersect, result);
}

AP_GTEST_MAIN()
