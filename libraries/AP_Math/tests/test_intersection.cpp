#include <AP_gtest.h>
#include <AP_Math/intersection.h>

TEST(Intersection,RaySegmentIntersection1)
{
  Vector2f p(0,0);
  Vector2f r(0,1);
  Vector2f q(-1,1);
  Vector2f s(1,0);
  Vector2f intersect;
  bool found = intersection(p,r,q,s,intersect);
  EXPECT_TRUE(found);
  Vector2f result(0,1);
  EXPECT_EQ(intersect,result);
}

TEST(Intersection,RaySegmentIntersection2)
{
  Vector2f p(0.5,0.5);
  Vector2f r(0.532,0.532);
  Vector2f q(-1,1);
  Vector2f s(2,0);
  Vector2f intersect;
  bool found = intersection(p,r,q,s,intersect);
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
  bool found = intersection(p,r,q,s,intersect);
  EXPECT_FALSE(found);
}

TEST(Intersection,RaySegmentIntersection4)
{
  Vector2f p(0,0);
  Vector2f r(1,0.5);
  Vector2f q(0,3);
  Vector2f s(3,-3);
  Vector2f intersect;
  bool found = intersection(p,r,q,s,intersect);
  EXPECT_TRUE(found);
  Vector2f result(2,1);
  EXPECT_EQ(intersect,result);
}

AP_GTEST_MAIN()
