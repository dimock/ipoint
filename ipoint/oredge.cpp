#include "oredge.h"
#include "delaunay.h"

OrEdge::OrEdge(DelanayTriangulator * container) :
  org_(-1), dst_(-1), container_(container), next_(0), adjacent_(0)
{
}

OrEdge::OrEdge(int o, int d, DelanayTriangulator * container) :
  org_(o), dst_(d), container_(container), next_(0), adjacent_(0)
{
  rect_.add((container_->points_)[o]);
  rect_.add((container_->points_)[d]);

  Vec3f dim  = rect_.dimension();
  double delta = (dim.x + dim.y + dim.z) * 0.001;
  rect_.inflate( Vec3f(delta, delta, delta) );
  length_ = dim.length();
}

OrEdge * OrEdge::adjacent()
{
  if ( !adjacent_ )
  {
    adjacent_ = container_->newOrEdge(dst_, org_);
    adjacent_->adjacent_ = this;
  }

  return adjacent_;
}

void OrEdge::clear_adjacent()
{
  adjacent_ = 0;
}

// topology
void OrEdge::rotate()
{
  if ( !adjacent_ )
    return;

  OrEdge * rnext = next();
  OrEdge * rprev = prev();
  OrEdge * lnext = adjacent()->next();
  OrEdge * lprev = adjacent()->prev();

  // verify topology
  if ( !rnext || !rprev || !lnext || !lprev )
    return;

  if ( rprev->next() != this || lprev != adjacent() )
    return;

  if ( rnext->next() != rprev || lnext->next() != lprev )
    return;

  // rotate this 90 deg CW
  this->set_next(rprev);
  rprev->set_next(lnext);
  lnext->set_next(this);

  // rotate adjacent 90 deg CW
  adjacent_->set_next(lprev);
  lprev->set_next(rnext);
  rnext->set_next(adjacent_);
}

OrEdge * OrEdge::next() const
{
  return next_;
}

OrEdge * OrEdge::prev() const
{
  OrEdge * prev = next_;
  while ( prev )
  {
    if ( prev->dst_ == org_ )
      break;

    prev = prev->next_;
  }
  return prev;
}

OrEdge * OrEdge::set_next(OrEdge * e)
{
  OrEdge * next = next_;
  next_ = e;
  return next;
}

bool OrEdge::split(int i)
{
  OrEdge * rnext = next();
  OrEdge * rprev = prev();

  if ( !rnext || !rprev || rnext->next() != rprev )
    return false;

  const Vec3f & p0 = container_->points_[org_];
  const Vec3f & p1 = container_->points_[dst_];
  const Vec3f & p2 = container_->points_[rnext->dst()];

  const Vec3f & p = container_->points_[i];
  if ( !iMath::inside_tri(p0, p1, p2, p) )
    return false;

  {
    OrEdge * a = container_->newOrEdge(dst_, i);
    OrEdge * b = container_->newOrEdge(i, org_);
    
    set_next(a);
    a->set_next(b);
    b->set_next(this);
  }

  {
    OrEdge * a = container_->newOrEdge(rnext->dst(), i);
    OrEdge * b = next()->adjacent();

    rnext->set_next(a);
    a->set_next(b);
    b->set_next(rnext);
  }

  {
    OrEdge * a = prev()->adjacent();
    OrEdge * b = rnext->next()->adjacent();

    rprev->set_next(a);
    a->set_next(b);
    b->set_next(rprev);
  }

  return true;
}

Triangle OrEdge::tri() const
{
  return Triangle(org_, dst_, next()->dst());
}

// comparision
bool OrEdge::operator < (const OrEdge & other) const
{
  return org_ < other.org_ || org_ == other.org_ && dst_ < other.dst_;
}

bool OrEdge::operator == (const OrEdge & other) const
{
  return org_ == other.org_ && dst_ == other.dst_;
}

bool OrEdge::touches(const OrEdge & other) const
{
  return org_ == other.org_ || org_ == other.dst_ || dst_ == other.org_ || dst_ == other.dst_;
}

// geometry
const Rect3f & OrEdge::rect() const
{
  return rect_;
}

double OrEdge::length() const
{
  return length_;
}

// math
bool OrEdge::intersect(const Rect3f & r) const
{
  return rect_.intersecting(r);
}

bool OrEdge::isectEdge(const Vec3f & p0, const Vec3f & p1, Vec3f & r, double & dist) const
{
  return iMath::edges_isect((container_->points_)[org_], (container_->points_)[dst_], p0, p1, r, dist);
}

bool OrEdge::isectEdge(const OrEdge & other, Vec3f & r, double & dist) const
{
  return isectEdge((other.container_->points_)[other.org_], (other.container_->points_)[other.dst_], r, dist);
}
