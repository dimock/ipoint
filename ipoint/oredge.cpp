#include "oredge.h"
#include "delaunay.h"

OrEdge::OrEdge(EdgesContainer * container) :
  org_(-1), dst_(-1), container_(container), next_(0), adjacent_(0)
{
}

OrEdge::OrEdge(int o, int d, EdgesContainer * container) :
  org_(o), dst_(d), container_(container), next_(0), adjacent_(0)
{
}

OrEdge * OrEdge::create_adjacent()
{
  if ( !adjacent_ )
  {
    adjacent_ = container_->new_edge(dst(), org());
    adjacent_->adjacent_ = this;
  }

  return adjacent_;
}

OrEdge * OrEdge::get_adjacent()
{
  return adjacent_;
}

const OrEdge * OrEdge::get_adjacent() const
{
  return adjacent_;
}

void OrEdge::clear_adjacent()
{
  if ( adjacent_ )
    adjacent_->adjacent_ = 0;

  adjacent_ = 0;
}

// topology
void OrEdge::rotate()
{
  if ( !get_adjacent() )
    return;

  OrEdge * rnext = next();
  OrEdge * rprev = prev();
  OrEdge * lnext = get_adjacent()->next();
  OrEdge * lprev = get_adjacent()->prev();

  // verify topology
  if ( !rnext || !rprev || !lnext || !lprev )
    return;

  if ( rprev->next() != this || lprev->next() != get_adjacent() )
    return;

  if ( rnext->next() != rprev || lnext->next() != lprev )
    return;

  // rotate this 90 deg CW
  this->set_next(rprev);
  rprev->set_next(lnext);
  lnext->set_next(this);

  // rotate adjacent 90 deg CW
  get_adjacent()->set_next(lprev);
  lprev->set_next(rnext);
  rnext->set_next(get_adjacent());

  this->org_ = lprev->org();
  this->dst_ = rprev->org();

  get_adjacent()->org_ = this->dst();
  get_adjacent()->dst_ = this->org();
}

OrEdge * OrEdge::next() const
{
  return next_;
}

OrEdge * OrEdge::prev() const
{
  OrEdge * prev = next();
  while ( prev )
  {
    if ( prev->dst() == org() )
      break;

    prev = prev->next();
  }
  return prev;
}

OrEdge * OrEdge::set_next(OrEdge * e)
{
  OrEdge * next = next_;
  next_ = e;
  return next;
}

bool OrEdge::splitTri(int i)
{
  OrEdge * rnext = next();
  OrEdge * rprev = prev();

  if ( !rnext || !rprev || rnext->next() != rprev )
    return false;

  const Vec3f & p0 = container_->points().at(org());
  const Vec3f & p1 = container_->points().at(dst());
  const Vec3f & p2 = container_->points().at(rnext->dst());

  const Vec3f & p = container_->points().at(i);
  if ( !iMath::inside_tri(p0, p1, p2, p) )
    return false;

  OrEdge * a1 = container_->new_edge(dst(), i);
  OrEdge * b1 = container_->new_edge(i, org());
  
  set_next(a1);
  a1->set_next(b1);
  b1->set_next(this);

  OrEdge * a2 = container_->new_edge(rnext->dst(), i);
  OrEdge * b2 = a1->create_adjacent();

  rnext->set_next(a2);
  a2->set_next(b2);
  b2->set_next(rnext);

  OrEdge * a3 = b1->create_adjacent();
  OrEdge * b3 = a2->create_adjacent();

  rprev->set_next(a3);
  a3->set_next(b3);
  b3->set_next(rprev);

  return true;
}

bool OrEdge::splitEdge(int i)
{
  if ( !get_adjacent() )
    return false;

  OrEdge * rprev = prev();
  OrEdge * rnext = next();

  if ( !rprev || !rnext || rprev != rnext->next() )
    return false;

  int p1 = rnext->dst();

  OrEdge * a1 = container_->new_edge(p1, i);
  OrEdge * b1 = a1->create_adjacent();
  OrEdge * c1 = container_->new_edge(org(), i);

  a1->set_next(this);
  rnext->set_next(a1);

  b1->set_next(rprev);
  rprev->set_next(c1);
  c1->set_next(b1);

  this->org_ = i;

  OrEdge * lprev = get_adjacent()->prev();
  OrEdge * lnext = get_adjacent()->next();

  if ( !lprev || !lnext || lprev != lnext->next() )
    return false;

  int p2 = lnext->dst();

  OrEdge * a2 = container_->new_edge(i, p2);
  OrEdge * b2 = a2->create_adjacent();
  OrEdge * c2 = c1->create_adjacent();

  a2->set_next(lprev);
  get_adjacent()->set_next(a2);

  b2->set_next(c2);
  c2->set_next(lnext);
  lnext->set_next(b2);

  get_adjacent()->dst_ = i;

  return true;
}

Triangle OrEdge::tri() const
{
  return Triangle(org(), dst(), next()->dst());
}

// comparision
bool OrEdge::operator < (const OrEdge & other) const
{
  return org() < other.org() || org() == other.org() && dst() < other.dst();
}

bool OrEdge::operator == (const OrEdge & other) const
{
  return org() == other.org() && dst() == other.dst();
}

bool OrEdge::touches(const OrEdge & other) const
{
  return org() == other.org() || org() == other.dst() || dst() == other.org() || dst() == other.dst();
}

// geometry
Rect3f OrEdge::rect() const
{
  Rect3f rect;
  rect.add(container_->points().at(org()));
  rect.add(container_->points().at(dst()));
  return rect;
}

double OrEdge::length() const
{
  return (container_->points().at(org()) - container_->points().at(dst())).length();
}

Vec3f OrEdge::dir() const
{
  Vec3f r = container_->points().at(org()) - container_->points().at(dst());
  r.norm();
  return r;
}

// math
bool OrEdge::intersect(const Rect3f & r) const
{
  return rect().intersecting(r);
}

bool OrEdge::isectEdge(const Vec3f & p0, const Vec3f & p1, Vec3f & r, double & dist) const
{
  return iMath::edges_isect(container_->points().at(org()), container_->points().at(dst()), p0, p1, r, dist);
}

bool OrEdge::isectEdge(const OrEdge & other, Vec3f & r, double & dist) const
{
  return isectEdge( other.container_->points().at(other.org()), other.container_->points().at(other.dst()), r, dist);
}

bool OrEdge::needRotate(const Vec3f & cw, double threshold) const
{
  if ( !get_adjacent() )
    return false;

  const Vec3f & po = container_->points().at(org());
  const Vec3f & pd = container_->points().at(dst());

  const Vec3f & pr = container_->points().at(next()->dst());
  const Vec3f & pl = container_->points().at(get_adjacent()->next()->dst());

  bool outside;
  Vec3f dist_r = iMath::dist_to_line(po, pd, pr, outside);
  if ( dist_r.length() < threshold )
    return false;

  Vec3f dist_l = iMath::dist_to_line(po, pd, pl, outside);
  if ( dist_l.length() < threshold )
    return false;

  Vec3f r1 = -next()->dir();
  Vec3f r2 = prev()->dir();

  Vec3f r3 = -get_adjacent()->next()->dir();
  Vec3f r4 = get_adjacent()->prev()->dir();

  Vec3f x1 = r1^r4;
  Vec3f x2 = r3^r2;

  if ( x1*cw <= 0 || x2*cw <= 0 )
    return false;

  double sa, ca;
  iMath::sincos(r1, r2, sa, ca);

  double sb, cb;
  iMath::sincos(r3, r4, sb, cb);

  double dln = sa*cb + sb*ca;
  return dln < 0;
}

//////////////////////////////////////////////////////////////////////////
OrEdge * EdgesContainer::new_edge(int o, int d)
{
  OrEdge_shared edge(new OrEdge(o, d, this));
  edges_.push_back(edge);
  return edge.get();
}
