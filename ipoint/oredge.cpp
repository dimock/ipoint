#include "oredge.h"

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
bool OrEdge::rotate()
{
  OrEdge * adj = get_adjacent();
  if ( !adj )
    return false;

  OrEdge * rnext = next();
  OrEdge * rprev = prev();
  OrEdge * lnext = adj->next();
  OrEdge * lprev = adj->prev();

  // verify topology before rotation
  //DO_VERIFY( verifyTopology(std::set<const OrEdge*>()) );

  const OrEdge * conn = findConnection();
  if ( conn )
    return false;

  // rotate this 90 deg CW
  this->set_next(rprev);
  rprev->set_next(lnext);
  lnext->set_next(this);

  // rotate adjacent 90 deg CW
  adj->set_next(lprev);
  lprev->set_next(rnext);
  rnext->set_next(get_adjacent());

  int org0 = this->org();
  int dst0 = this->dst();

  this->org_ = lprev->org();
  this->dst_ = rprev->org();

  adj->org_ = this->dst();
  adj->dst_ = this->org();

  THROW_IF( org() == dst(), "bad topology" );

  // verify topology after
  //DO_VERIFY( verifyTopology(std::set<const OrEdge*>()) );

  return true;
}

const OrEdge * OrEdge::findConnection() const
{
  const OrEdge * adj = get_adjacent();
  if ( !adj )
    return 0;

  const OrEdge * rnext = next();
  const OrEdge * rprev = prev();
  const OrEdge * lnext = adj->next();
  const OrEdge * lprev = adj->prev();

  // verify topology
  THROW_IF( !rnext || !rprev || !lnext || !lprev, "bad topology" );
  THROW_IF( rprev->next() != this || lprev->next() != adj, "bad topology" );
  THROW_IF( rnext->next() != rprev || lnext->next() != lprev, "bad topology" );
  THROW_IF( lprev->org() == rprev->org(), "bad topology" );
  THROW_IF( lprev->org() != lnext->dst() || rprev->org() != rnext->dst(), "bad topology" );

  int idx0 = rnext->dst();
  int idx1 = lnext->dst();

  const OrEdge * conn = 0;

  bool stop = false;
  const OrEdge * curr = rnext;

  for ( ; curr; )
  {
    curr = curr->get_adjacent();
    if ( !curr )
      break;

    THROW_IF( !curr->next(), "bad topology" );

    curr = curr->next()->next();

    THROW_IF( !curr, "bad topology" );
    
    if ( curr == rnext )
    {
      stop = true;
      break;
    }

    if ( curr->org() == idx1 && curr->dst() == idx0 )
    {
      conn = curr;
      break;
    }
  }

  if ( stop || conn )
    return conn;

  curr = rprev;

  for ( ; curr; )
  {
    curr = curr->get_adjacent();
    if ( !curr )
      break;

    THROW_IF( !curr->next(), "bad topology" );

    curr = curr->next();

    if ( curr == rprev )
    {
      stop = true;
      break;
    }

    if ( curr->org() == idx0 && curr->dst() == idx1 )
    {
      conn = curr;
      break;
    }
  }

  return conn;
}

void OrEdge::verifyTopology(std::set<const OrEdge*> & verified) const
{
  if ( verified.find(this) != verified.end() )
    return;

  verified.insert(this);

  THROW_IF( org() == dst(), "bad topology");

  const OrEdge * rprev = prev();
  const OrEdge * rnext = next();

  THROW_IF( !rprev || !rnext, "bad topology" );
  THROW_IF( !rnext->next() || rnext->next() != rprev, "bad topology" );
  THROW_IF( rnext->dst() != rprev->org(), "bad topology" );

  const OrEdge * adj = get_adjacent();
  if ( adj )
  {
    const OrEdge * lprev = adj->prev();
    const OrEdge * lnext = adj->next();

    THROW_IF( !lprev || !lnext, "bad topology" );
    THROW_IF( lprev->org() == rprev->org(), "bad topology" );

    adj->verifyTopology(verified);
  }

  rnext->verifyTopology(verified);
  rprev->verifyTopology(verified);
}

OrEdge * OrEdge::next()
{
  return next_;
}

OrEdge * OrEdge::prev()
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

const OrEdge * OrEdge::next() const
{
  return next_;
}

const OrEdge * OrEdge::prev() const
{
  const OrEdge * prev = next();
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

  //DO_VERIFY( verifyTopology(std::set<const OrEdge*>()) );

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

  //DO_VERIFY( verifyTopology(std::set<const OrEdge*>()) );

  return true;
}

bool OrEdge::splitEdge(int i)
{
  if ( !get_adjacent() )
    return false;

  OrEdge * rprev = prev();
  OrEdge * rnext = next();

  if ( !rprev || !rnext || rprev != rnext->next() )
    throw std::runtime_error("bad topology");

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
    throw std::runtime_error("bad topology");

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
  return Triangle(org(), next()->dst(), dst());
}

double OrEdge::length() const
{
  return (container_->verts().at(org()).p() - container_->verts().at(dst()).p()).length();
}

Vec3f OrEdge::dir() const
{
  Vec3f r = container_->verts().at(org()).p() - container_->verts().at(dst()).p();
  r.normalize();
  return r;
}

Rect3f OrEdge::rect() const
{
  Rect3f rc;
  rc.add(container_->verts().at(org()).p());
  rc.add(container_->verts().at(dst()).p());
  return rc;
}

//////////////////////////////////////////////////////////////////////////
OrEdge * EdgesContainer::new_edge(int o, int d)
{
  OrEdge_shared edge(new OrEdge(o, d, this));
  edges_.push_back(edge);
  return edge.get();
}
