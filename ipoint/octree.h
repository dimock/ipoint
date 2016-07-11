#pragma once

#include <set>
#include <list>
#include <algorithm>
#include "rect.h"

template <class T>
class OcTree
{
  Rect3f rect_;
  int depth_;

  struct Node
  {
    Rect3f r_;

    Node(const Rect3f & r, int level) : r_(r), level_(level)
    {
    }

    bool intersect(const T & t) const
    {
      return ::intersect(r_, t);
    }

    bool intersect(const Rect3f & rc) const
    {
      return r_.intersecting(rc);
    }

    boost::shared_ptr<Node> children_[8];

    int level_;
    std::list<const T*> array_;
  };

  boost::shared_ptr<Node> root_;
  const Vec3f scale_percent_;

public:

  OcTree(Rect3f & rc, int depth) : rect_(rc), depth_(depth), scale_percent_(1.05, 1.05, 1.05)
  {
    rect_.scale( scale_percent_ );
    root_.reset( new Node(rect_, 0) );
  }

  void add(const T * t)
  {
    splitNode(root_.get(), t);
  }

  void remove(const T * t)
  {
    remove(root_.get(), t);
  }

  void collect(const Rect3f & rc, std::set<const T*> & items)
  {
    search(root_.get(), rc, items);
  }

private:

  void search(Node * node, const Rect3f & rc, std::set<const T*> & items)
  {
    if ( !node || !node->intersect(rc) )
      return;

    if ( node->level_ >= depth_ )
    {
      for (typename std::list<const T*>::iterator i = node->array_.begin(); i != node->array_.end(); ++i)
        items.insert(*i);

      return;
    }

    for (int i = 0; i < 8; ++i)
      search(node->children_[i].get(), rc, items);
  }

  void splitNode(Node * node, const T * t)
  {
    if ( !t || !node->intersect(*t) )
      return;

    if ( node->level_ >= depth_ )
    {
      node->array_.push_back(t);
      return;
    }

    for (int i = 0; i < 8; ++i)
    {
      if ( !node->children_[i].get() )
      {
        Rect3f rc = node->r_.octant(i);
        rc.scale(scale_percent_);
        node->children_[i].reset( new Node(rc, node->level_+1) );
      }

      splitNode(node->children_[i].get(), t);
    }
  }

  void remove(Node * node, const T * t)
  {
    if ( !node || !t || !node->intersect(*t) )
      return;

    if ( !node->array_.empty() )
    {
      typename std::list<const T*>::iterator iter = std::find(node->array_.begin(), node->array_.end(), t);
      if ( iter != node->array_.end() )
        node->array_.erase(iter);
      return;
    }

    for (int i = 0; i < 8; ++i)
    {
      Node * child = node->children_[i].get();
      remove(child, t);
    }
  }
};
