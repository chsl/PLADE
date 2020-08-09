#ifndef _GEOMETRY_VERTEX_GROUP_H_
#define _GEOMETRY_VERTEX_GROUP_H_

#include "geometry_common.h"
#include "../image/color.h"
#include "../basic/counted.h"
#include "../basic/smart_pointer.h"
#include "../math/math_types.h"

#include <vector>
#include <list>
#include <string>


enum VertexGroupType {
	VG_PLANE = 0,
	VG_SPHERE = 1,
	VG_CYLINDER = 2,
	VG_CONE = 3,
	VG_TORUS = 4,
	VG_GENERAL = 5
};


class PointSet;

class VertexGroup : public std::vector<int>, public Counted
{
public:
	typedef SmartPointer<VertexGroup>	Ptr;

	class Compare {
	public:
		bool operator()(const VertexGroup* g0, const VertexGroup* g1) const {
			return g0->size() > g1->size();
		}
	};

public:
	VertexGroup(VertexGroupType type) 
		: type_(type)
		, point_set_(nil)
		, parent_(nil)
		, label_("unknown")
		, visible_(true)
		, highlighted_(false)
	{
	}
	~VertexGroup() {}

	VertexGroupType type() const { return type_; }

	const std::string& label() const { return label_; }
	void set_label(const std::string& lb) { label_ = lb; }

	const Color& color() const { return color_; }
	void set_color(const Color& c) { color_ = c; }

	PointSet* point_set() { return point_set_; }
	void set_point_set(PointSet* pset) { point_set_ = pset; }

	VertexGroup* parent() { return parent_; }
	void set_parent(VertexGroup* g) { parent_ = g; }

	std::vector<VertexGroup::Ptr>& children() { return children_; }
	void set_children(const std::vector<VertexGroup::Ptr>& childr) { children_ = childr; }

	void add_child(VertexGroup* g) { 
		std::vector<VertexGroup::Ptr>::iterator pos = std::find(children_.begin(), children_.end(), g);
		if (pos == children_.end()) {
			g->set_parent(this);
			children_.push_back(g);
		}
	}

	void remove_child(VertexGroup* g) {
		std::vector<VertexGroup::Ptr>::iterator pos = std::find(children_.begin(), children_.end(), g);
		if (pos != children_.end()) {
			children_.erase(pos);
		}
	}

	bool is_visible() const  { return visible_; }
	void set_visible(bool b) { visible_ = b; }

	bool is_highlighted() const  { return highlighted_; }
	virtual void set_highlighted(bool b) { highlighted_ = b; }

private:
	VertexGroupType	type_;

	std::string		label_;
	PointSet*		point_set_;
	Color			color_;

	VertexGroup*	parent_;
	std::vector<VertexGroup::Ptr>	children_;

	bool			visible_;
	bool			highlighted_;
};

//////////////////////////////////////////////////////////////////////////

class VertexGroupPlane : public VertexGroup
{
public:
	VertexGroupPlane() : VertexGroup(VG_PLANE) { }

	void set_plane(const Plane3& plane) { plane_ = plane; }
	const Plane3& plane() const { return plane_; }

private:
	Plane3	plane_;
};


//////////////////////////////////////////////////////////////////////////


class VertexGroupCylinder : public VertexGroup
{
public:
	VertexGroupCylinder() : VertexGroup(VG_CYLINDER) {}

	// 	void set_cylinder(const Cylinder& cylinder) {...}
	// 	const Cylinder& cylinder() const { return cylinder_; }

private:
	//Cylinder	 cylinder_;
};


//////////////////////////////////////////////////////////////////////////

class VertexGroupSphere : public VertexGroup
{
public:
	VertexGroupSphere() : VertexGroup(VG_SPHERE) {}

// 	void set_sphere(const Sphere3d& sphere) { sphere_ = sphere; }
// 	const Sphere3d& sphere() const { return sphere_; }

private:
	//Sphere3d	 sphere_;
};


//////////////////////////////////////////////////////////////////////////


class VertexGroupCone : public VertexGroup
{
public:
	VertexGroupCone() : VertexGroup(VG_CONE) {}

	//void set_cone(const Cone& c) { cone_ = c; }
	//const cone& cone() const { return cone_; }

private:
	//Cone	 cone_;
};


//////////////////////////////////////////////////////////////////////////


class VertexGroupTorus : public VertexGroup
{
public:
	VertexGroupTorus() : VertexGroup(VG_TORUS) {}

	//	void set_torus(const Torus& t) { torus_ = t; }
	//	const Torus& torus() const { return torus_; }
private:
	//	Torus	 torus_;
};


//////////////////////////////////////////////////////////////////////////


class VertexGroupGeneral : public VertexGroup
{
public:
	VertexGroupGeneral() : VertexGroup(VG_GENERAL) {}
};


#endif