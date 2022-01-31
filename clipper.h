/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  21 November 2020                                                *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2020                                         *
* Purpose   :  Polygon 'clipping' (boolean operations on polygons)             *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*******************************************************************************/

#ifndef clipper_h
#define clipper_h

#define CLIPPER_VERSION "10.0.0"

#include <float.h>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <stdexcept>
#include <vector>

#include "clipper_core.hpp"

namespace clipperlib {

static double const PI = 3.141592653589793238;

struct Scanline;
struct IntersectNode;
struct Active;
struct Vertex;
struct LocalMinima;
struct OutRec;

struct OutPt {
	PointI pt;
	OutPt *next = NULL;
	OutPt *prev = NULL;
	OutRec *outrec = NULL;  //used in descendant classes
};

enum class OutRecState { Undefined, Open, Outer, OuterCheck, Inner, InnerCheck };

template <typename T>
class PolyTree;
using PolyTreeI = PolyTree<int64_t>;
using PolyTreeD = PolyTree<double>;

//OutRec: contains a path in the clipping solution. Edges in the AEL will
//have OutRec pointers assigned when they form part of the clipping solution.
struct OutRec {
	size_t idx;
	OutRec *owner;
	Active *front_e;
	Active *back_e;
	OutPt *pts;
	PolyTreeI *PolyTree;
	OutRecState state;
};

// Active ----------------------------------------------------------------------

//Active: an edge in the AEL that may or may not be 'hot' (part of the clip solution).
struct Active {
	OutPt *op = NULL;  //used in descendant classes
	PointI bot;
	PointI top;
	int64_t curr_x = 0;  //current (updated at every new scanline)
	double dx = 0.0;
	int wind_dx = 1;  //1 or -1 depending on winding direction
	int wind_cnt = 0;
	int wind_cnt2 = 0;  //winding count of the opposite polytype
	OutRec *outrec = NULL;
	//AEL: 'active edge list' (Vatti's AET - active edge table)
	//     a linked list of all edges (from left to right) that are present
	//     (or 'active') within the current scanbeam (a horizontal 'beam' that
	//     sweeps from bottom to top over the paths in the clipping operation).
	Active *prev_in_ael = NULL;
	Active *next_in_ael = NULL;
	//SEL: 'sorted edge list' (Vatti's ST - sorted table)
	//     linked list used when sorting edges into their new positions at the
	//     top of scanbeams, but also (re)used to process horizontals.
	Active *prev_in_sel = NULL;
	Active *next_in_sel = NULL;
	Active *jump = NULL;
	Vertex *vertex_top = NULL;
	LocalMinima *local_min = NULL;  //the bottom of an edge 'bound' (also Vatti)
};

// Clipper ---------------------------------------------------------------------

class Clipper {
private:
	typedef std::vector<OutRec *> OutRecList;
	typedef std::vector<IntersectNode *> IntersectList;
	typedef std::priority_queue<int64_t> ScanlineList;
	typedef std::vector<LocalMinima *> MinimaList;
	typedef std::vector<Vertex *> VertexList;

	int64_t bot_y_ = 0;
	double scale_ = 1.0;
	bool has_open_paths_ = false;
	bool minima_list_sorted_ = false;
	ClipType cliptype_ = ClipType::None;
	FillRule fillrule_ = FillRule::EvenOdd;
	Active *actives_ = NULL;
	Active *sel_ = NULL;
	MinimaList minima_list_;
	MinimaList::iterator curr_loc_min_;
	IntersectList intersect_list_;
	VertexList vertex_list_;
	ScanlineList scanline_list_;

	void Reset();
	void InsertScanline(int64_t y);
	bool PopScanline(int64_t &y);
	bool PopLocalMinima(int64_t y, LocalMinima *&local_minima);
	void DisposeAllOutRecs();
	void DisposeVerticesAndLocalMinima();
	void AddLocMin(Vertex &vert, PathType polytype, bool is_open);
	void AddPathToVertexList(const PathI &p, PathType polytype, bool is_open);
	bool IsContributingClosed(const Active &e) const;
	inline bool IsContributingOpen(const Active &e) const;
	void SetWindCountForClosedPathEdge(Active &edge);
	void SetWindCountForOpenPathEdge(Active &e);
	virtual void InsertLocalMinimaIntoAEL(int64_t bot_y);
	void InsertLeftEdge(Active &e);
	inline void PushHorz(Active &e);
	inline bool PopHorz(Active *&e);
	inline void StartOpenPath(Active &e, const PointI pt);
	inline void UpdateEdgeIntoAEL(Active *e);
	virtual void IntersectEdges(Active &e1, Active &e2, 
    const PointI pt, bool orientation_check_required = false);
	inline void DeleteFromAEL(Active &e);
	inline void AdjustCurrXAndCopyToSEL(const int64_t top_y);
	void DoIntersections(const int64_t top_y);
	void DisposeIntersectNodes();
	void AddNewIntersectNode(Active &e1, Active &e2, const int64_t top_y);
	bool BuildIntersectList(const int64_t top_y);
	void ProcessIntersectList();
	void SwapPositionsInAEL(Active &edge1, Active &edge2);
	OutPt *AddOutPt(const Active &e, const PointI pt);
	void AddLocalMinPoly(Active &e1, Active &e2, const PointI pt, 
    bool is_new = false, bool orientation_check_required = false);
	void AddLocalMaxPoly(Active &e1, Active &e2, const PointI pt);
	void DoHorizontal(Active &horz);
	bool ResetHorzDirection(const Active &horz, Active *max_pair,
    int64_t &horz_left, int64_t &horz_right);
	void DoTopOfScanbeam(const int64_t top_y);
	Active *DoMaxima(Active &e);
	virtual OutPt *CreateOutPt();
	virtual OutRec *CreateOutRec();
	void JoinOutrecPaths(Active &e1, Active &e2);

protected:
	OutRecList outrec_list_;
	void CleanUp();  //unlike Clear, CleanUp preserves added paths
	virtual void ExecuteInternal(ClipType ct, FillRule ft);
	bool BuildResultI(PathsI &closed_paths, PathsI *open_paths);
	bool BuildResultTreeI(PolyTreeI &polytree, PathsI *open_paths);

public:
	Clipper(){};
	virtual ~Clipper();
	void Clear();
	virtual RectI GetBounds();

	//ADDPATH & ADDPATHS METHODS ...
	virtual void AddPath(const PathI &path, 
    PathType polytype = PathType::Subject, bool is_open = false);
	virtual void AddPaths(const PathsI &paths, 
    PathType polytype = PathType::Subject, bool is_open = false);
	//EXECUTE METHODS ...
	virtual bool Execute(ClipType clip_type, 
    FillRule fill_rule, PathsI &closed_paths);
	virtual bool Execute(ClipType clip_type, 
    FillRule fill_rule, PathsI &closed_paths, PathsI &open_paths);
	virtual bool Execute(ClipType clip_type, 
    FillRule fill_rule, PolyTreeI &poly_tree, PathsI &open_paths);
};

// Note: ClipperD: is a wrapper for ClipperI.
// Since all clipping is done using integer polygon coordinates to maintain
// robustness, floating point coords must be converted to and from integer
// values. Also, scaling is usually required to achieve a desired precision.

class ClipperD : public Clipper {
private:
	double scale_ = 100.0;

protected:
	bool BuildResultD(PathsD &closed_paths, PathsD *open_paths);
	bool BuildResultTreeD(PolyTreeD &polytree, PathsD *open_paths);

public:
	explicit ClipperD(double scale = 100);
	//ADDPATH & ADDPATHS METHODS ...
	virtual void AddPath(const PathD &path,
    PathType polytype = PathType::Subject, bool is_open = false);
	virtual void AddPaths(const PathsD &paths,
    PathType polytype = PathType::Subject, bool is_open = false);
	//EXECUTE METHODS ...
	bool Execute(ClipType clip_type, 
    FillRule fill_rule, PathsD &closed_paths);
	bool Execute(ClipType clip_type, 
    FillRule fill_rule, PathsD &closed_paths, PathsD &open_paths);
	bool Execute(ClipType clip_type, 
    FillRule fill_rule, PolyTreeD &poly_tree, PathsD &open_paths);
};

// PolyTree --------------------------------------------------------------------

//PolyTree: is intended as a READ-ONLY data structure for CLOSED paths returned
//by clipping operations. While this structure is more complex than the
//alternative Paths structure, it does preserve path 'ownership' - ie those
//paths that contain (or own) other paths. This will be useful to some users.

template <typename T>
class PolyTree {
private:
	double scale_;
	Path<T> path_;

protected:
	PolyTree<T> *parent_;
	std::vector<PolyTree *> childs_;

public:
	explicit PolyTree(double scale = 1.0) {  //only for root node
		scale_ = scale;
		parent_ = nullptr;
	}
	PolyTree(PolyTree<T>& parent, const clipperlib::Path<T>& path) {
		this->parent_ = &parent;
		this->scale_ = parent.scale_;
		this->path_.Append(path);
		parent.childs_.push_back(this);
	}
	virtual ~PolyTree() { Clear(); };
	void Clear() {
		for (auto child : childs_) {
			child->Clear();
			delete child;
		}
		childs_.resize(0);
	}
	size_t ChildCount() const { return childs_.size(); }
	PolyTree<T>& Child(size_t index) {
		if (index >= childs_.size())
			throw ClipperLibException("invalid range in PolyTree::GetChild.");
		return *childs_[index];
	}
	const PolyTree<T> *Parent() const { return parent_; };
	const Path<T> &GetPath() const { return path_; };
	bool IsHole() const {
		PolyTree* pp = parent_;
		bool is_hole = pp;
		while (pp) {
			is_hole = !is_hole;
			pp = pp->parent_;
		}
		return is_hole;
	}
	void SetScale(double scale) {
		scale_ = scale;
		for (auto child : childs_)
			child->SetScale(scale);
	}
};

//------------------------------------------------------------------------------

}  // namespace clipperlib

#endif  //clipper_h
