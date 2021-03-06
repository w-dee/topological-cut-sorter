/*
    Topological cut sort program
    Copyright (C) 2012 W.Dee

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <CGAL/Cartesian.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <boost/lexical_cast.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <string>
#include <vector>

// typedefs
typedef CGAL::Quotient<double>				Number_type;
typedef CGAL::Cartesian<Number_type>		Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel>	Traits_2;
typedef Traits_2::Point_2					Point_2;
typedef Traits_2::Curve_2					Segment_2;
typedef Traits_2::Vector_2					Vector_2;
typedef CGAL::Arrangement_2<Traits_2>		Arrangement_2;
typedef Arrangement_2::Vertex_handle		Vertex_handle;
typedef Arrangement_2::Halfedge_handle		Halfedge_handle;




// constants
static const double scale = 1.0; //!< coordinates are to be multiplied by
	//!< this number to preserve internal numeric precision.

#define WRITE_INTEGER_PLT //!< define this to write integer-only PLT files


namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

class plt_parser_t
{
	double last_x, last_y; //!< last plotter pen point
	double tmp_x; //!< temporary value for x-axis
	std::vector<Segment_2> vector; // returning vector
	bool is_pen_down;
	bool is_relative;

	void initialize() { last_x = last_y = tmp_x = 0;
		is_pen_down = false; is_relative = false; } //!< for IN command

	/**
	 * common function for commands with xy position - x
	 */
	void move_common_x(const double & val) { tmp_x = val; }

	/**
	 * common function for commands with xy position - y
	 */
	void move_common_y(const double & val)
	{
		double x = tmp_x;
		double y = val;
		if(is_relative)
			x += last_x, y += last_y;
		if(is_pen_down)
		{
			if(last_x != x || last_y != y)
			{
				vector.push_back(
					Segment_2(
						Point_2(Number_type(last_x*scale), Number_type(last_y*scale)),
						Point_2(Number_type(     x*scale), Number_type(     y*scale))));
			}
		}

		last_x = x;
		last_y = y;
	}

	void pen_down () //!< for PD command
	{
		is_pen_down = true;
	}

	void pen_up () //!< for PU command
	{
		is_pen_down = false;
	}

	void move_rel () //!< for PR command
	{
		is_relative = true;
	}

	void move_abs () //!< for PA command
	{
		is_relative = false;
	}

public:

	/**
	 * the constructor
	 */
	plt_parser_t() { initialize(); }

	/**
	 * returns resulting vector
	 */
	std::vector<Segment_2>  & get_vector() { return vector; }

	/**
	 * do parse
	 */
	bool parse(std::string::iterator first, std::string::iterator last)
	{
		using qi::lit;
		using qi::double_;
		using qi::char_;
		using qi::phrase_parse;
		using ascii::space;
		using qi::rule;

#define bind_initialize     boost::bind(&plt_parser_t::initialize,    this)
#define bind_move_common_x  boost::bind(&plt_parser_t::move_common_x, this, _1)
#define bind_move_common_y  boost::bind(&plt_parser_t::move_common_y, this, _1)
#define bind_pen_up         boost::bind(&plt_parser_t::pen_up,        this)
#define bind_pen_down       boost::bind(&plt_parser_t::pen_down,      this)
#define bind_move_rel       boost::bind(&plt_parser_t::move_rel,      this)
#define bind_move_abs       boost::bind(&plt_parser_t::move_abs,      this)


		bool r = phrase_parse(first, last, 
			*(
				// initialize commands
				lit("IN")[bind_initialize]>>(char_(';')|char_('\n')) |

				// commands with coordinates
				(
					lit("PD")[bind_pen_down]  |
					lit("PU")[bind_pen_up]    |
					lit("PR")[bind_move_rel]  |
					lit("PA")[bind_move_abs]
				) >>
					*(
						double_[bind_move_common_x]>>double_[bind_move_common_y]
					)>>(char_(';')|char_('\n')) |

				// unsupported commands
				*(char_ - (char_(';')|char_('\n'))) >> (char_(';')|char_('\n'))
			)
		, (space - char_('\n')) |char_(','));

		if (first != last)
			return false;
		return r;
	}

	/**
	 * Do parse
	 */
	bool parse(std::istream &in)
	{
		std::string str;
		while (getline(std::cin, str))
		{
			parse(str.begin(), str.end());
		}
		return true;
	}
};

#ifdef WRITE_INTEGER_PLT
	typedef long long plt_cord_type;

	static plt_cord_type plt_round(Number_type x)
	{
		double d = to_double(x); // approx.
		if(d < 0.0) return (plt_cord_type)(d - 0.5);
		return             (plt_cord_type)(d + 0.5);
	}
	#define ROUND_NUM(x) plt_round(x)
#else
	typedef Number_type plt_cord_type;
	#define ROUND_NUM(x) (x)
#endif



/**
 * Write PLT file to stdout.
 * @param		segments	reference to a vector which contains array of line segments
 */
void write_PLT(const std::vector<Segment_2>& segments)
{
	std::ostream & file = std::cout;


	plt_cord_type x = 0, y = 0;
	bool is_first = true;

	file << "IN;SP0;"; // initial commands

	for(std::vector<Segment_2>::const_iterator i = segments.begin();
		i != segments.end(); ++i)
	{
		plt_cord_type
			x1 = ROUND_NUM(i->source().x() / scale),
			y1 = ROUND_NUM(i->source().y() / scale);
		plt_cord_type
			x2 = ROUND_NUM(i->target().x() / scale),
			y2 = ROUND_NUM(i->target().y() / scale);

		if(is_first || x != x1 || y != y1) // line discontinued ?
			file << "PU" << x1 << "," << y1 << ";"; // then use PU
		
		is_first = false;
		file << "PD" << x2 << "," << y2 << ";";
		x = x2; y = y2;
	}

	file << "PU;" << std::endl; // last PU command and finish the line
}

class node_t;

/**
 * graph "edge" class.
 * This edge class has 1:1 relation to CGAL's Edge (or HaldEdge) representation.
 */
class edge_t
{
	node_t *node1; //!< node1
	node_t *node2; //!< node2

	const Arrangement_2::Halfedge & edge; //!< CGAL's line halfedge representation
	bool visited; //!< flag used in traversing the graph

public:
	/**
	 * Constructor
	 * @poram		n1		a pointer to node1
	 * @param		n2		a pointer to node2
	 * @param		e		CGAL format halfedge representation
	 */
	edge_t(node_t *n1, node_t *n2, const Arrangement_2::Halfedge & e) :
		node1(n1), node2(n2), edge(e), visited(false) {}

	bool get_visited() const { return visited; } //!< get whether this edge is already visited or not
	void set_visited(bool b) { visited = b; } //!< set edge visited flag

	node_t * get_node1() const { return node1; } //!< returns node1
	node_t * get_node2() const { return node2; } //!< returns node2

	/**
	 * Returns CGAL's Segment_2 format line segment
	 */
	Segment_2 get_segment() const
	{
		return Segment_2(edge.source()->point(), edge.target()->point());
	}

	/**
	 * returns counter-node, that is, opposite node of given 'this-node'
	 */
	node_t * get_counter_node(const node_t * this_node) const
	{
		return this_node == node1 ? node2 : node1;
	}

	/**
	 * Print debugging information
	 */
	void print()
	{
		std::cerr << "[" <<
			edge.source()->point() << "] - [" <<
			edge.target()->point() << "]";
	}
};

/**
 * Graph "node" class.
 * This node class has 1:1 relation to CGAL's Face representation.
 */
class node_t
{
	const Arrangement_2::Face & face; //!< CGAL's face representation
	std::vector<edge_t *> edges; //!< list of edges
	bool visited; //!< visited flag used in traversing


public:
	/**
	 * The constructor.
	 * @param	f	CGAL's Face
	 */
	node_t(const Arrangement_2::Face & f) : face(f),  visited(false) {}

	void insert_edge(edge_t * edge) { edges.push_back(edge); } //!< insert an edge to edge list
	const std::vector<edge_t *> & get_edges() const { return edges; }
		//!< returns edge list
	
	/**
	 * let traverse on this node.
	 * @param		segments	an array of arrays which stores traversed line segments
	 */
	void traverse(std::vector<std::vector<Segment_2> > & segments) 
	{
		if(visited) return;
		visited = true;

		typedef std::vector<edge_t *> edge_vector_t;
		typedef std::vector<node_t *> node_vector_t;
		typedef std::map<node_t *, edge_vector_t> node_edge_map_t;
		
		node_edge_map_t node_edge_map;

		// first, check unvisited edge  (and calc center point)
		Number_type cx=0, cy=0;
		int count = 0;
		for(std::vector<edge_t *>::const_iterator i = edges.begin();
			i != edges.end(); ++i)
		{
			if(!(*i)->get_visited())
			{
				(*i)->set_visited(true);

				// get counter face and make counterface - edge array map
				node_t * counter_node =(*i)->get_counter_node(this);
				node_edge_map_t::iterator mi = node_edge_map.find(counter_node);
				if(mi  == node_edge_map.end())
				{
					// new node
					edge_vector_t vec;
					vec.push_back(*i);
					node_edge_map.insert(node_edge_map_t::value_type(counter_node, vec));
				}
				else
				{
					// already there
					mi->second.push_back(*i);
				}
				count ++;
				Segment_2 seg = (*i)->get_segment();
				cx += seg.source().x() + seg.target().x();
				cy += seg.source().y() + seg.target().y();
			}
		}
		if(count)
		{
			cx /= count*2;
			cy /= count*2;
		}

		node_vector_t node_vector;

		// sort node vector by its distance ...
		// TODO optimization
		if(face.is_unbounded()) cx = 0, cy = 0; // for unbounded face, use machine initial point
		node_edge_map_t tmp = node_edge_map;
		while(tmp.size() > 0)
		{
			Number_type min_dist = -1;
			node_edge_map_t::iterator min = node_edge_map.end();
			Number_type min_cx, min_cy;
			for(node_edge_map_t::iterator mi = tmp.begin();
				mi != tmp.end(); ++mi)
			{
				Segment_2 first_segment = (*mi->second.begin())->get_segment();
				Number_type px, py, dist;
				px = first_segment.source().x();
				py = first_segment.source().y();
				dist = (px-cx)*(px-cx) + (py-cy)*(py-cy);
				if(min_dist < 0 || dist < min_dist)
				{
					min_dist = dist;
					min = mi;
					min_cx = px;
					min_cy = py;
				}
			}

			// nearest edges found
			cx = min_cx;
			cy = min_cy;
			node_vector.push_back(min->first);
			tmp.erase(min);
		}
/*
		for(node_edge_map_t::iterator mi = node_edge_map.begin();
			mi != node_edge_map.end(); ++mi)
 		{
			node_vector.push_back(mi->first);
		}
*/
		// for all unvisited edges, recurse into the opposite face.
		for(node_vector_t::iterator ni = node_vector.begin();
			ni != node_vector.end(); ++ni)
		{
			std::vector<Segment_2> vector;
			edge_vector_t & unvisited_edges = node_edge_map.find(*ni)->second;
			for(std::vector<edge_t *>::const_iterator i = unvisited_edges.begin();
				i != unvisited_edges.end(); ++i)
			{
				vector.push_back((*i)->get_segment());
			}
			(*ni)->traverse(segments);
			segments.push_back(vector);
		}
	}

};

/**
 * The Graph class
 */
class graph_t
{
	typedef std::map<const Arrangement_2::Face *, node_t *> face_node_map_t;
		//!< typedef of map class of CGAL's face and node_t

	std::vector<node_t *> nodes; //!< all node list
	std::vector<edge_t *> edges; //!< all edge list
	face_node_map_t face_node_map; //!< CGAL's face and node relation map
	const Arrangement_2::Face * unbounded_face; //!< a pointer to unbounded (most outer) face

public:

	/**
	 * the constructor
	 */
	graph_t() : unbounded_face(NULL) {}

	/**
	 * the destructor
	 */
	~graph_t()
	{
		for(std::vector<node_t *>::iterator i = nodes.begin(); i != nodes.end(); ++i)
			delete *i;
		for(std::vector<edge_t *>::iterator i = edges.begin(); i != edges.end(); ++i)
			delete *i;
	}

	/**
	 * returns unbounded (most outer) face
	 */
	const Arrangement_2::Face * get_unbouded_face() const { return unbounded_face; }

	/**
	 * ensure the CGAL's face is already in our face-node map
	 */
	node_t * ensure_node(const Arrangement_2::Face & face)
	{
		// ensure node existence in this graph
		face_node_map_t::iterator mi;
		mi = face_node_map.find(&face);
		if(mi == face_node_map.end())
		{
			// insert a map
			node_t * new_node = new node_t(face);
			face_node_map.insert(face_node_map_t::value_type(&face, new_node));
//			std::cerr << "map " << &face << " made" << std::endl;
			if(face.is_unbounded())
			{
				// unbounded face found
				unbounded_face = &face;
			}
			return new_node;
		}
		return mi->second;
	}

	/**
	 * insert a edge to the graph
	 * @param	e		CGAL's halfedge
	 */
	void insert_edge(const Arrangement_2::Halfedge & e)
	{
		// ensure both face
		Arrangement_2::Face_const_handle f1, f2;
		f1 = e.face();
		f2 = e.twin()->face();
		node_t *n1 = ensure_node(*f1);
		node_t *n2 = ensure_node(*f2);
		// make and insert an edge
		edge_t * edge = new edge_t(n1, n2, e);
		edges.push_back(edge);
		n1->insert_edge(edge);
		n2->insert_edge(edge);
//		std::cerr << "edge : ";
//		edge->print();
//		std::cerr << " has created." << std::endl;
	}

	/**
	 * start traversing from unbounded edge
	 * @segments an array of arrays which stores traversed line segments	
	 */
	void traverse(std::vector<std::vector<Segment_2> > & segments)
	{
		node_t * unbounded_node =
			ensure_node(*unbounded_face);
		unbounded_node->traverse(segments);
	}

};

/**
 * construct node-edge graph from CGAL's 2D arrangement
 * @param		graph		the graph
 * @param		arr			the CGAL's 2D arrangement
 */
static void construct_edge_graph(graph_t & graph, const Arrangement_2  & arr)
{
	Arrangement_2::Edge_const_iterator    eit;
	for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
		graph.insert_edge(*eit);
}

/**
 * evaluation function of two line segments;
 * this function returns logical prefference value of
 *   +how two line segments are close
 *   +how two line segments are continuous
 * @return		prefference value (lower is better)
 */
static double evaluate(const Segment_2 & s1, const Segment_2 & s2)
{
	// first, evaluate its distance
	double x1,y1,x2,y2;
	x1 = to_double(s1.target().x());
	y1 = to_double(s1.target().y());
	x2 = to_double(s2.source().x());
	y2 = to_double(s2.source().y());

	double dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) * 2.0;

	// next, evaluate its straightness
	x1 = to_double(s1.target().x() - s1.source().x());
	y1 = to_double(s1.target().y() - s1.source().y());
	x2 = to_double(s2.target().x() - s2.source().x());
	y2 = to_double(s2.target().y() - s2.source().y());
	
	double divider = sqrt(x1*x1 + y1*y1) * sqrt(x2*x2 + y2*y2);
	if(divider > 0.0)
		dist += 2.0 - ((x1*x2 + y1*y2) / (divider) + 1.0);
	else
		dist +=                               2.0;

	return dist;
}
 
/**
 * do simple (non-topological) cut-sort
 * @param segments      input segments
 * @param last_segment  a reference to line segment which has been inserted last
 */
static void simple_cut_sort(std::vector<Segment_2>& segments, Segment_2 &last_segment)
{
	// TODO:optimization
	typedef std::vector<Segment_2> vect_t;
	typedef std::list<Segment_2> list_t;


	// first, make a list from vector
	list_t list;
	for(vect_t::const_iterator i = segments.begin();
		i != segments.end(); ++i)
	{
		list.push_front(*i);
	}
	segments.clear();

	while(list.size())
	{
		// find nearest point from x,y
		double min_dist = -1;
		bool is_target = false;
		list_t::iterator min = list.end();
		for(list_t::iterator i = list.begin();
			i != list.end(); ++i)
		{
			double dist;
			dist = evaluate(last_segment, *i); 
			if(min_dist < 0 || dist < min_dist)
			{
				min_dist = dist;
				min = i;
				is_target = false;
			}
			dist = evaluate(last_segment, Segment_2(i->target(), i->source())); 
			if(min_dist < 0 || dist < min_dist)
			{
				min_dist = dist;
				min = i;
				is_target = true;
			}
		}

		// remove it from the list and insert into the vector
		if(is_target)
		{
			// swap target and the source
			*min = Segment_2(min->target(), min->source());
		}
		segments.push_back(*min);
		last_segment = *min;
		list.erase(min);
	}

}

/**
 * sharp point limit
 * joint degree above this is not recognized as a sharp joint
 **/
#define SHARP_JOINT_MAX 120


/**
 * returns reversed 2d segment
 */
static Segment_2 reverse_seg(const Segment_2 & seg)
{
	return Segment_2(seg.target(), seg.source());
}


/**
 * Find most sharp joint point from the list
 */
static const Segment_2 find_most_sharp_joint_point(const std::vector<Segment_2>& segments, const Segment_2 & start_point)
{
	// check segment size
	if(segments.size() < 2)
	{
		return reverse_seg(segments[0]); // cannnot make joint
	}

	typedef std::vector<Segment_2> vect_t;
	typedef CGAL::Segment_2<Kernel> st_seg_t;
	vect_t candidates;

	// convert degree to cos(degree)
	const double deg = cos((180-SHARP_JOINT_MAX) * (M_PI / 180.0) );

	// start a loop
	size_t orig_size = segments.size();
	for(size_t nth = 0; nth < orig_size; ++ nth)
	{
		size_t prev_nth = (nth == 0) ? (orig_size - 1) : (nth - 1);
		const Segment_2 & current = segments[nth];
		const Segment_2 & prev    = segments[prev_nth];
		if(prev.target() != current.source())
		{
			// discontinued path
			// discontinued path may be good point to start plotting
			candidates.push_back(current);
		}

		// check segment straightness
		st_seg_t current_st_seg(current.source(), current.target());
		st_seg_t prev_st_seg(prev.source(), prev.target());
		Vector_2 current_vec = current_st_seg.to_vector();
		Vector_2 prev_vec    = prev_st_seg.to_vector();
		double current_len = sqrt(
			to_double(current_vec.x() * current_vec.x() + current_vec.y() * current_vec.y()));
		double prev_len = sqrt(
			to_double(prev_vec.x() * prev_vec.x() + prev_vec.y() * prev_vec.y()));

		double divider = current_len * prev_len;
		if(divider <= 0.0) continue; // invalid length; should not happen
		double cos_th = to_double((current_vec.x()*prev_vec.x() +
			current_vec.y()*prev_vec.y()) / divider);

		if(cos_th <  deg)
		{
			// sharper than junction angle limit
			// make this start point
			candidates.push_back(current);
		}
	}

	if(candidates.size() == 0)
	{
		candidates.push_back(segments[0]); // push default point
	}

	// find most nearest point from start_point
	Segment_2 nearest;
	double min_len = -1;
	for(vect_t::iterator i = candidates.begin(); i != candidates.end(); ++i)
	{
		double x1,y1,x2,y2;
		x1 = to_double(i->source().x());
		y1 = to_double(i->source().y());
		x2 = to_double(start_point.target().x());
		y2 = to_double(start_point.target().y());
		double length = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
		if(min_len < 0 || min_len > length)
		{
			min_len = length;
			nearest = *i;
		}
	}

	return reverse_seg(nearest);
}


/**
 * Make some short overlap on start and end of the path
 * @param segments      input segments
 * @param length		overlap length
 * @param deg			lest angle which continues overlapping (in degree)
 */
static void make_overlap(std::vector<Segment_2>& segments, double length, double deg)
{
	// check segment size
	if(segments.size() < 3)
		return; // cannnot make closed path

	typedef std::vector<Segment_2> vect_t;
	typedef CGAL::Segment_2<Kernel> st_seg_t;

	// convert degree to cos(degree)
	deg = cos((180.0-deg) * (M_PI / 180.0) );

	// start a loop
	size_t orig_size = segments.size();
	size_t nth = 0;
	while(length > 0)
	{
		size_t prev_nth = (nth == 0) ? (orig_size - 1) : (nth - 1);
		const Segment_2 & current = segments[nth];
		const Segment_2 & prev    = segments[prev_nth];
		if(prev.target() != current.source()) return; // discontinued path
		nth ++;
		if(nth >= orig_size) nth = 0;

		// check segment straightness
		st_seg_t current_st_seg(current.source(), current.target());
		st_seg_t prev_st_seg(prev.source(), prev.target());
		Vector_2 current_vec = current_st_seg.to_vector();
		Vector_2 prev_vec    = prev_st_seg.to_vector();
		double current_len = sqrt(
			to_double(current_vec.x() * current_vec.x() + current_vec.y() * current_vec.y()));
		double prev_len = sqrt(
			to_double(prev_vec.x() * prev_vec.x() + prev_vec.y() * prev_vec.y()));

		double divider = current_len * prev_len;
		if(divider <= 0.0) return; // invalid length; should not happen
		double cos_th = to_double((current_vec.x()*prev_vec.x() +
			current_vec.y()*prev_vec.y()) / divider);

		if(cos_th < deg)
		{
			// junction angle limit exceeded; stop
			return;
		}

		// add first some line segments the the last, until the length meets the given length
		if(current_len < length)
		{
			// short enough;
			length -= current_len;
			segments.push_back(current);
		}
		else
		{
			// the line segment is longer than length; non-simple case
			Vector_2 short_vec = current_vec / current_len * length;
			segments.push_back(Segment_2(current.source(), current.source() + short_vec));
			break;
		}
	}

}


/**
 * Make gates (that is, line segments which are cut after all other line segments are cut)
 * @param segments      input segments
 * @param gate_len		gate length
 * @param interval		gate interval
 * @param gates			output gate line segments (contents are not cleared)
 */
static void make_gates(std::vector<Segment_2>& segments, double gate_len, double interval_len,
	std::vector<Segment_2>& gates)
{
	typedef std::vector<Segment_2> vect_t;
	typedef CGAL::Segment_2<Kernel> st_seg_t;

	// caluculate total length
	double total_length = 0;
	for(vect_t::iterator i = segments.begin(); i != segments.end(); i++)
	{
		// check segment length
		Segment_2 current = *i;
		st_seg_t current_st_seg(current.source(), current.target());
		Vector_2 current_vec = current_st_seg.to_vector();
		double current_len = sqrt(
			to_double(current_vec.x() * current_vec.x() + current_vec.y() * current_vec.y()));
		total_length += current_len;
	}

	// start a loop
	vect_t out_vect;
	bool is_interval = true;
	double next_point = interval_len;
	for(vect_t::iterator i = segments.begin(); i != segments.end(); i++)
	{
		// check segment length
		Segment_2 current = *i;

		for(;;)
		{
			st_seg_t current_st_seg(current.source(), current.target());
			Vector_2 current_vec = current_st_seg.to_vector();
			double current_len = sqrt(
				to_double(current_vec.x() * current_vec.x() + current_vec.y() * current_vec.y()));
			if(current_len <= 0.0 ) break;

			if(current_len > next_point)
			{
				// length exceeds the interval
				// split the line segment into some gates and its intervals
				// next split point is at next_point
				Vector_2 short_vec = current_vec / current_len * next_point;
				Segment_2 seg (current.source(), current.source() + short_vec);
				Segment_2 seg2(current.source() + short_vec, current.target());
				if(is_interval)
				{
					out_vect.push_back(seg);
				}
				else
				{
					gates.push_back(seg);
				}

				// quick hack to avoid rational numbers becoming too big;
				// any remedy ?
				current = Segment_2(
					Point_2(to_double(seg2.source().x()),
					        to_double(seg2.source().y())),
					Point_2(to_double(seg2.target().x()),
					        to_double(seg2.target().y())));

				current_len -= next_point;
				total_length -= next_point;

				if(is_interval && total_length >= gate_len)
				{
					is_interval = false;
					next_point = gate_len;
				}
				else
				{
					is_interval = true;
					next_point = interval_len;
				}
			}
			else
			{
				next_point -= current_len;
				total_length -= current_len;
				if(is_interval)
				{
					out_vect.push_back(current);
				}
				else
				{
					gates.push_back(current);
				}
				break;
			}
		}
	}
	segments = out_vect; // overwrite input back
}


/**
 * the main function
 */
int main(int argc, char *argv[])
{
	Arrangement_2   arr;
	graph_t graph;
	double overlap_length = 20.0;
	double overlap_limit_deg = 150;
	double gate_length = 120.0;
	double gate_interval = 600.0;
	bool do_overlap = false;
	bool do_gating = false;

	std::vector<Segment_2> segments;
	std::vector<std::vector<Segment_2> >  tmp_segments;

	// parse arguments
	for(int i = 1; i < argc; i++)
	{
		if(argv[i][0] == '-' && argv[i][1] == 'r') // overlap path's start and end
		{
			do_overlap = true;
		}
		else if(argv[i][0] == '-' && argv[i][1] == 'g') // do gating
		{
			do_gating = true;
		}
		else if(argv[i][0] == '-' && argv[i][1] == 'l') // specify overlap length
		{
			char *p = argv[i][2] ? argv[i] + 2 : argv[++i];
			overlap_length = atof(p);
			if(overlap_length <= 0.0)
			{
				std::cerr << "overlap length must be greater than 0" << std::endl;
				return 3;
			}
		}
		else if(argv[i][0] == '-' && argv[i][1] == 'a') // specify overlap liimt degree
		{
			char *p = argv[i][2] ? argv[i] + 2 : argv[++i];
			overlap_limit_deg = atof(p);
			if(overlap_limit_deg < 0 || overlap_limit_deg >= 180)
			{
				std::cerr << "overlap limit degree must be in 0 to 180" << std::endl;
				return 3;
			} 
		}
		else
		{
			std::cout <<
				"Topological cut-sorter (GPL3) (C) W.Dee" << std::endl <<
				" takes an HPGL input from stdin and outputs an HPGL to stdout." << std::endl <<
				"Options:" << std::endl <<
				"  -r         : Overlap closed path start and end." << std::endl <<
				"  -l  <val>  : Overlap length as used in -r option " << std::endl <<
				"               in HPGL unit, normally 1/1016 inch." << std::endl <<
				"  -a  <val>  : Overlap limit angle as used in -r option " << std::endl <<
				"               in degree. Overlapping will stop where" << std::endl <<
				"               path line segment junction angle is smaller than this limit." << std::endl <<
				"               The default is 150.0." << std::endl <<
				"  -g         : Do gating." << std::endl;
			return 3;
		}
	}


	// load PLT
	plt_parser_t parser;
	parser.parse(std::cin);
	segments.operator = (parser.get_vector());
	if(segments.size() == 0)
	{
		std::cerr << "No line segments found." << std::endl;
		return 3;
	}

	// build 2D arrangement
	CGAL::insert (arr, segments.begin(), segments.end());

	// construct node-edge graph
	construct_edge_graph(graph, arr);

	// traverse the graph with topological order
	graph.traverse(tmp_segments);

	// do simple cut-sort
	Segment_2 last_segment(Point_2(0,0) , Point_2(1,1));
	for(std::vector<std::vector<Segment_2> > ::iterator i = tmp_segments.begin();
		i != tmp_segments.end(); ++i)
	{
		simple_cut_sort(*i, last_segment);
	}

	// some line segments are reverse in direction due to the algorithm above,
	// check them and make good. 
	for(std::vector<std::vector<Segment_2> > ::iterator ti = tmp_segments.begin();
		ti != tmp_segments.end(); ++ti)
	{
		std::vector<Segment_2> & segments = *ti;
		for(std::vector<Segment_2>::iterator i = segments.begin();
			i!= segments.end(); ++i)
		{
			// check reversed segements
			
			// get previous and next line segments
			if(i == segments.begin() && ti == tmp_segments.begin()) continue;
			std::vector<Segment_2>::iterator prev = 
				(i == segments.begin()) ? ((ti-1)->end()-1)  : (i-1);
			if(i == segments.end()-1  && ti == tmp_segments.end()-1 ) continue;
			std::vector<Segment_2>::iterator next =
				(i == segments.end()-1 ) ? ((ti+1)->begin()) : (i+1);

			// check continuity
			bool continued = prev->target() == i->source();

			// check segment direction
			if(!continued && i->source() == next->source())
				*i = Segment_2(i->target(), i->source());
		}
	}

	// find sharper joint as plotting start
	for(std::vector<std::vector<Segment_2> > ::iterator i = tmp_segments.begin();
		i != tmp_segments.end(); ++i)
	{
		last_segment = find_most_sharp_joint_point(*i, last_segment);
		simple_cut_sort(*i, last_segment);
	}


	// do overlap
	if(do_overlap)
	{
		for(std::vector<std::vector<Segment_2> > ::iterator i = tmp_segments.begin();
			i != tmp_segments.end(); ++i)
		{
			make_overlap(*i, overlap_length, overlap_limit_deg);
		}
	}

	// do gating
	if(do_gating)
	{
		std::vector<Segment_2> gates;
		for(std::vector<std::vector<Segment_2> > ::iterator i = tmp_segments.begin();
			i != tmp_segments.end(); ++i)
		{
			make_gates(*i, gate_length, gate_interval, gates);
		}
		tmp_segments.push_back(gates);
	}

	// see if first cut line segment is only a single line (not a path, nor a curve)
	if(tmp_segments.size() > 0 && tmp_segments[0].size() == 1)
	{
		// insert dummy point at its center
		Segment_2 current = tmp_segments[0][0];
		tmp_segments[0].clear();
		Point_2 center((current.target().x() + current.source().x()) / 2,
		               (current.target().y() + current.source().y()) / 2);
 		tmp_segments[0].push_back(Segment_2(current.source(), center));
 		tmp_segments[0].push_back(Segment_2(center, current.target()));
 	}

	// flatten segments
	segments.clear();
	for(std::vector<std::vector<Segment_2> > ::iterator i = tmp_segments.begin();
		i != tmp_segments.end(); ++i)
	{
		for(std::vector<Segment_2>::iterator j = i->begin(); j != i->end(); ++j)
			segments.push_back(*j);
	}

	// write PLT out
	write_PLT(segments);

	return (0);
}



