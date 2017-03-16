#include "partition/multi_level_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <random>
#include <vector>

using namespace osrm;
using namespace osrm::partition;

namespace
{
struct MockEdge
{
    NodeID source;
    NodeID target;
};

auto makeGraph(const MultiLevelPartition &mlp, const std::vector<MockEdge> &mock_edges)
{
    struct EdgeData
    {
        bool forward;
        bool backward;
    };
    using Edge = util::static_graph_details::SortableEdgeWithData<EdgeData>;
    std::vector<Edge> edges;
    std::size_t max_id = 0;
    for (const auto &m : mock_edges)
    {
        max_id = std::max<std::size_t>(max_id, std::max(m.source, m.target));
        edges.push_back(Edge{m.source, m.target, true, false});
        edges.push_back(Edge{m.target, m.source, false, true});
    }
    std::sort(edges.begin(), edges.end());
    return MultiLevelGraph<EdgeData, false>(mlp, max_id + 1, edges);
}
}

BOOST_AUTO_TEST_SUITE(multi_level_graph)

BOOST_AUTO_TEST_CASE(check_edges_sorting)
{
    // node:                0  1  2  3  4  5  6  7  8  9 10 11
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3}};
    std::vector<CellID> l3{{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}};
    std::vector<CellID> l4{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    MultiLevelPartition mlp{{l1, l2, l3, l4}, {5, 4, 2, 1}};

    std::vector<MockEdge> edges = {
        // edges sorted into border/internal by level
        //  level:  (1) (2) (3) (4)
        {0, 1},   //  i   i   i   i
        {2, 3},   //  i   i   i   i
        {3, 7},   //  b   b   b   i
        {4, 0},   //  b   b   b   i
        {4, 5},   //  i   i   i   i
        {5, 6},   //  b   i   i   i
        {6, 4},   //  b   i   i   i
        {6, 7},   //  i   i   i   i
        {7, 11},  //  b   b   i   i
        {8, 9},   //  i   i   i   i
        {9, 8},   //  i   i   i   i
        {10, 11}, //  i   i   i   i
        {11, 10}  //  i   i   i   i
    };

    auto graph = makeGraph(mlp, edges);

    for (auto from : util::irange(0u, graph.GetNumberOfNodes()))
    {
        LevelID level = INVALID_LEVEL_ID;
        for (auto edge : graph.GetAdjacentEdgeRange(from))
        {
            auto to = graph.GetTarget(edge);
            BOOST_CHECK(mlp.GetHighestDifferentLevel(from, to) <= level);
            level = mlp.GetHighestDifferentLevel(from, to);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
