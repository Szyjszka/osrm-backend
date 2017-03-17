#ifndef OSRM_PARTITION_MULTI_LEVEL_GRAPH_HPP
#define OSRM_PARTITION_MULTI_LEVEL_GRAPH_HPP

#include "partition/multi_level_partition.hpp"

#include "util/static_graph.hpp"

#include <boost/iterator/permutation_iterator.hpp>

namespace osrm
{
namespace partition
{
template <typename EdgeDataT, bool UseSharedMemory> class MultiLevelGraph;

namespace io
{
template <typename EdgeDataT, bool UseSharedMemory>
void read(const boost::filesystem::path &path, MultiLevelGraph<EdgeDataT, UseSharedMemory> &graph);

template <typename EdgeDataT, bool UseSharedMemory>
void write(const boost::filesystem::path &path,
           const MultiLevelGraph<EdgeDataT, UseSharedMemory> &graph);
}

template <typename EdgeDataT, bool UseSharedMemory>
class MultiLevelGraph : public util::StaticGraph<EdgeDataT, UseSharedMemory>
{
  private:
    using SuperT = util::StaticGraph<EdgeDataT, UseSharedMemory>;
    template <typename T> using Vector = typename util::ShM<T, UseSharedMemory>::vector;

  public:
    MultiLevelGraph() = default;

    MultiLevelGraph(Vector<typename SuperT::NodeArrayEntry> node_array_,
                    Vector<typename SuperT::EdgeArrayEntry> edge_array_,
                    Vector<LevelID> edge_to_level_)
        : SuperT(std::move(node_array_), std::move(edge_array_)),
          edge_to_level(std::move(edge_to_level_))
    {
    }

    template <typename ContainerT>
    MultiLevelGraph(const MultiLevelPartition &mlp,
                    const std::uint32_t num_nodes,
                    const ContainerT &edges)
    {
        auto highest_border_level = GetHighestBorderLevel(mlp, edges);
        auto permutation = SortEdgesByHighestLevel(highest_border_level, edges);
        SuperT::InitializeFromSortedEdgeRange(
            num_nodes,
            boost::make_permutation_iterator(edges.begin(), permutation.begin()),
            boost::make_permutation_iterator(edges.begin(), permutation.end()));

        // if the node ordering is sorting the border nodes first,
        // the id of the maximum border edge will be rather low as they will
        // also only be in the front part of the edge array
        auto max_border_edge_id = 0u;
        std::for_each(permutation.begin(),
                      permutation.end(),
                      [&highest_border_level, &max_border_edge_id](auto idx) {
                          if (highest_border_level[idx] > 0)
                              max_border_edge_id = std::max(idx, max_border_edge_id);
                      });
        BOOST_ASSERT(max_border_edge_id < permutation.size());

        const auto upper_bound_border_edges = max_border_edge_id + 1;
        edge_to_level.resize(upper_bound_border_edges);
        std::copy(
            boost::make_permutation_iterator(highest_border_level.begin(), permutation.begin()),
            boost::make_permutation_iterator(highest_border_level.begin(),
                                             permutation.begin() + upper_bound_border_edges),
            edge_to_level.begin());
    }

    // Fast scan over all relevant border edges since we can terminate if we know they
    // are sorted descending by level on which they are relevant
    template <typename Func>
    void ForAllBorderEdges(const LevelID level, const NodeID node, Func func) const
    {
        for (auto edge : SuperT::GetAdjacentEdgeRange(node))
        {
            if (!IsBorderEdge(level, edge))
                return;

            func(edge);
        }
    }

    // Fast scan over all relevant internal edges since we can terminate if we know they
    // are sorted descending by level on which they are relevant
    template <typename Func>
    void ForAllInternalEdges(const LevelID level, const NodeID node, Func func) const
    {
        for (auto edge : boost::adaptors::reverse(SuperT::GetAdjacentEdgeRange(node)))
        {
            if (!IsInternalEdge(level, edge))
                return;

            func(edge);
        }
    }

    bool IsBorderEdge(LevelID level, EdgeID edge) const
    {
        return edge < edge_to_level.size() && edge_to_level[edge] <= level;
    }

    bool IsInternalEdge(LevelID level, EdgeID edge) const
    {
        return edge < edge_to_level.size() || edge_to_level[edge] > level;
    }

  private:
    template <typename ContainerT>
    auto GetHighestBorderLevel(const MultiLevelPartition &mlp, const ContainerT &edges) const
    {
        std::vector<LevelID> highest_border_level(edges.size());
        std::transform(
            edges.begin(), edges.end(), highest_border_level.begin(), [&mlp](const auto &edge) {
                return mlp.GetHighestDifferentLevel(edge.source, edge.target);
            });
        return highest_border_level;
    }

    template <typename ContainerT>
    auto SortEdgesByHighestLevel(const std::vector<LevelID> &highest_border_level,
                                 const ContainerT &edges) const
    {
        std::vector<std::uint32_t> permutation(edges.size());
        std::iota(permutation.begin(), permutation.end(), 0);
        std::sort(permutation.begin(),
                  permutation.end(),
                  [&edges, &highest_border_level](const auto &lhs, const auto &rhs) {
                      // sort by source node and then by level in _decreasing_ order
                      return std::tie(edges[lhs].source, highest_border_level[rhs]) <
                             std::tie(edges[rhs].source, highest_border_level[lhs]);
                  });

        return permutation;
    }

    friend void
    io::read<EdgeDataT, UseSharedMemory>(const boost::filesystem::path &path,
                                         MultiLevelGraph<EdgeDataT, UseSharedMemory> &graph);
    friend void
    io::write<EdgeDataT, UseSharedMemory>(const boost::filesystem::path &path,
                                          const MultiLevelGraph<EdgeDataT, UseSharedMemory> &graph);

    Vector<LevelID> edge_to_level;
};
}
}

#endif
