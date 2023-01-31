#include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/named_function_params.hpp>


struct VertexData {
    std::string name;
    int id;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS,
                              boost::directedS,
                              boost::no_property,
                              boost::property<boost::edge_weight_t, double>
                              > MyGraphType;

int main()
{
    int num_satellites = 5;

    MyGraphType G(num_satellites);

    std::vector<double> e_weights {0.5, 1, 2.3, 1.0, 5.0};
    std::vector<std::tuple<int, int>> edges {std::make_tuple(0, 1),
                                             std::make_tuple(1, 2),
                                             std::make_tuple(2, 3),
                                             std::make_tuple(4, 1),
                                             std::make_tuple(0, 3)};

    std::vector<boost::adjacency_list<>::edge_descriptor> edge_refs;

    int counter = 0;
    for (std::tuple<int, int> e : edges) {
        auto ef = boost::add_edge(std::get<0>(e), std::get<1>(e), e_weights.at(counter), G).first;
        edge_refs.push_back(ef);
        counter++;
    }

    // Shortest path

    // This is the generic vertex iterator (int in the case of ajacency list)
    typedef typename boost::graph_traits<MyGraphType>::vertex_descriptor vertex_descriptor;

    // Mapping for vertex properties
    typedef boost::property_map<MyGraphType, boost::vertex_index_t>::type IdMap;

    // This will store predecessor of vertices returned by shortest path algorithm
    std::vector<vertex_descriptor> pred(num_vertices(G));

    boost::iterator_property_map<std::vector<vertex_descriptor>::iterator,
                                 IdMap,
                                 vertex_descriptor,
                                 vertex_descriptor&>
    predmap(pred.begin(), get(boost::vertex_index, G));

    std::vector<int> distvector(num_vertices(G));
    boost::iterator_property_map<std::vector<int>::iterator,
                                 IdMap,
                                 int,
                                 int&>
    distmap_vect(distvector.begin(), get(boost::vertex_index, G));


    IdMap index = boost::get(boost::vertex_index, G);

    vertex_descriptor v0 = index(*vertices(G).first);
    dijkstra_shortest_paths(G, v0,
                          predecessor_map(predmap)
                          .distance_map(distmap_vect));

    // Print out predecessor
    boost::graph_traits<MyGraphType>::vertex_iterator vi;
    std::cout << "The origin is " << v0 << std::endl;
    for (vi = vertices(G).first; vi != vertices(G).second; vi++) {
        vertex_descriptor pred_v = pred.at(index(*vi));
        std::cout << "The predecessor of " << index(*vi) << " is " << pred_v << std::endl;
    }

    // Get the next hop of the origin and destination
    vertex_descriptor v_origin = index(*vertices(G).first);

    // (origin, destination, next_hop)
    std::vector<std::tuple<int, int, int>> next_hops;
    for (vi = vertices(G).first; vi != vertices(G).second; vi++) {
        vertex_descriptor pred_v = pred.at(index(*vi));
        // trace back to origin
        vertex_descriptor succ_v = pred_v;

        bool reachable = true;
        while (pred_v != v_origin) {

            // Not reachable, next_hop is -1
            if (pred_v == index(*vi)) {
                next_hops.push_back(std::make_tuple(v_origin, index(*vi), -1));
                reachable = false;
                break;
            }

            succ_v = pred_v;
            pred_v = pred.at(pred_v);
        }

        if (reachable) {
            next_hops.push_back(std::make_tuple(v_origin, index(*vi), succ_v));
        }

    }

    // Print out next hop
    std::cout << "origin, destination, next_hop" << std::endl;
    for (auto t : next_hops) {
        std::cout << std::get<0>(t) << ", " << std::get<1>(t) << ", " << std::get<2>(t) << std::endl;
    }
}
