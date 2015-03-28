#ifndef COMPONENTTREEPARSER_HPP_
#define COMPONENTTREEPARSER_HPP_

#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <boost/optional.hpp>

/*
G must provide:
  NodeIndex
  Value // must be totally ordered by <
  Data

  static const Value inf

  G(const Data&)
  NodeIndex get_source()

  Value value (NodeIndex node);
  bool has_more_neighbors (NodeIndex node);
  NodeIndex get_next_neighbor (NodeIndex node);

A must provide:
  Component
  Result

  add_node(G::NodeIndex, G::Value)

  ComponentRef add_component()
  merge_components(ComponentRef comp1, ComponentRef comp2)
*/

template <typename G, typename A, typename P>
// requires GraphAccessor<G>
//      &&  ComponentAnalyzer<A>
class ComponentTreeParser {
    // interface
    public:
    using GraphAccessor = G;
    using Value = typename G::Value;
    using NodeIndex = typename G::NodeIndex;
    using Data = typename G::Data;

    using Analyzer = A;
    using Result = typename A::Result;
    using Component = typename A::Component;

    using PriorityQueue = P;

    ComponentTreeParser () = default;

    Result operator() (const Data& data) {
        return parse_(data);
    }

    // implementation
    private:

    // contains (component, level) pairs
    struct ComponentStack {
        public:
        ComponentStack (Analyzer& analyzer) : analyzer_(analyzer), components_() {
            push_component(GraphAccessor::inf);
        }

        void push_component(Value level) {
            components_.push_back(Component{});
            values_.push_back(level);
        }
        void push_node(NodeIndex node) {
            analyzer_.add_node(node, components_.back());
        }

        void raise_level(Value level) {
            // level of last component
            while (level >  values_.back()) {
                 // level of second last component (exists, since current_level < inf)
                auto next_level = values_.rbegin()[1];
                if  (level < next_level) {
                    values_.back() = level;
                } else {
                    analyzer_.merge_component_into(components_.rbegin()[0], components_.rbegin()[1]);
                    components_.pop_back();
                    values_.pop_back();
                }
            }
        }

        std::vector<Component> components_;
        std::vector<Value> values_;
        Analyzer& analyzer_;
    };

    struct NodePriorityLess {
        GraphAccessor& graph;

        bool operator() (const NodeIndex& node1, const NodeIndex& node2) {
            return (graph.value(node1) > graph.value(node2));
        }
    };

    // actual algorithm
    Result parse_(const Data& data);

};

// definition
template <typename G, typename A, typename P>
typename ComponentTreeParser<G,A,P>::Result ComponentTreeParser<G,A,P>::parse_(const ComponentTreeParser<G,A,P>::Data &data) {
    // data structures
    auto graph = GraphAccessor(data);
    auto analyzer = Analyzer{};
    auto component_stack = ComponentStack{analyzer};
    //auto boundary_nodes = std::priority_queue<NodeIndex, std::vector<NodeIndex>, NodePriorityLess>{NodePriorityLess{graph}};

    auto boundary_nodes = PriorityQueue{};

    // initialize
    auto source_node = graph.get_source();
    boundary_nodes.push(source_node, graph.value(source_node));
    bool flowingdown_phase = true;
    //auto current_node = graph.get_source();

    // we are done, when there is no boundary node left
    while (auto current_node_or_none = boundary_nodes.pop()) {
        // get next node
        auto current_node = *current_node_or_none;
        component_stack.raise_level(graph.value(current_node));
        if (analyzer.is_finished())
            break;

        /*
            std::cout << "Current node: " << current_node
                      << " Value = " << static_cast<int>(graph.value(current_node)) << std::endl;
            */


        // explore neighborhood of current node
        // the accessor has to make sure, that we access every node only once
        while (auto neighbor_or_none = graph.get_next_neighbor(current_node)) {
            auto neighbor_node = *neighbor_or_none;
            // flow (further) down?
            if (graph.value(neighbor_node) < graph.value(current_node)) {
                flowingdown_phase = true;
                boundary_nodes.push(current_node, graph.value(current_node));
                current_node = neighbor_node;
            } else {
                boundary_nodes.push(neighbor_node, graph.value(neighbor_node));
            }
        }

        // new minimum found?
        if (flowingdown_phase) {
            component_stack.push_component(graph.value(current_node));
            flowingdown_phase = false;
        }
        component_stack.push_node(current_node);
        if (analyzer.is_finished())
            break;
    }
    return analyzer.get_result();
}

#endif // COMPONENTTREEPARSER_HPP_
