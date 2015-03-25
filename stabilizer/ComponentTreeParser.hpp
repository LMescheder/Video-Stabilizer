#ifndef COMPONENTTREEPARSER_HPP_
#define COMPONENTTREEPARSER_HPP_

#include <vector>
#include <stack>
#include <queue>
#include <utility>
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

template <typename G, typename A>
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

    ComponentTreeParser () = default;

    Result operator() (const Data& data) {
        return parse_(data);
    }

    // implementation
    private:

    struct ComponentStack {
        public:
        ComponentStack (Analyzer& analyzer) : analyzer_(analyzer), components_() {
            push_component(GraphAccessor::inf);
        }

        void push_component(Value level) {
            components_.push_back(analyzer_.add_component(level));
        }
        void push_node(NodeIndex node) {
            analyzer_.add_node(node, components_.back());
        }

        void raise_level(Value level) {
            // level of last component
            auto current_level = components_.rbegin()[0].level();
            while (level > current_level) {
                 // level of second last component (exists, since current_level < inf)
                auto next_level = components_.rbegin()[1].level();
                if (level < next_level) {
                    components_.back().set_level(level);
                    current_level = level;
                } else {
                    auto current_component = components_.back();
                    components_.pop_back();
                    auto next_component = components_.back();
                    components_.pop_back();
                    next_level = components_.back().level();
                    components_.push_back(analyzer_.merge_components(current_component, next_component));
                    current_level = components_.back().level();
                }
            }
        }

        private:
        std::vector<Component> components_;
        Analyzer& analyzer_;
    };

    struct NodePriorityLess {
        GraphAccessor& graph;

        bool operator() (const NodeIndex& node1, const NodeIndex& node2) {
            return (graph.value(node1) > graph.value(node2));
        }
    };

    // actual algorithm
    Result parse_(const Data& data) {
        // data structures
        auto graph = GraphAccessor(data);
        auto analyzer = Analyzer{};
        auto component_stack = ComponentStack{analyzer};
        auto boundary_nodes = std::priority_queue<NodeIndex, std::vector<NodeIndex>, NodePriorityLess>{NodePriorityLess{graph}};

        // initialize
        boundary_nodes.push(graph.get_source());
        bool flowingdown_phase = true;
        //auto current_node = graph.get_source();

        // we are done, when there is no boundary node left
        while (!boundary_nodes.empty()) {
            // get next node
            auto current_node = boundary_nodes.top();
            boundary_nodes.pop();

            /*
            std::cout << "Current node: " << current_node
                      << " Value = " << static_cast<int>(graph.value(current_node)) << std::endl;
            */

            component_stack.raise_level(graph.value(current_node));

            // explore neighborhood of current node
            // the accessor has to make sure, that we access every node only once

            boost::optional<NodeIndex> neighbor_or_none;
            while (neighbor_or_none = graph.get_next_neighbor(current_node)) {
                auto neighbor_node = *neighbor_or_none;
                // flow (further) down?
                if (graph.value(neighbor_node) < graph.value(current_node)) {
                    flowingdown_phase = true;
                    boundary_nodes.push(current_node);
                    current_node = neighbor_node;
                } else {
                    boundary_nodes.push(neighbor_node);
                }
            }

            // new minimum found?
            if (flowingdown_phase)
                component_stack.push_component(graph.value(current_node));

            flowingdown_phase = false;
            component_stack.push_node(current_node);

        }
        return analyzer.get_result();
    }

};

#endif // COMPONENTTREEPARSER_HPP_
