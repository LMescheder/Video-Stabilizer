#ifndef COMPONENTTREEPARSER_HPP_
#define COMPONENTTREEPARSER_HPP_

#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <boost/optional.hpp>

// declarations

/*
G must provide:
  NodeIndex
  Node
  Value // must be totally ordered by <
  Data

  static const Value inf

  G(const Data&)
  NodeIndex get_source()

  Value value (NodeIndex node);
  Node node (NodeIndex node);
  boost::optional<NodeIndex> get_next_neighbor (NodeIndex node);

A must provide:
  ComponentIndex
  Result

  void add_node(G::Node, G::Value)
  ComponentIndex add_component(G::Value)

  ComponentIndex merge_components(ComponentIndex comp1, ComponentIndex comp2) \\ 1st into 2nd

P must provide:
  void push(NodeIndex)
  boost::optional<NodeIndex> pop()

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
    using Node = typename G::Node;
    using Data = typename G::Data;

    using Analyzer = A;
    using Result = typename A::Result;
    using ComponentIndex = typename A::ComponentIndex;

    using PriorityQueue = P;

    ComponentTreeParser () = default;

    Result operator() (const Data& data) {
        auto graph = GraphAccessor(data);
        auto analyzer = Analyzer{};
        auto boundary_nodes = PriorityQueue{};
        return parse_(graph, analyzer, boundary_nodes);
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
            current_levels_.push_back(level);
        }
        void push_node(Node node, Value level) {
            analyzer_.add_node(node, level, components_.back());
        }

        void raise_level(Value level);

        std::vector<Value> current_levels_;
        Analyzer& analyzer_;
        std::vector<ComponentIndex> components_;
    };


    // actual algorithm
    Result parse_(GraphAccessor& graph, Analyzer& analyzer, PriorityQueue& boundary_nodes);

};

// definitions

template <typename G, typename A, typename P>
typename ComponentTreeParser<G,A,P>::Result ComponentTreeParser<G,A,P>::parse_(G& graph, A& analyzer, P& boundary_nodes) {
    // data structures

    auto component_stack = ComponentStack{analyzer};
    //auto boundary_nodes = std::priority_queue<NodeIndex, std::vector<NodeIndex>, NodePriorityLess>{NodePriorityLess{graph}};

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
        component_stack.push_node(graph.node(current_node), graph.value(current_node));
        if (analyzer.is_finished())
            break;
    }
    return analyzer.get_result();
}

template <typename G, typename A, typename P>
void ComponentTreeParser<G,A,P>::ComponentStack::raise_level(ComponentTreeParser<G,A,P>::Value level) {
    while (level >  current_levels_.back()) {
        // level of second last component (exists, since current_level < inf)
        auto next_level = current_levels_.rbegin()[1];
        if  (level < next_level) {
            current_levels_.back() = level;
        } else {
            analyzer_.merge_component_into(components_.rbegin()[0], components_.rbegin()[1], level);
            components_.pop_back();
            current_levels_.pop_back();
        }
    }
}

#endif // COMPONENTTREEPARSER_HPP_



