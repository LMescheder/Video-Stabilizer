#ifndef COMPONENTTREEPARSER_HPP
#define COMPONENTTREEPARSER_HPP

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

  TODO: put current levels in analyzer instead of parser
  TODO: use size hints from accessor to preallocate component stack
  TODO: better way to return analyzers results?
*/

template <typename G, typename A>
// requires GraphAccessor<G>
//      &&  ComponentAnalyzer<A>
class ComponentTreeParser {
    // interface
    public:
    using GraphAccessor = G;
    using NodeIndex = typename G::NodeIndex;
    using Node = typename G::Node;
    using Data = typename G::Data;
    using Value = typename G::Value;
    using Analyzer = A;
    using Result = typename A::Result;
    using Component = typename A::Component;

    using PriorityQueue = typename G::PriorityQueue;

    ComponentTreeParser() = default;

    Result operator() (const Data& data, bool inverted=false) {
        auto graph = GraphAccessor(data, inverted);
        auto analyzer = Analyzer{};
        auto boundary_nodes = PriorityQueue{inverted};
        return parse_(graph, analyzer, boundary_nodes);
    }

    Result operator() (const Data& data, Analyzer& analyzer, bool inverted=false) {
        auto graph = GraphAccessor(data, inverted);
        analyzer.reset();
        auto boundary_nodes = PriorityQueue{inverted};
        return parse_(graph, analyzer, boundary_nodes);
    }

    Result operator() (GraphAccessor& graph, Analyzer& analyzer, bool inverted=false) {
        graph.reset();
        analyzer.reset();
        auto boundary_nodes = PriorityQueue{inverted};
        return parse_(graph, analyzer, boundary_nodes);
    }

    Result operator() (GraphAccessor& graph, Analyzer& analyzer, PriorityQueue& boundary_nodes) {
        graph.reset();
        analyzer.reset();
        boundary_nodes.reset();
        return parse_(graph, analyzer, boundary_nodes);
    }


    // implementation
    private:

    struct ComponentStack {
        public:
        ComponentStack (const GraphAccessor& graph, Analyzer& analyzer)
            : graph_(graph), analyzer_(analyzer), components_(){
            //components_.push_back(analyzer_.add_component(graph_.inf()));
        }

        void push_component(NodeIndex node_idx, Value level) {
            assert(components_.empty() || graph_.less(level, analyzer_.get_level(components_.back())));
            components_.push_back(analyzer_.add_component(graph_.node(node_idx), level));
            //current_levels_.push_back(level);
        }
        void push_node(NodeIndex node_idx) {
            assert(analyzer_.get_level(components_.back()) == graph_.value(node_idx));
            analyzer_.add_node(graph_.node(node_idx), components_.back());
        }

        void raise_level(Value level);

       // std::vector<Value> current_levels_;
        const GraphAccessor& graph_;
        Analyzer& analyzer_;
        std::vector<Component> components_;

        Value level (unsigned int i) {
            return analyzer_.get_level(components_.rbegin()[i]);
        }

        Component& component (unsigned int i) {
            return components_.rbegin()[i];
        }
    };


    // actual algorithm
    Result parse_(GraphAccessor& graph, Analyzer& analyzer, PriorityQueue& boundary_nodes);


};

// definitions

template <typename G, typename A>
typename ComponentTreeParser<G,A>::Result ComponentTreeParser<G,A>::parse_(
    typename ComponentTreeParser<G,A>::GraphAccessor& graph,
    typename ComponentTreeParser<G,A>::Analyzer& analyzer,
    typename ComponentTreeParser<G,A>::PriorityQueue& boundary_nodes) {
    // data structures

    auto component_stack = ComponentStack{graph, analyzer};
    //auto boundary_nodes = std::priority_queue<NodeIndex, std::vector<NodeIndex>, NodePriorityLess>{NodePriorityLess{graph}};

    // initialize
    auto current_node = graph.get_source();
    bool flowingdown_phase = true;
    //auto current_node = graph.get_source();

    // we are done, when there is no boundary node left
    while (true) {
        // get next node
        /*
        std::cout << "Current node: " << current_node
                  << " Value = " << static_cast<int>(graph.value(current_node)) << std::endl;
        */

        // explore neighborhood of current node
        // the accessor has to make sure, that we access every node only once
        while (auto neighbor_or_none = graph.get_next_neighbor(current_node)) {
            auto neighbor_node = *neighbor_or_none;
            // flow (further) down?
            if (graph.less(graph.value(neighbor_node), graph.value(current_node))) {
                flowingdown_phase = true;
                boundary_nodes.push(current_node, graph.value(current_node));
                current_node = neighbor_node;
            } else {
                boundary_nodes.push(neighbor_node, graph.value(neighbor_node));
            }
        }

        // all neighbors already added

        // new minimum found?
        if (flowingdown_phase) {
            component_stack.push_component(current_node, graph.value(current_node));
            flowingdown_phase = false;
        } else {
            component_stack.push_node(current_node);
        }

        auto current_node_or_none = boundary_nodes.pop();

        // are we done?
        if (!current_node_or_none)
            break;

        current_node = *current_node_or_none;
        // process component stack
        component_stack.raise_level(graph.value(current_node));
    }

    return analyzer.get_result();
}

template <typename G, typename A>
void ComponentTreeParser<G,A>::ComponentStack::raise_level(ComponentTreeParser<G,A>::Value new_level) {
    while (graph_.less(level(0), new_level)) {
        // level of second last component (exists, since current_level < inf)
        if  (components_.size() == 1 || graph_.less(new_level, level(1))) {
            analyzer_.raise_level(component(0), new_level);
        } else {
            assert(graph_.less(level(0), level(1)));
            // is it correct to use level instead of next_level here?
            analyzer_.merge_component_into(component(0), component(1));
            components_.pop_back();
        }
    }
}

#endif // COMPONENTTREEPARSER_HPP



