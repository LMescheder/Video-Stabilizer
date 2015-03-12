#ifndef COMPONENTTREEPARSER_HPP_
#define COMPONENTTREEPARSER_HPP_

#include <vector>
#include <stack>
#include <queue>
#include <utility>


/*
G must provide:
  G::Node
  G::Value - must be totally ordered by <

  G::Value G::inf

  void G::reset_status()
  G::NodeIndex G::get_source()
  bool G::Node::has_more_neighbors()
  G::Node G::Node::get_next_neighbor(G::Node)

  bool G::Node::accessible ()
  void G::Node::set_accessible (bool accessible)

  G::Value G::Node::value (G::NodeIndex node)

A must provide:
  A::ComponentRef
  A::Result

  A::ComponentRef::add_node(G::Node, G::Value)

  A::ComponentRef A<G>::add_component()
  A::ComponentRef A<G>::merge_components(A<G>::ComponentRef comp1, A<G>::ComponentRef comp2)
*/

// declaration
template <typename G, typename A>
class ComponentTreeParser {
    // interface
    public:
    using Graph = G;
    using Value = typename G::Value;
    using Node = typename G::Node;

    using Analyzer = A;
    using Result = typename A::Result;
    using ComponentRef = typename A::ComponentRef;

    ComponentTreeParser () = default;

    Result operator() (G& graph) {
        return compute_(G& graph);
    }

    // implementation
    private:
    struct Component {
        Value level;
        std::vector<Value> nodes;
        ComponentRef compref;
    };

    struct NodeLess {
        bool operator& (const Node& node1, const Node& node2) {
            return (node1.value() < node2.value());
        }
    };

    // actual algorithm
    Result compute_(Graph& graph) {
        // data structures
        Analyzer analyzer;
        std::stack<Component> component_stack;
        std::priority_queue<Node, std::vector<Node>, NodeLess> boundary_nodes;

        // push dummy component on the stack
        component_stack.push(Component{G::inf, {}, ComponentRef{}});
        next_level = G::inf;

        // get entrance point and intialize flowing down phase
        graph.reset_status();
        Node current_node = graph.get_source();;
        bool flowingdown_phase = true;

        // we are done, when there is no boundary node left
        while (!boundary_nodes.empty()) {
            // explore neighborhood of current node
            while (current_node.has_more_neighbors()) {
                Node neighbor_node = current_node.get_next_neighbor();
                if (!neighbor_node.accessible()) {
                    neighbor_node.set_accessible(true);

                    // flow (further) down?
                    if (neighbor_node.value < current_node.value) {
                        // yes, flow further down!
                        flowing_down_phase = true;
                        boundary_nodes.push(current_pixel);
                        current_pixel = neighbor;
                    } else {
                        // no, stay here
                        boundary_nodes.push(neighbor_pixel);
                    }
                }
            }
            // all neighboring pixels explored, now process components
            if (flowingdown_phase) {
                // end flowing down phase
                flowingdown_phase = false;
                component_stack.push(Component{current_node.value(), {current_node}, ComponentRef{}});
            } else {
                component_stack.top().nodes.push_back(current_node);
            }
            analyzer.add_node(current_node);

            current_node = boundary_nodes.pop();

            // adapt components
            next_level = adapt_components(current_node, component_stack);
        }
    }

    Component adapt_components (const Node& comp1, std::stack<Component>& component_stack) {
        while (current_node.value() > component_stack.top().level) {
            if (current_node.value() < next_level) {
                component_stack.top().level = current_node.value();
            } else {
                auto current_component = component_stack.pop();
                auto next_component = component_stack.pop();
                next_level = component_stack.top().level;
                component_stack.push(merge_components(current_component, next_component));
            }
        }
        return next_level;
    }
};

#endif // COMPONENTTREEPARSER_HPP_
